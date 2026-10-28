#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <atomic>
#include "ControlCore.h"
#include "NetworkRecovery.h"
#include "BoundedWebServer.h"
#include "wifi_credentials.h"
#include "WebAssets.h"

constexpr uint8_t LEFT_EN = 0, LEFT_IN1 = 1, LEFT_IN2 = 3;
constexpr uint8_t RIGHT_EN = 4, RIGHT_IN1 = 5, RIGHT_IN2 = 6;
constexpr int LEFT_POLARITY = 1, RIGHT_POLARITY = -1;
constexpr uint32_t REVERSAL_PAUSE_MS = 40;
Rover::Control control;
bool pwmReady = false;
#include "MotorOutputs.h"

Rover::BoundedWebServer server(80);
SemaphoreHandle_t controlMutex;
std::atomic<bool> wifiUp{false};
Rover::DisconnectLatch disconnectLatch;
uint32_t consumedDisconnectGeneration = 0;  // protected by controlMutex
portMUX_TYPE networkMetadataMux = portMUX_INITIALIZER_UNLOCKED;
struct NetworkMetadata {
  Rover::NetworkRecovery recovery;
  uint32_t events = 0, starts = 0, associations = 0, gotIPs = 0;
  uint32_t disconnects = 0, lostIPs = 0, stops = 0;
  uint32_t lastEventAt = 0, lastDisconnectAt = 0, lastGotIPAt = 0;
  Rover::NetworkEvent lastEvent = Rover::NetworkEvent::Start;
  int lastReason = 0, protocolResult = 0, beginStatus = 0, powerResult = 0;
  uint32_t recoveryAttempts = 0, lastRecoveryAt = 0;
  int lastRecoveryStartResult = 0, lastRecoveryConnectResult = 0;
  uint32_t mdnsAttempts = 0, mdnsFailures = 0;
  bool mdnsReady = false;
} networkMetadata;
Rover::MdnsRetry mdnsRetry;
uint32_t mdnsDisconnectGeneration = 0;
bool mdnsTouched = false;
char bootId[17] = "";
esp_reset_reason_t bootResetReason;
char sessionToken[33] = "";
String sessionClient;

void lockControl() { xSemaphoreTake(controlMutex, portMAX_DELAY); }
void unlockControl() { xSemaphoreGive(controlMutex); }

// This runs only with controlMutex held. A lost link permanently disarms the
// previous lease even when GOT_IP arrives before the next motor-task tick.
void consumeNetworkStopLocked() {
  if (disconnectLatch.consume(consumedDisconnectGeneration)) stopMotors();
}

void noteNetworkEvent(Rover::NetworkEvent event, int reason = 0) {
  portENTER_CRITICAL(&networkMetadataMux);
  // Sample after acquiring the metadata lock: a callback waiting behind a
  // newer poll must not carry an older timestamp and lose its progress event.
  const uint32_t now = millis();
  networkMetadata.recovery.event(event, now);
  ++networkMetadata.events;
  networkMetadata.lastEvent = event;
  networkMetadata.lastEventAt = now;
  switch (event) {
    case Rover::NetworkEvent::Start: ++networkMetadata.starts; break;
    case Rover::NetworkEvent::Associated: ++networkMetadata.associations; break;
    case Rover::NetworkEvent::GotIP:
      ++networkMetadata.gotIPs; networkMetadata.lastGotIPAt = now;
      wifiUp.store(true); break;
    case Rover::NetworkEvent::Disconnected:
      ++networkMetadata.disconnects; networkMetadata.lastDisconnectAt = now;
      networkMetadata.lastReason = reason;
      wifiUp.store(false); disconnectLatch.signal(); break;
    case Rover::NetworkEvent::LostIP:
      ++networkMetadata.lostIPs;
      wifiUp.store(false); disconnectLatch.signal(); break;
    case Rover::NetworkEvent::Stop:
      ++networkMetadata.stops;
      wifiUp.store(false); disconnectLatch.signal(); break;
  }
  portEXIT_CRITICAL(&networkMetadataMux);
}

NetworkMetadata networkSnapshot() {
  portENTER_CRITICAL(&networkMetadataMux);
  NetworkMetadata result = networkMetadata;
  portEXIT_CRITICAL(&networkMetadataMux);
  return result;
}

// Runs independently of WebServer. Slow HTTP transfers cannot hold the motors on.
void motorTask(void *) {
  for (;;) {
    lockControl();
    consumeNetworkStopLocked();
    if (!wifiUp.load() || control.tick(millis())) stopMotors();
    if (pwmReady && control.armed) {
      uint32_t now = millis();
      if (!setMotor(leftMotor, control.left, now) || !setMotor(rightMotor, control.right, now)) {
        stopMotors(); pwmReady = false;
      }
    }
    unlockControl();
    vTaskDelay(max(TickType_t(1), pdMS_TO_TICKS(5)));
  }
}

String stateJson(const char *type) {
  char json[240];
  lockControl();
  snprintf(json, sizeof json,
    "{\"type\":\"%s\",\"protocol\":\"rover-wifi-v1\",\"ready\":%s,\"armed\":%s,\"seq\":%lu,\"left\":%d,\"right\":%d}",
    type, pwmReady ? "true" : "false", control.armed ? "true" : "false",
    (unsigned long)control.sequence, leftMotor.applied, rightMotor.applied);
  unlockControl();
  return String(json);
}
void jsonReply(int code, const String &json) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", json);
}
bool sameOrigin() {
  String origin = server.header("Origin");
  return !origin.length() || origin == "http://" + WiFi.localIP().toString() || origin == "http://rover.local";
}
bool authorized(bool beacon = false) {
  if (!sameOrigin()) { jsonReply(403, "{\"error\":\"Origin rejected\"}"); return false; }
  String token = server.header("X-Rover-Token");
  if (!token.length() && beacon) token = server.arg("plain");
  if (!sessionToken[0] || token != sessionToken) {
    jsonReply(403, "{\"error\":\"Session expired; reconnect\"}"); return false;
  }
  return true;
}
void newSession() {
  if (!sameOrigin()) { jsonReply(403, "{\"error\":\"Origin rejected\"}"); return; }
  String client = server.arg("plain");
  if (client.length() != 32) { jsonReply(400, "{\"error\":\"Invalid client\"}"); return; }
  for (char c : client) if (!isxdigit((unsigned char)c)) {
    jsonReply(400, "{\"error\":\"Invalid client\"}"); return;
  }
  lockControl();
  consumeNetworkStopLocked();
  bool busy = control.armed;
  // Never replace a live controller, even if another tab knows the network.
  if (!busy) {
    stopMotors();
    if (pwmReady && wifiUp.load()) control.hello(millis());
  }
  unlockControl();
  if (busy) { jsonReply(409, "{\"error\":\"Another controller is connected; disconnect it first\"}"); return; }
  sessionClient = client;
  for (int i = 0; i < 4; ++i) snprintf(sessionToken + i * 8, 9, "%08lx", (unsigned long)esp_random());
  String response = stateJson("hello");
  response.remove(response.length() - 1);
  response += ",\"token\":\"" + String(sessionToken) + "\"}";
  jsonReply(200, response);
}
void motorCommand() {
  if (!authorized()) return;
  String body = server.arg("plain");
  Rover::Command command;
  bool parsed = body.length() < 80 && Rover::parse(body.c_str(), command);
  lockControl();
  consumeNetworkStopLocked();
  // Check the lease here as well as in the independent watchdog task.
  control.tick(millis());
  bool accepted = parsed && pwmReady && wifiUp.load() && control.accept(command, millis());
  if (!accepted) stopMotors();
  unlockControl();
  jsonReply(accepted ? 200 : 409, stateJson(accepted ? "state" : "fault"));
}
void stopCommand() {
  if (!authorized(true)) return;
  lockControl(); stopMotors(); unlockControl();
  jsonReply(200, stateJson("stopped"));
}
const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_watchdog";
    case ESP_RST_TASK_WDT: return "task_watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    case ESP_RST_EFUSE: return "efuse";
    case ESP_RST_PWR_GLITCH: return "power_glitch";
    case ESP_RST_CPU_LOCKUP: return "cpu_lockup";
    default: return "unknown";
  }
}
const char *networkEventName(Rover::NetworkEvent event) {
  switch (event) {
    case Rover::NetworkEvent::Start: return "start";
    case Rover::NetworkEvent::Associated: return "associated";
    case Rover::NetworkEvent::GotIP: return "got_ip";
    case Rover::NetworkEvent::Disconnected: return "disconnected";
    case Rover::NetworkEvent::LostIP: return "lost_ip";
    case Rover::NetworkEvent::Stop: return "stop";
    default: return "unknown";
  }
}
String networkJson() {
  // Wi-Fi calls are deliberately outside both the motor and metadata locks.
  const NetworkMetadata data = networkSnapshot();
  const IPAddress address = WiFi.localIP();
  const int status = int(WiFi.status());
  wifi_ap_record_t ap;
  const bool associated = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
  const bool connected = wifiUp.load() && status == WL_CONNECTED && uint32_t(address) != 0;
  char rssi[16];
  if (associated) snprintf(rssi, sizeof rssi, "%d", int(ap.rssi));
  else snprintf(rssi, sizeof rssi, "null");
  const String ip = address.toString();
  char json[2048];
  snprintf(json, sizeof json,
    "{\"type\":\"network\",\"connected\":%s,\"associated\":%s,\"ip\":\"%s\",\"url\":\"http://%s/\",\"hostname\":\"rover.local\","
    "\"status\":%d,\"disconnectReason\":%d,\"disconnect_reason_historical\":true,\"rssi\":%s,"
    "\"boot_id\":\"%s\",\"uptime_ms\":%lu,\"reset_reason\":%d,\"reset_reason_name\":\"%s\","
    "\"event_count\":%lu,\"last_event\":\"%s\",\"last_event_at_ms\":%lu,"
    "\"start_count\":%lu,\"association_count\":%lu,\"got_ip_count\":%lu,\"disconnect_count\":%lu,\"lost_ip_count\":%lu,\"stop_count\":%lu,"
    "\"last_disconnect_at_ms\":%lu,\"last_got_ip_at_ms\":%lu,"
    "\"initial_begin_status\":%d,\"protocol_result\":%d,\"power_result\":%d,"
    "\"recovery_attempts\":%lu,\"last_recovery_at_ms\":%lu,\"last_recovery_start_result\":%d,\"last_recovery_connect_result\":%d,"
    "\"recovery_next_backoff_ms\":%lu,\"mdns_ready\":%s,\"mdns_attempts\":%lu,\"mdns_failures\":%lu}",
    connected ? "true" : "false", associated ? "true" : "false", ip.c_str(), ip.c_str(), status, data.lastReason, rssi,
    bootId, (unsigned long)millis(), int(bootResetReason), resetReasonName(bootResetReason),
    (unsigned long)data.events, data.events ? networkEventName(data.lastEvent) : "none", (unsigned long)data.lastEventAt,
    (unsigned long)data.starts, (unsigned long)data.associations, (unsigned long)data.gotIPs,
    (unsigned long)data.disconnects, (unsigned long)data.lostIPs, (unsigned long)data.stops,
    (unsigned long)data.lastDisconnectAt, (unsigned long)data.lastGotIPAt,
    data.beginStatus, data.protocolResult, data.powerResult, (unsigned long)data.recoveryAttempts,
    (unsigned long)data.lastRecoveryAt, data.lastRecoveryStartResult, data.lastRecoveryConnectResult,
    (unsigned long)data.recovery.nextBackoffMs(), data.mdnsReady ? "true" : "false",
    (unsigned long)data.mdnsAttempts, (unsigned long)data.mdnsFailures);
  return String(json);
}
String infoJson() {
  String result = stateJson("info");
  result.remove(result.length() - 1);
  return result + ",\"network\":" + networkJson() + "}";
}
// USB CDC accepts only a small burst when its write timeout is zero. Queue
// complete lines and drain a bounded chunk per loop so diagnostics neither
// truncate nor block the network/control loop when the host stops reading.
constexpr size_t SERIAL_QUEUE_CAPACITY = 4096;
char serialQueue[SERIAL_QUEUE_CAPACITY];
size_t serialQueueHead = 0, serialQueueSize = 0;
void queueSerialLine(const String &line, bool stopAck = false) {
  const size_t needed = line.length() + 1;
  const size_t reserve = stopAck ? 0 : 64;
  if (needed + reserve > SERIAL_QUEUE_CAPACITY - serialQueueSize) return;
  for (size_t i = 0; i < needed; ++i) {
    serialQueue[(serialQueueHead + serialQueueSize) % SERIAL_QUEUE_CAPACITY] =
      i < line.length() ? line[i] : '\n';
    ++serialQueueSize;
  }
}
void serviceSerialOutput() {
  const int available = Serial.availableForWrite();
  if (!serialQueueSize || available <= 0) return;
  size_t count = min(size_t(available), min(size_t(128), serialQueueSize));
  count = min(count, SERIAL_QUEUE_CAPACITY - serialQueueHead);
  const size_t written = Serial.write(
    reinterpret_cast<const uint8_t *>(serialQueue + serialQueueHead), count);
  serialQueueHead = (serialQueueHead + written) % SERIAL_QUEUE_CAPACITY;
  serialQueueSize -= written;
}
void info() { queueSerialLine(networkJson()); }

void serviceMdns(uint32_t now, bool connected) {
  const uint32_t generation = disconnectLatch.generation();
  if (generation != mdnsDisconnectGeneration || !connected) {
    if (mdnsTouched) MDNS.end();
    mdnsTouched = false;
    mdnsRetry.reset();
    mdnsDisconnectGeneration = generation;
    portENTER_CRITICAL(&networkMetadataMux);
    networkMetadata.mdnsReady = false;
    portEXIT_CRITICAL(&networkMetadataMux);
  }
  if (!connected || !mdnsRetry.due(now)) return;
  // begin() can partially initialize before failing. Free that state before
  // every attempt and on any failure; addService is called only once per start.
  MDNS.end();
  bool success = MDNS.begin("rover");
  if (success) success = MDNS.addService("http", "tcp", 80);
  if (!success) MDNS.end();
  mdnsTouched = success;
  mdnsRetry.result(now, success);
  portENTER_CRITICAL(&networkMetadataMux);
  ++networkMetadata.mdnsAttempts;
  if (!success) ++networkMetadata.mdnsFailures;
  networkMetadata.mdnsReady = success;
  portEXIT_CRITICAL(&networkMetadataMux);
}

void serviceNetwork(uint32_t now) {
  static uint32_t lastServiceAt = 0;
  if (Rover::elapsedMs(now, lastServiceAt) < 250) return;
  lastServiceAt = now;
  const bool connected = wifiUp.load() && WiFi.status() == WL_CONNECTED && uint32_t(WiFi.localIP()) != 0;
  wifi_ap_record_t ap;
  const bool associated = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
  const bool started = WiFi.status() != WL_STOPPED;
  portENTER_CRITICAL(&networkMetadataMux);
  const Rover::RecoveryAttempt attempt = networkMetadata.recovery.reserve(now, connected, associated, started);
  portEXIT_CRITICAL(&networkMetadataMux);
  if (attempt.due) {
    // Recheck after reserving: a late association/GOT_IP cancels the attempt.
    // esp_wifi_connect never deliberately disconnects an existing association.
    const bool freshAssociated = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
    const bool freshConnected = wifiUp.load() && WiFi.status() == WL_CONNECTED;
    portENTER_CRITICAL(&networkMetadataMux);
    const bool valid = networkMetadata.recovery.valid(attempt, millis(), freshConnected, freshAssociated);
    if (valid) { ++networkMetadata.recoveryAttempts; networkMetadata.lastRecoveryAt = now; }
    portEXIT_CRITICAL(&networkMetadataMux);
    if (valid) {
      const esp_err_t startResult = attempt.startStation ? esp_wifi_start() : ESP_OK;
      const esp_err_t connectResult = startResult == ESP_OK ? esp_wifi_connect() : startResult;
      portENTER_CRITICAL(&networkMetadataMux);
      networkMetadata.lastRecoveryStartResult = int(startResult);
      networkMetadata.lastRecoveryConnectResult = int(connectResult);
      portEXIT_CRITICAL(&networkMetadataMux);
    }
  }
  serviceMdns(now, connected);
}
void setup() {
  const uint8_t pins[] = {LEFT_EN, RIGHT_EN, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2};
  for (uint8_t pin : pins) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
  bool leftReady = ledcAttach(LEFT_EN, 1000, 8), rightReady = ledcAttach(RIGHT_EN, 1000, 8);
  if (leftReady) ledcWrite(LEFT_EN, 0);
  if (rightReady) ledcWrite(RIGHT_EN, 0);
  pwmReady = leftReady && rightReady;
  stopMotors();
  Serial.begin(115200);
  controlMutex = xSemaphoreCreateMutex();
  if (!controlMutex || xTaskCreate(motorTask, "motor-watchdog", 4096, nullptr, 2, nullptr) != pdPASS) {
    stopMotors(); pwmReady = false;
    for (;;) delay(1000);
  }
  Serial.setTxTimeoutMs(0);
  bootResetReason = esp_reset_reason();
  snprintf(bootId, sizeof bootId, "%08lx%08lx", (unsigned long)esp_random(), (unsigned long)esp_random());
  networkMetadata.recovery = Rover::NetworkRecovery(millis());
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
      case ARDUINO_EVENT_WIFI_STA_START: noteNetworkEvent(Rover::NetworkEvent::Start); break;
      case ARDUINO_EVENT_WIFI_STA_CONNECTED: noteNetworkEvent(Rover::NetworkEvent::Associated); break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP: noteNetworkEvent(Rover::NetworkEvent::GotIP); break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        noteNetworkEvent(Rover::NetworkEvent::Disconnected, info.wifi_sta_disconnected.reason); break;
      case ARDUINO_EVENT_WIFI_STA_LOST_IP: noteNetworkEvent(Rover::NetworkEvent::LostIP); break;
      case ARDUINO_EVENT_WIFI_STA_STOP: noteNetworkEvent(Rover::NetworkEvent::Stop); break;
      default: break;
    }
  });
  WiFi.setHostname("rover");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  // This router/board link drops beacons in the default 802.11n mode.
  // 802.11b/g keeps the same WPA2 security and is ample for control traffic.
  const int protocolResult = int(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G));
  const int beginStatus = int(WiFi.begin(WIFI_SSID, WIFI_PASSWORD));
  // Conservative RF power for the compact board and USB power source.
  const int powerResult = WiFi.setTxPower(WIFI_POWER_8_5dBm) ? 0 : -1;
  portENTER_CRITICAL(&networkMetadataMux);
  networkMetadata.protocolResult = protocolResult;
  networkMetadata.beginStatus = beginStatus;
  networkMetadata.powerResult = powerResult;
  portEXIT_CRITICAL(&networkMetadataMux);
  const char *headers[] = {"Origin", "X-Rover-Token"};
  server.collectHeaders(headers, 2);
  server.on("/", HTTP_GET, [] { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/style.css", HTTP_GET, [] { server.send_P(200, "text/css", STYLE_CSS); });
  server.on("/app.mjs", HTTP_GET, [] { server.send_P(200, "text/javascript", APP_JS); });
  server.on("/controls.mjs", HTTP_GET, [] { server.send_P(200, "text/javascript", CONTROLS_JS); });
  server.on("/wifi-link.mjs", HTTP_GET, [] { server.send_P(200, "text/javascript", WIFI_LINK_JS); });
  server.on("/api/session", HTTP_POST, newSession);
  server.on("/api/command", HTTP_POST, motorCommand);
  server.on("/api/stop", HTTP_POST, stopCommand);
  server.on("/api/info", HTTP_GET, [] { jsonReply(200, infoJson()); });
  server.onNotFound([] { server.send(404, "text/plain", "Not found"); });
  server.begin();
}
void loop() {
  static bool wasConnected = false;
  bool connected = wifiUp.load();
  if (connected != wasConnected) {
    wasConnected = connected;
    info();
  }
  // Give core association/DHCP progress its grace; fallback never disconnects
  // an associated station and never acquires or restores a motor session.
  serviceNetwork(millis());
  server.handleClient();
  static char line[32]; static size_t length = 0;
  for (int budget = 0; budget < 32 && Serial.available(); ++budget) {
    char c = char(Serial.read());
    if (c == '\n') {
      line[length] = 0;
      if (!strcmp(line, "INFO")) info();
      if (!strcmp(line, "STOP")) { lockControl(); stopMotors(); unlockControl(); queueSerialLine("STOPPED", true); }
      length = 0;
    } else if (c != '\r') {
      if (length < sizeof(line) - 1) line[length++] = c;
      else length = 0;
    }
  }
  serviceSerialOutput();
  delay(1);
}
