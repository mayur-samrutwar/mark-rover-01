#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <atomic>
#include "ControlCore.h"
#include "wifi_credentials.h"
#include "WebAssets.h"

constexpr uint8_t LEFT_EN = 0, LEFT_IN1 = 1, LEFT_IN2 = 3;
constexpr uint8_t RIGHT_EN = 4, RIGHT_IN1 = 5, RIGHT_IN2 = 6;
constexpr int LEFT_POLARITY = 1, RIGHT_POLARITY = -1;
constexpr uint32_t REVERSAL_PAUSE_MS = 40;
Rover::Control control;
bool pwmReady = false;
#include "MotorOutputs.h"

WebServer server(80);
SemaphoreHandle_t controlMutex;
std::atomic<bool> wifiUp{false};
std::atomic<int> wifiReason{0};
char sessionToken[33] = "";
String sessionClient;

void lockControl() { xSemaphoreTake(controlMutex, portMAX_DELAY); }
void unlockControl() { xSemaphoreGive(controlMutex); }

// Runs independently of WebServer. Slow HTTP transfers cannot hold the motors on.
void motorTask(void *) {
  for (;;) {
    lockControl();
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
void info() {
  Serial.printf("{\"type\":\"network\",\"connected\":%s,\"ip\":\"%s\",\"url\":\"http://%s/\",\"hostname\":\"rover.local\",\"status\":%d,\"disconnectReason\":%d,\"rssi\":%d}\n",
    wifiUp.load() ? "true" : "false", WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str(),
    int(WiFi.status()), wifiReason.load(), int(WiFi.RSSI()));
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
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) wifiUp.store(true);
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      wifiUp.store(false); wifiReason.store(info.wifi_sta_disconnected.reason);
    }
  });
  WiFi.setHostname("rover");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  // This router/board link drops beacons in the default 802.11n mode.
  // 802.11b/g keeps the same WPA2 security and is ample for control traffic.
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Conservative RF power for the compact board and USB power source.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
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
  server.on("/api/info", HTTP_GET, [] { jsonReply(200, stateJson("info")); });
  server.onNotFound([] { server.send(404, "text/plain", "Not found"); });
  server.begin();
}
void loop() {
  static bool wasConnected = false;
  bool connected = wifiUp.load();
  if (connected != wasConnected) {
    wasConnected = connected;
    if (connected) { MDNS.begin("rover"); MDNS.addService("http", "tcp", 80); }
    else MDNS.end();
    info();
  }
  // The core reconnects after connection loss. Forcing reconnect while DHCP is
  // still pending can disconnect an already associated station before it gets IP.
  server.handleClient();
  static char line[32]; static size_t length = 0;
  for (int budget = 0; budget < 32 && Serial.available(); ++budget) {
    char c = char(Serial.read());
    if (c == '\n') {
      line[length] = 0;
      if (!strcmp(line, "INFO")) info();
      if (!strcmp(line, "STOP")) { lockControl(); stopMotors(); unlockControl(); Serial.println("STOPPED"); }
      length = 0;
    } else if (c != '\r') {
      if (length < sizeof(line) - 1) line[length++] = c;
      else length = 0;
    }
  }
  delay(1);
}
