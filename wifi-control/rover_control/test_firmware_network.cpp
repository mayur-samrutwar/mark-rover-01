// Pure host tests. No Arduino runtime, serial, network, GPIO or motors.
#include "../firmware/rover_wifi_control/NetworkRecovery.h"
#include "../firmware/rover_wifi_control/ControlCore.h"
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace Rover;
unsigned checks = 0;
#define CHECK(value) do { ++checks; if (!(value)) { \
  std::cerr << "check failed at line " << __LINE__ << ": " #value "\n"; std::exit(1); } } while (0)

void graceAndBackoff() {
  NetworkRecovery policy;
  CHECK(!policy.reserve(0, false, false, true).due);
  CHECK(!policy.reserve(29999, false, false, true).due);
  auto first = policy.reserve(30000, false, false, true);
  CHECK(first.due && !first.startStation);
  CHECK(policy.valid(first, 30001, false, false));
  CHECK(!policy.valid(first, 30001, true, false));
  CHECK(!policy.valid(first, 30001, false, true));
  CHECK(policy.nextBackoffMs() == 60000);
  CHECK(!policy.reserve(89999, false, false, true).due);
  CHECK(policy.reserve(90000, false, false, true).due);
  CHECK(policy.nextBackoffMs() == 120000);
  CHECK(!policy.reserve(209999, false, false, true).due);
  CHECK(policy.reserve(210000, false, false, true).due);
  CHECK(policy.nextBackoffMs() == 120000);
  CHECK(!policy.reserve(329999, false, false, true).due);
  CHECK(policy.reserve(330000, false, false, true).due);
}

void progressAndDhcp() {
  NetworkRecovery policy;
  CHECK(policy.event(NetworkEvent::Start, 100));
  CHECK(policy.event(NetworkEvent::Associated, 29000));
  CHECK(!policy.reserve(30000, false, false, true).due);
  CHECK(!policy.reserve(58999, false, false, true).due);
  // An associated station is never disconnected to "repair" slow/stuck DHCP.
  CHECK(!policy.reserve(59000, false, true, true).due);
  CHECK(!policy.reserve(900000, false, true, true).due);
  CHECK(policy.event(NetworkEvent::GotIP, 900001));
  CHECK(!policy.reserve(930001, true, true, true).due);
  CHECK(policy.event(NetworkEvent::Disconnected, 930002));
  CHECK(!policy.reserve(960001, false, false, true).due);
  CHECK(policy.reserve(960002, false, false, true).due);
}

void unlistedFailuresAndLateEvents() {
  NetworkRecovery policy;
  // Policy is reason-independent: unlisted reason36 cannot permanently strand
  // it, but repeated failures do not reset the progress grace indefinitely.
  CHECK(policy.event(NetworkEvent::Disconnected, 1000));
  CHECK(policy.event(NetworkEvent::Disconnected, 10000));
  CHECK(policy.event(NetworkEvent::Disconnected, 20000));
  auto attempt = policy.reserve(30000, false, false, true);
  CHECK(attempt.due);
  CHECK(policy.event(NetworkEvent::Associated, 30001));
  CHECK(!policy.valid(attempt, 30002, false, false));
  CHECK(!policy.reserve(59999, false, false, true).due);
  CHECK(policy.event(NetworkEvent::GotIP, 60000));
  CHECK(!policy.event(NetworkEvent::Disconnected, 59999));
  CHECK(!policy.reserve(60001, true, true, true).due);
  CHECK(policy.nextBackoffMs() == 30000);
  CHECK(policy.event(NetworkEvent::LostIP, 60002));
  CHECK(!policy.reserve(90001, false, false, true).due);
  CHECK(policy.reserve(90002, false, false, true).due);
}

void stoppedStationAndRecoveryNeverArms() {
  NetworkRecovery policy;
  Control control;
  CHECK(policy.event(NetworkEvent::Stop, 100));
  auto attempt = policy.reserve(30100, false, false, false);
  CHECK(attempt.due && attempt.startStation);
  CHECK(policy.event(NetworkEvent::Start, 30101));
  CHECK(!policy.valid(attempt, 30102, false, false));
  CHECK(policy.event(NetworkEvent::Associated, 30103));
  CHECK(policy.event(NetworkEvent::GotIP, 30104));
  CHECK(!policy.reserve(30105, true, true, true).due);
  CHECK(!control.armed && control.left == 0 && control.right == 0);
}

void wrapAndLongQuietConnection() {
  const uint32_t boot = std::numeric_limits<uint32_t>::max() - 10000;
  NetworkRecovery policy(boot);
  CHECK(!policy.reserve(uint32_t(boot + 29999), false, false, true).due);
  CHECK(policy.reserve(uint32_t(boot + 30000), false, false, true).due);
  CHECK(!policy.reserve(uint32_t(boot + 89999), false, false, true).due);
  CHECK(policy.reserve(uint32_t(boot + 90000), false, false, true).due);
  CHECK(policy.event(NetworkEvent::GotIP, uint32_t(boot + 90001)));
  CHECK(!policy.event(NetworkEvent::Disconnected, uint32_t(boot + 90000)));

  NetworkRecovery quiet;
  CHECK(quiet.event(NetworkEvent::GotIP, 1));
  // 60 days of an online poll clock, including the 49.7-day millis wrap. No
  // Wi-Fi events are required to keep its ordering reference current.
  uint64_t time = 1;
  for (unsigned hour = 0; hour < 60 * 24; ++hour) {
    time += 3600000;
    CHECK(!quiet.reserve(uint32_t(time), true, true, true).due);
  }
  CHECK(quiet.event(NetworkEvent::Disconnected, uint32_t(time + 1)));
  CHECK(!quiet.reserve(uint32_t(time + 30000), false, false, true).due);
  CHECK(quiet.reserve(uint32_t(time + 30001), false, false, true).due);
}

void disconnectCannotBeHiddenByFastReconnect() {
  DisconnectLatch latch;
  uint32_t consumed = 0;
  Control control;
  control.hello(1);
  CHECK(control.accept({1, 20, 20}, 2));  // simulated values only, no outputs
  CHECK(!latch.consume(consumed));
  latch.signal();                        // DISCONNECTED
  bool wifiUp = false;
  wifiUp = true;                        // GOT_IP before the next motor tick
  if (latch.consume(consumed)) control.stop();
  CHECK(wifiUp && !control.armed && control.left == 0 && control.right == 0);
  CHECK(!control.accept({2, 20, 20}, 3));
  CHECK(!latch.consume(consumed));
  control.hello(4);                      // only a new explicit lease can arm
  latch.signal();                       // LOST_IP
  latch.signal();                       // STA_STOP
  if (latch.consume(consumed)) control.stop();
  CHECK(!control.armed && !latch.consume(consumed));
  CHECK(TIMEOUT_MS == 350);
  control.hello(100);
  CHECK(!control.tick(449));
  CHECK(control.tick(450) && !control.armed);
}

void mdnsRetryIsBounded() {
  MdnsRetry retry;
  CHECK(retry.due(0));
  retry.result(0, false);
  CHECK(!retry.due(4999));
  CHECK(retry.due(5000));
  retry.result(5000, false);
  CHECK(!retry.due(14999));
  CHECK(retry.due(15000));
  retry.result(15000, true);
  CHECK(retry.active() && !retry.due(1000000));
  retry.reset();
  CHECK(!retry.active() && retry.due(1000001));
  uint32_t now = 1000001;
  for (unsigned i = 0; i < 10; ++i) {
    retry.result(now, false);
    CHECK(!retry.due(now));
    CHECK(retry.due(uint32_t(now + 60000)));
    now += 60000;
  }
  retry.reset();
  retry.result(std::numeric_limits<uint32_t>::max() - 1000, false);
  CHECK(!retry.due(3998));
  CHECK(retry.due(3999));
}

int main() {
  graceAndBackoff(); progressAndDhcp(); unlistedFailuresAndLateEvents();
  stoppedStationAndRecoveryNeverArms(); wrapAndLongQuietConnection();
  disconnectCannotBeHiddenByFastReconnect(); mdnsRetryIsBounded();
  std::cout << checks << " firmware network host checks passed; no hardware used\n";
}
