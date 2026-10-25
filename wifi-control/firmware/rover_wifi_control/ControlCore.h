#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>

namespace Rover {
constexpr uint32_t TIMEOUT_MS = 350;
constexpr long MAX_SEQUENCE = 2147483647L;
struct Command { uint32_t sequence; int left; int right; };

inline bool number(const char *&p, long minimum, long maximum, long &value) {
  while (*p == ' ') ++p;
  if (!isdigit(static_cast<unsigned char>(*p)) && *p != '-') return false;
  char *end;
  errno = 0;
  value = strtol(p, &end, 10);
  if (p == end || errno == ERANGE || value < minimum || value > maximum) return false;
  if (*end && *end != ' ') return false;
  p = end;
  return true;
}

inline bool parse(const char *line, Command &command) {
  if (line[0] != 'M' || line[1] != ' ') return false;
  const char *p = line + 2;
  long sequence, left, right;
  if (!number(p, 1, MAX_SEQUENCE, sequence) ||
      !number(p, -255, 255, left) || !number(p, -255, 255, right)) return false;
  while (*p == ' ') ++p;
  if (*p) return false;
  command = {uint32_t(sequence), int(left), int(right)};
  return true;
}

// A timeout disarms the session: late buffered movement cannot restart it.
struct Control {
  bool armed = false;
  uint32_t sequence = 0, lastCommandAt = 0;
  int left = 0, right = 0;
  void stop() { left = right = 0; armed = false; }
  void hello(uint32_t now) { stop(); sequence = 0; lastCommandAt = now; armed = true; }
  bool accept(const Command &command, uint32_t now) {
    if (!armed || command.sequence <= sequence) { stop(); return false; }
    sequence = command.sequence;
    left = command.left; right = command.right; lastCommandAt = now;
    return true;
  }
  bool tick(uint32_t now) {
    if (armed && uint32_t(now - lastCommandAt) >= TIMEOUT_MS) { stop(); return true; }
    return false;
  }
};
}
