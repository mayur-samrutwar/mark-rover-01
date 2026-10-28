#pragma once
#include <stddef.h>
#include <stdint.h>

namespace Rover {
enum class FrameStatus { NeedMore, Ready, Reject };
struct FrameResult { FrameStatus status; size_t length = 0; };

// The rover serves small, single HTTP requests. No uploads, streaming bodies,
// pipelining, interim responses or multipart forms are needed by its protocol.
class HttpFrameGate {
 public:
  static constexpr size_t HEADER_LIMIT = 2048;
  static constexpr size_t BODY_LIMIT = 1024;
  static constexpr size_t FRAME_LIMIT = HEADER_LIMIT + BODY_LIMIT;
  static constexpr uint32_t ADMISSION_MS = 150;
 private:
  struct View { const uint8_t *data; size_t size; };
  static bool equal(View value, const char *literal, bool insensitive = false) {
    size_t i = 0;
    for (; literal[i]; ++i) {
      if (i >= value.size) return false;
      uint8_t a = value.data[i], b = uint8_t(literal[i]);
      if (insensitive) {
        if (a >= 'A' && a <= 'Z') a += 'a' - 'A';
        if (b >= 'A' && b <= 'Z') b += 'a' - 'A';
      }
      if (a != b) return false;
    }
    return i == value.size;
  }
  static bool token(uint8_t c) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) return true;
    const char *extra = "!#$%&'*+-.^_`|~";
    for (; *extra; ++extra) if (c == uint8_t(*extra)) return true;
    return false;
  }
  static View trim(View value) {
    while (value.size && (value.data[0] == ' ' || value.data[0] == '\t')) { ++value.data; --value.size; }
    while (value.size && (value.data[value.size - 1] == ' ' || value.data[value.size - 1] == '\t')) --value.size;
    return value;
  }
  static FrameResult lineEnd(const uint8_t *data, size_t size, size_t start) {
    for (size_t i = start; i < size; ++i) {
      const uint8_t c = data[i];
      if (c == '\r') {
        if (i + 1 == size) return {FrameStatus::NeedMore};
        return data[i + 1] == '\n' ? FrameResult{FrameStatus::Ready, i} : FrameResult{FrameStatus::Reject};
      }
      if (c == '\n' || c == 0 || c == 127 || (c < 32 && c != '\t')) return {FrameStatus::Reject};
    }
    return {FrameStatus::NeedMore};
  }
 public:
  static bool expired(uint32_t now, uint32_t acceptedAt) { return uint32_t(now - acceptedAt) >= ADMISSION_MS; }
  static FrameResult inspect(const uint8_t *data, size_t size) {
    if (size > FRAME_LIMIT || (size && !data)) return {FrameStatus::Reject};
    FrameResult line = lineEnd(data, size, 0);
    if (line.status != FrameStatus::Ready)
      return size >= HEADER_LIMIT ? FrameResult{FrameStatus::Reject} : line;
    if (!line.length || line.length > 512) return {FrameStatus::Reject};
    size_t firstSpace = 0;
    while (firstSpace < line.length && data[firstSpace] != ' ') ++firstSpace;
    if (firstSpace == line.length) return {FrameStatus::Reject};
    const View method{data, firstSpace};
    const bool post = equal(method, "POST");
    if (!post && !equal(method, "GET") && !equal(method, "HEAD")) return {FrameStatus::Reject};
    const size_t target = firstSpace + 1;
    size_t secondSpace = target;
    while (secondSpace < line.length && data[secondSpace] != ' ') ++secondSpace;
    if (target == secondSpace || secondSpace == line.length || data[target] != '/') return {FrameStatus::Reject};
    for (size_t i = target; i < secondSpace; ++i)
      if (data[i] < 33 || data[i] > 126) return {FrameStatus::Reject};
    const View version{data + secondSpace + 1, line.length - secondSpace - 1};
    if (!equal(version, "HTTP/1.1") && !equal(version, "HTTP/1.0")) return {FrameStatus::Reject};
    size_t offset = line.length + 2, body = 0, headerCount = 0;
    bool lengthSeen = false, hostSeen = false, originSeen = false, tokenSeen = false, typeSeen = false;
    for (;;) {
      if (offset > HEADER_LIMIT) return {FrameStatus::Reject};
      line = lineEnd(data, size, offset);
      if (line.status != FrameStatus::Ready)
        return size >= HEADER_LIMIT ? FrameResult{FrameStatus::Reject} : line;
      const size_t end = line.length;
      if (end + 2 > HEADER_LIMIT) return {FrameStatus::Reject};
      if (end == offset) {
        const size_t frame = end + 2 + body;
        if (!post && body) return {FrameStatus::Reject};
        if (size > frame) return {FrameStatus::Reject};
        for (size_t i = end + 2; i < size; ++i) if (data[i] == 0) return {FrameStatus::Reject};
        return size == frame ? FrameResult{FrameStatus::Ready, frame} : FrameResult{FrameStatus::NeedMore};
      }
      if (++headerCount > 32) return {FrameStatus::Reject};
      size_t colon = offset;
      while (colon < end && data[colon] != ':') {
        if (!token(data[colon])) return {FrameStatus::Reject};
        ++colon;
      }
      if (colon == offset || colon == end) return {FrameStatus::Reject};
      const View name{data + offset, colon - offset};
      const View value = trim({data + colon + 1, end - colon - 1});
      if (equal(name, "Transfer-Encoding", true) || equal(name, "Expect", true)) return {FrameStatus::Reject};
      if (equal(name, "Content-Length", true)) {
        if (lengthSeen || !value.size) return {FrameStatus::Reject};
        lengthSeen = true;
        for (size_t i = 0; i < value.size; ++i) {
          const uint8_t c = value.data[i];
          if (c < '0' || c > '9' || body > (BODY_LIMIT - size_t(c - '0')) / 10) return {FrameStatus::Reject};
          body = body * 10 + size_t(c - '0');
        }
      } else if (equal(name, "Host", true)) {
        if (hostSeen || !value.size) return {FrameStatus::Reject};
        hostSeen = true;
      } else if (equal(name, "Origin", true)) {
        if (originSeen) return {FrameStatus::Reject};
        originSeen = true;
      } else if (equal(name, "X-Rover-Token", true)) {
        if (tokenSeen) return {FrameStatus::Reject};
        tokenSeen = true;
      } else if (equal(name, "Content-Type", true)) {
        if (typeSeen) return {FrameStatus::Reject};
        typeSeen = true;
        size_t mediaEnd = 0;
        while (mediaEnd < value.size && value.data[mediaEnd] != ';') ++mediaEnd;
        if (!equal(trim({value.data, mediaEnd}), "text/plain", true)) return {FrameStatus::Reject};
      }
      offset = end + 2;
    }
  }
};
}
