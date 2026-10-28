#pragma once
#include <WebServer.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include "HttpFrameGate.h"

namespace Rover {
// The stock parser's plain-body helper waits 5 seconds even after a peer closes.
// Replay an already complete, owned frame so parser reads never wait on TCP and
// a reset between admission and parsing cannot reopen that blocking path.
class RequestSnapshotClient : public NetworkClient {
  const uint8_t *data_;
  size_t size_, offset_ = 0;
 public:
  RequestSnapshotClient(const uint8_t *data, size_t size)
      : NetworkClient(), data_(data), size_(size) {
    // Keep the base client socket-free as well: the stock parser also calls
    // nonvirtual NetworkClient::clear(), which must never drain the real peer.
    // All data is already owned. Allow normal task preemption between parser
    // reads; a 1ms Stream timeout can reject a complete frame after a task switch.
    setTimeout(HttpFrameGate::ADMISSION_MS);
  }
  int available() override { return int(size_ - offset_); }
  int read() override { return offset_ < size_ ? data_[offset_++] : -1; }
  int peek() override { return offset_ < size_ ? data_[offset_] : -1; }
  int read(uint8_t *buffer, size_t size) override {
    if (!buffer || !size) return 0;
    const size_t count = size < size_ - offset_ ? size : size_ - offset_;
    if (count) { memcpy(buffer, data_ + offset_, count); offset_ += count; }
    return int(count);
  }
  size_t readBytes(char *buffer, size_t size) override { return size_t(read(reinterpret_cast<uint8_t *>(buffer), size)); }
  size_t readBytes(uint8_t *buffer, size_t size) override { return size_t(read(buffer, size)); }
  uint8_t connected() override { return offset_ < size_; }
};

class BoundedWebServer : public WebServer {
  uint8_t frame_[HttpFrameGate::FRAME_LIMIT + 1];
  size_t frameSize_ = 0;
  void releaseClient() {
    _currentClient.stop();
    _currentClient = NetworkClient();
    _currentStatus = HC_NONE;
    _currentUpload.reset(); _currentRaw.reset();
    frameSize_ = 0;
  }
 public:
  explicit BoundedWebServer(int port = 80) : WebServer(port) {}
  void handleClient() override {
    if (_currentStatus == HC_NONE) {
      _currentClient = _server.accept();
      if (_currentClient.fd() < 0) return;
      _currentStatus = HC_WAIT_READ;
      _statusChange = millis();
    }
    if (HttpFrameGate::expired(uint32_t(millis()), uint32_t(_statusChange))) { releaseClient(); return; }
    const int socket = _currentClient.fd();
    if (socket < 0) { releaseClient(); return; }
    for (;;) {
      if (HttpFrameGate::expired(uint32_t(millis()), uint32_t(_statusChange))) { releaseClient(); return; }
      // Consume into our owned frame incrementally. lwIP MSG_PEEK can expose
      // only the first queued TCP buffer, so repeatedly peeking cannot assemble
      // a normal POST whose headers and body arrived in different packets.
      const int count = int(recv(socket, frame_ + frameSize_, sizeof frame_ - frameSize_, MSG_DONTWAIT));
      if (!count) { releaseClient(); return; }
      if (count < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) releaseClient();
        return;
      }
      frameSize_ += size_t(count);
      const FrameResult result = HttpFrameGate::inspect(frame_, frameSize_);
      if (result.status == FrameStatus::Reject) { releaseClient(); return; }
      if (result.status == FrameStatus::NeedMore) continue;
      // Reject already queued trailing input. A future pipelined request is
      // never parsed because this connection closes after its first response.
      uint8_t extra;
      if (recv(socket, &extra, 1, MSG_PEEK | MSG_DONTWAIT) > 0) { releaseClient(); return; }
      break;
    }
    if (HttpFrameGate::expired(uint32_t(millis()), uint32_t(_statusChange))) { releaseClient(); return; }
    RequestSnapshotClient snapshot(frame_, frameSize_);
    // The stock parser does not reset this field when a request omits the header.
    _clientContentLength = 0;
    if (_parseRequest(snapshot)) {
      _contentLength = CONTENT_LENGTH_NOT_SET;
      _responseCode = 0;
      _clearResponseHeaders();
      _handleRequest();
    }
    // One request per connection, including malformed/unknown routes. The rover
    // does not use SSE, uploads or streaming middleware.
    releaseClient();
  }
};
}
