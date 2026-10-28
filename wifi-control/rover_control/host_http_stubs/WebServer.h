// Minimal host Arduino API double. Sockets are local socketpairs, never devices.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <sys/socket.h>
#include <unistd.h>
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

extern uint32_t hostMillis;
inline uint32_t millis() { return hostMillis; }
using PGM_P = const char *;
enum HTTPClientStatus { HC_NONE, HC_WAIT_READ, HC_WAIT_CLOSE };
constexpr size_t CONTENT_LENGTH_NOT_SET = size_t(-2);

class NetworkClient {
  struct Handle { int socket; explicit Handle(int value) : socket(value) {} ~Handle() { if (socket >= 0) ::close(socket); } };
  std::shared_ptr<Handle> handle_;
 public:
  NetworkClient() = default;
  explicit NetworkClient(int fd) : handle_(std::make_shared<Handle>(fd)) {}
  virtual ~NetworkClient() = default;
  int fd() const { return handle_ ? handle_->socket : -1; }
  virtual int available() { throw std::runtime_error("Parser reached raw socket available()"); }
  virtual int read() { throw std::runtime_error("Parser reached raw socket read()"); }
  virtual int read(uint8_t *, size_t) { throw std::runtime_error("Parser reached raw socket read(buffer)"); }
  virtual int peek() { throw std::runtime_error("Parser reached raw socket peek()"); }
  virtual size_t readBytes(char *, size_t) { throw std::runtime_error("Parser reached raw socket readBytes(char)"); }
  virtual size_t readBytes(uint8_t *, size_t) { throw std::runtime_error("Parser reached raw socket readBytes(bytes)"); }
  virtual uint8_t connected() { return fd() >= 0; } // Mirrors the core's EOF bug.
  void clear() { if (fd() >= 0) throw std::runtime_error("Parser clear() reached the real socket"); }
  virtual void stop() { if (fd() >= 0) {::close(fd());handle_->socket=-1;} }
  void setTimeout(unsigned long) {}
};
class NetworkServer {
  std::deque<NetworkClient> queued_;
 public:
  void queue(NetworkClient client) { queued_.push_back(client); }
  NetworkClient accept() { if (queued_.empty()) return {}; auto client=queued_.front();queued_.pop_front();return client; }
};
class WebServer {
 public:
  std::vector<std::string> parsed;
  size_t handled=0;
  std::function<void()> beforeParse;
  explicit WebServer(int) {}
  virtual ~WebServer() = default;
  virtual void handleClient() {}
 protected:
  NetworkServer _server;
  NetworkClient _currentClient;
  HTTPClientStatus _currentStatus=HC_NONE;
  unsigned long _statusChange=0;
  std::unique_ptr<int> _currentUpload, _currentRaw;
  int _clientContentLength=0, _responseCode=0;
  size_t _contentLength=CONTENT_LENGTH_NOT_SET;
  void _clearResponseHeaders() {}
  bool _parseRequest(NetworkClient &client) {
    if (beforeParse) beforeParse();
    std::string value;
    if (client.available()) {
      const int peeked=client.peek(), first=client.read();
      if (peeked != first) throw std::runtime_error("Snapshot peek consumed input");
      value.push_back(char(first));
    }
    char first[11];
    const size_t one=client.readBytes(first,sizeof first);
    value.append(first,one);
    uint8_t rest[4096];
    const size_t two=client.readBytes(rest,sizeof rest);
    value.append(reinterpret_cast<char *>(rest),two);
    if (client.available() || client.read()!=-1 || client.readBytes(rest,1)!=0)
      throw std::runtime_error("Snapshot is not bounded at EOF");
    client.clear(); // Stock parser calls this nonvirtual method after parsing.
    if (_clientContentLength != 0) throw std::runtime_error("Stale Content-Length was retained");
    parsed.push_back(value);
    return true;
  }
  bool _handleRequest() {
    ++handled;
    const char response[]="HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nOK";
    // A peer may already be gone; no retries or waits are used in this double.
    ::send(_currentClient.fd(),response,sizeof response-1,0);
    return true;
  }
};
