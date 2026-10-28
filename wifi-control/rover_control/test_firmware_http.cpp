// Host framing and real socketpair admission tests. No Arduino device or motor.
#include <sys/socket.h>
#include <stddef.h>
static size_t receiveChunkLimit = size_t(-1);
// lwIP may expose one queued packet per recv/peek, unlike a coalescing host
// socket. Limit each read so a peeking-only implementation cannot pass this.
static ssize_t fragmentedRecv(int fd, void *buffer, size_t size, int flags) {
  return ::recv(fd, buffer, size < receiveChunkLimit ? size : receiveChunkLimit, flags);
}
#define recv fragmentedRecv
#include "../firmware/rover_wifi_control/BoundedWebServer.h"
#undef recv
#include <cstdlib>
#include <iostream>
#include <limits>
#include <signal.h>
#include <string>

uint32_t hostMillis=0;
unsigned checks=0;
#define CHECK(value) do { ++checks; if (!(value)) { \
  std::cerr << "check failed at line " << __LINE__ << ": " #value "\n"; std::exit(1); } } while (0)
using namespace Rover;
FrameResult inspect(const std::string &value) { return HttpFrameGate::inspect(reinterpret_cast<const uint8_t *>(value.data()),value.size()); }
std::string post(const std::string &body, const std::string &extra="") {
  return "POST /api/command HTTP/1.1\r\nHost: rover.local\r\nContent-Type: text/plain;charset=UTF-8\r\n"+extra+
    "Content-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
}
const std::string get="GET /api/info HTTP/1.1\r\nHost: rover.local\r\n\r\n";

void framing() {
  CHECK(inspect(get).status==FrameStatus::Ready);
  const std::string valid=post("M 1 128 -128");
  for (size_t i=0;i<valid.size();++i) CHECK(inspect(valid.substr(0,i)).status==FrameStatus::NeedMore);
  CHECK(inspect(valid).status==FrameStatus::Ready);
  CHECK(inspect(valid).length==valid.size());
  CHECK(inspect("HEAD / HTTP/1.0\r\n\r\n").status==FrameStatus::Ready);
  CHECK(inspect("POST /api/stop HTTP/1.1\r\nContent-Length: 0\r\n\r\n").status==FrameStatus::Ready);
  CHECK(inspect("POST /api/stop HTTP/1.1\r\n\r\n").status==FrameStatus::Ready);
  CHECK(inspect(post(std::string(1024,'x'))).status==FrameStatus::Ready);
  const std::string largestBody(1024,'x');
  const size_t fixedHeader=post(largestBody,"X-Pad: \r\n").size()-largestBody.size();
  const std::string largest=post(largestBody,"X-Pad: "+std::string(HttpFrameGate::HEADER_LIMIT-fixedHeader,'x')+"\r\n");
  CHECK(largest.size()==HttpFrameGate::FRAME_LIMIT && inspect(largest).status==FrameStatus::Ready);
  CHECK(inspect(post(std::string(1025,'x'))).status==FrameStatus::Reject);
  const std::string bad[]={
    "GET /api/info HTTP/1.1\nHost: rover\n\n", "GET /api/info HTTP/1.1\rX", "GET\t/api/info HTTP/1.1\r\n\r\n",
    "PUT /api/command HTTP/1.1\r\n\r\n", "GET http://rover/ HTTP/1.1\r\n\r\n", "GET / HTTP/2.0\r\n\r\n",
    "GET / HTTP/1.1\r\n Content-Length: 0\r\n\r\n", "GET / HTTP/1.1\r\nBad Name: x\r\n\r\n",
    "GET / HTTP/1.1\r\nContent-Length: 1\r\n\r\nx", get+get, post("x")+"x",
    post("", "Transfer-Encoding: chunked\r\n"), post("", "transfer-encoding:\r\n"),
    post("", "Expect: 100-continue\r\n"), post("", "Content-Type: multipart/form-data;boundary=x\r\n"),
    post("", "Content-Type: MULTIPART/FORM-DATA;boundary=x\r\n"), post("", "content-length: 0\r\n"),
    "POST /api/command HTTP/1.1\r\nContent-Type: multipart/form-data;boundary=x\r\nContent-Length: 0\r\n\r\n",
    "POST /api/command HTTP/1.1\r\nContent-Type: MULTIPART/FORM-DATA\r\nContent-Length: 0\r\n\r\n",
    post("", "Host: other\r\n"), post("", "Origin: x\r\norigin: y\r\n"),
    post("", "X-Rover-Token: x\r\nx-rover-token: y\r\n"),
    post(std::string("M 1 0 0\0ignored",sizeof("M 1 0 0\0ignored")-1)),
    std::string("GET / HTTP/1.1\r\nX: a\0b\r\n\r\n",sizeof("GET / HTTP/1.1\r\nX: a\0b\r\n\r\n")-1),
    std::string(HttpFrameGate::HEADER_LIMIT,'x'), std::string(HttpFrameGate::FRAME_LIMIT+1,'x'),
  };
  for (const auto &value:bad) CHECK(inspect(value).status==FrameStatus::Reject);
  for (const std::string &length:{"+1","-1","1x","1,1","","9999999999999999999999999","1025"}) {
    CHECK(inspect("POST /api/stop HTTP/1.1\r\nContent-Length: "+length+"\r\n\r\n").status==FrameStatus::Reject);
  }
  CHECK(!HttpFrameGate::expired(149,0)); CHECK(HttpFrameGate::expired(150,0));
  const uint32_t before=std::numeric_limits<uint32_t>::max()-50;
  CHECK(!HttpFrameGate::expired(uint32_t(before+149),before)); CHECK(HttpFrameGate::expired(uint32_t(before+150),before));
}

class TestServer : public BoundedWebServer {
 public:
  int queue(const std::string &input, bool closePeer=false) {
    int sockets[2]; CHECK(::socketpair(AF_UNIX,SOCK_STREAM,0,sockets)==0);
    if (!input.empty()) CHECK(::send(sockets[1],input.data(),input.size(),0)==ssize_t(input.size()));
    _server.queue(NetworkClient(sockets[0]));
    if (closePeer) {::close(sockets[1]);return -1;}
    return sockets[1];
  }
};
void admission() {
  hostMillis=0;
  TestServer server;
  server.queue("GET /api/info HT",true);
  const int healthy=server.queue(get);
  server.handleClient(); CHECK(server.handled==0);
  // Consuming the partial frame exposes FIN immediately; the healthy queued
  // request can be served on the next loop without the admission timeout.
  hostMillis=1; server.handleClient(); CHECK(server.handled==1);
  CHECK(server.parsed.back()==get);
  char response[256]; const ssize_t count=::recv(healthy,response,sizeof response,MSG_DONTWAIT);
  CHECK(count>0 && std::string(response,size_t(count)).find("200 OK")!=std::string::npos); ::close(healthy);

  // Empty peer EOF is discarded immediately, rather than occupying a 5s slot.
  TestServer eof;
  eof.queue("",true); const int next=eof.queue(get);
  eof.handleClient(); eof.handleClient(); CHECK(eof.handled==1); ::close(next);

  // Byte progress never resets the absolute admission deadline.
  TestServer trickle;
  hostMillis=0; const int peer=trickle.queue("G"); trickle.handleClient();
  hostMillis=100; CHECK(::send(peer,"E",1,0)==1); trickle.handleClient();
  hostMillis=150; trickle.handleClient(); CHECK(trickle.handled==0); ::close(peer);
  const int after=trickle.queue(get); trickle.handleClient(); CHECK(trickle.handled==1); ::close(after);

  // Headers alone are never passed to the parser while a POST body is missing.
  TestServer body;
  hostMillis=0; const std::string full=post("M 1 10 10");
  const int writer=body.queue(full.substr(0,full.size()-3)); body.handleClient(); CHECK(body.handled==0);
  CHECK(::send(writer,full.data()+full.size()-3,3,0)==3);
  hostMillis=20; body.handleClient(); CHECK(body.handled==1); CHECK(body.parsed.back()==full); ::close(writer);

  // Every socket receive can return just one tiny packet. A complete POST must
  // still assemble across them rather than repeatedly peeking the first one.
  TestServer fragmented;
  receiveChunkLimit=7;
  const std::string packetBody=post("M 2 0 0");
  const int packetPeer=fragmented.queue(packetBody);
  fragmented.handleClient(); CHECK(fragmented.handled==1);
  CHECK(fragmented.parsed.back()==packetBody);
  receiveChunkLimit=size_t(-1); ::close(packetPeer);

  // TCP disappearance after frame consumption cannot affect parser reads.
  TestServer reset;
  const int gone=reset.queue(post("M 1 0 0"));
  reset.beforeParse=[gone](){::close(gone);};
  reset.handleClient(); CHECK(reset.handled==1); CHECK(reset.parsed.back()==post("M 1 0 0"));

  // New bytes arriving after admission remain on the real socket. The stock
  // parser's nonvirtual clear() must act on a socket-free snapshot base object.
  TestServer late;
  const int lateWriter=late.queue(get);
  late.beforeParse=[lateWriter](){CHECK(::send(lateWriter,"late bytes",10,0)==10);};
  late.handleClient(); CHECK(late.handled==1); CHECK(late.parsed.back()==get); ::close(lateWriter);

  // Malformed input is discarded without entering the existing parser.
  TestServer reject;
  const int bad=reject.queue(post("", "Transfer-Encoding: chunked\r\n"));
  const int good=reject.queue(get);
  reject.handleClient(); CHECK(reject.handled==0); reject.handleClient(); CHECK(reject.handled==1);
  ::close(bad); ::close(good);
}
int main() {
  ::signal(SIGPIPE,SIG_IGN);
  framing(); admission();
  std::cout << checks << " firmware HTTP host checks passed; no hardware used\n";
}
