"""Local UI + Wi-Fi relay. Bind only to loopback; never generates motor commands."""
import argparse, json
from pathlib import Path
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--rover',default='http://192.168.1.6')
parser.add_argument('--port',type=int,default=8765)
args=parser.parse_args()
root=Path(__file__).resolve().parent
allowed={'/api/session','/api/command','/api/stop','/api/info'}
class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*values,**kwargs):super().__init__(*values,directory=str(root),**kwargs)
    def log_message(self,*args):pass
    def end_headers(self):
        self.send_header('Cache-Control','no-store')
        super().end_headers()
    def relay(self):
        if self.path not in allowed:self.send_error(404);return
        origin=self.headers.get('Origin')
        if origin and origin not in {f'http://127.0.0.1:{args.port}',f'http://localhost:{args.port}'}:
            self.send_error(403);return
        try:
            length=int(self.headers.get('Content-Length','0'))
            if not 0<=length<=1024:raise ValueError()
        except ValueError:self.send_error(400);return
        headers={'Content-Type':'text/plain','Connection':'close'}
        token=self.headers.get('X-Rover-Token')
        if token:headers['X-Rover-Token']=token
        body=self.rfile.read(length) if self.command=='POST' else None
        request=Request(args.rover.rstrip('/')+self.path,data=body,headers=headers,method=self.command)
        try:
            try:response=urlopen(request,timeout=2)
            except HTTPError as error:response=error
            with response:status=response.status;data=response.read(8192)
        except (OSError,URLError):
            status=502;data=json.dumps({'error':'Rover unavailable over Wi-Fi. Check power and home network.'}).encode()
        self.send_response(status);self.send_header('Content-Type','application/json')
        self.send_header('Content-Length',str(len(data)));self.end_headers()
        try:self.wfile.write(data)
        except (BrokenPipeError,ConnectionResetError):pass
    def do_POST(self):self.relay()
    def do_GET(self):
        if self.path.startswith('/api/'):self.relay()
        elif self.path=='/' or self.path.split('?')[0] in {'/index.html','/style.css','/app.mjs','/controls.mjs','/wifi-link.mjs'}:
            super().do_GET()
        else:self.send_error(404)
Handler.extensions_map={**SimpleHTTPRequestHandler.extensions_map,'.mjs':'text/javascript'}
print(f'Rover control: http://127.0.0.1:{args.port}/ → {args.rover}',flush=True)
ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()
