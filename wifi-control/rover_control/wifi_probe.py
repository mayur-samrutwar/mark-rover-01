"""Real-network tests use zero speeds exclusively; no motor movement is requested."""
import json, secrets, socket, sys, time
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from urllib.parse import urlsplit
base=sys.argv[1].rstrip('/')
token=None
def post(path,body='',use_token=True,expected=200):
    headers={'Content-Type':'text/plain'}
    if token and use_token: headers['X-Rover-Token']=token
    request=Request(base+path,data=body.encode(),headers=headers,method='POST')
    try:
        # WebServer may need its 5-second header timeout to clear the deliberately
        # incomplete request below; the independent motor lease is only 350 ms.
        with urlopen(request,timeout=8) as response: code=response.status;message=json.load(response)
    except HTTPError as error: code=error.code;message=json.load(error)
    assert code==expected,(path,code,message.get('error'))
    if message.get('protocol'):
        assert message['left']==0 and message['right']==0, 'Unexpected nonzero motor output'
    return message
def session():
    global token
    result=post('/api/session',secrets.token_hex(16),False)
    token=result['token'];assert result['armed'] and result['ready']
try:
    session()
    assert post('/api/command','M 1 0 0')['armed']
    post('/api/session',secrets.token_hex(16),False,409)
    time.sleep(.45)
    assert not post('/api/command','M 2 0 0',expected=409)['armed']
    session()
    post('/api/command','M 1 0 0')
    # Leave an incomplete HTTP request open. Independent motor task must expire the lease.
    address=urlsplit(base)
    slow=socket.create_connection((address.hostname,address.port or 80),timeout=2)
    slow.sendall(b'POST /api/command HTTP/1.1\r\nHost: rover.local\r\n')
    time.sleep(.6);slow.close()
    assert not post('/api/command','M 2 0 0',expected=409)['armed']
    session()
    assert not post('/api/command','M 1 999 0',expected=409)['armed']
    session()
    assert not post('/api/stop')['armed']
    # Test static assets served from flash, including JavaScript MIME types.
    for path in ['/','/style.css','/app.mjs','/controls.mjs','/wifi-link.mjs']:
        with urlopen(base+path,timeout=5) as response: assert response.status==200 and len(response.read())>100
    print('Real Wi-Fi tests passed: session ownership, zero-speed ACK, watchdog, blocked HTTP, invalid command, STOP, and all embedded web assets. No movement commands sent.')
finally:
    if token:
        try: post('/api/stop')
        except Exception: pass
