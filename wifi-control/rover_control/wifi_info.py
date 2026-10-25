"""Read the rover's Wi-Fi address over USB. Never sends movement commands."""
import array, fcntl, json, os, select, sys, termios, time, tty
fd=os.open(sys.argv[1],os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
try:
    tty.setraw(fd)
    attrs=termios.tcgetattr(fd); attrs[4]=attrs[5]=termios.B115200
    termios.tcsetattr(fd,termios.TCSANOW,attrs)
    try: fcntl.ioctl(fd,termios.TIOCMBIS,array.array('i',[termios.TIOCM_DTR]))
    except OSError: pass
    buffered=b''; deadline=time.monotonic()+25; next_query=0; last_message=None
    while time.monotonic()<deadline:
        if time.monotonic()>=next_query:
            os.write(fd,b'INFO\n');next_query=time.monotonic()+1
        if select.select([fd],[],[],.3)[0]: buffered+=os.read(fd,4096)
        while b'\n' in buffered:
            line,buffered=buffered.split(b'\n',1)
            try: message=json.loads(line)
            except (ValueError,UnicodeDecodeError): continue
            if message.get('type')=='network':
                if message != last_message: print(json.dumps(message),flush=True)
                last_message=message
                if message.get('connected'): sys.exit(0)
    print('Wi-Fi has not connected yet. Motor outputs remain stopped.');sys.exit(1)
finally: os.close(fd)
