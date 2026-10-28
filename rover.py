#!/usr/bin/env python3
"""Start the local Rover project and keep its browser services available."""
from __future__ import annotations

import argparse
import fcntl
import http.client
import os
from pathlib import Path
import plistlib
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
RUNTIME = ROOT / '.rover-runtime'
URL = 'http://rover.localhost:1355/'
LABEL = 'local.mark-rover.project'
PYTHON = ROOT / '.venv-reconstruction/bin/python'
PATH = '/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin'


def ready(port, expected, *, public=False):
    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=2)
    try:
        headers = {'Host': 'rover.localhost:1355'} if public else {}
        connection.request('GET', '/', headers=headers)
        response = connection.getresponse()
        return response.status == 200 and expected in response.read(16384).decode(errors='replace')
    except (OSError, http.client.HTTPException):
        return False
    finally:
        connection.close()


def occupied(port):
    with socket.socket() as connection:
        connection.settimeout(.3)
        return connection.connect_ex(('127.0.0.1', port)) == 0


def run():
    RUNTIME.mkdir(exist_ok=True)
    lock = (RUNTIME / 'supervisor.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('Rover is already supervised.', flush=True)
        return
    os.environ['PATH'] = PATH
    portless = shutil.which('portless')
    if not portless:
        raise SystemExit('Install Vercel Portless first: npm install -g portless')
    if not PYTHON.exists():
        raise SystemExit('Set up .venv-reconstruction using reconstruction/README.md first.')
    code_file = RUNTIME / 'pair-code'
    if not code_file.exists():
        code_file.write_text(f'{secrets.randbelow(1_000_000):06d}')
        code_file.chmod(0o600)
    code = code_file.read_text().strip()
    if len(code) != 6 or not code.isdigit():
        raise SystemExit('Invalid .rover-runtime/pair-code; expected six digits.')
    env = {**os.environ, 'PYTHONDONTWRITEBYTECODE': '1',
           'PORTLESS_PORT': '1355', 'PORTLESS_HTTPS': '0', 'PORTLESS_SYNC_HOSTS': '0'}
    services = {
        'reconstruction': (8768, '<title>Reconstruction</title>', [str(PYTHON), 'reconstruction/server.py']),
        'control': (8765, '<title>Control</title>', [str(PYTHON), 'wifi-control/rover_control/serve.py',
                    '--public-url', URL.rstrip('/'), '--pair-code', code]),
    }
    children = {}
    last_start = {}
    stopped = False

    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print(f'Rover project: {URL}', flush=True)
    alias_ready = False
    last_alias = 0.
    try:
        while not stopped:
            for name, (port, title, command) in services.items():
                child = children.get(name)
                if child and child.poll() is None:
                    continue
                if ready(port, title, public=name == 'control'):
                    continue  # Reuse a service that was already running.
                if occupied(port):
                    if time.monotonic() - last_start.get(name, 0) > 30:
                        print(f'{name}: port {port} is held by an incompatible service; leaving it alone.', flush=True)
                        last_start[name] = time.monotonic()
                    continue
                if time.monotonic() - last_start.get(name, 0) < 3:
                    continue
                with (RUNTIME / f'{name}.log').open('ab') as log:
                    children[name] = subprocess.Popen(command, cwd=ROOT, env=env,
                        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True)
                last_start[name] = time.monotonic()
                print(f'Started {name} (PID {children[name].pid}).', flush=True)
            if time.monotonic() - last_alias > 15:
                # An alias persists across app exits. Re-register after a proxy restart.
                if not alias_ready or not occupied(1355):
                    result = subprocess.run([portless, 'alias', 'rover', '8765'], env=env,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=20)
                    print(result.stdout.strip(), flush=True)
                    alias_ready = result.returncode == 0
                last_alias = time.monotonic()
            time.sleep(1)
    finally:
        # Only stop children owned by this supervisor, never adopted processes.
        for child in children.values():
            if child.poll() is None:
                child.terminate()
        for child in children.values():
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        lock.close()


def start(open_browser=False):
    if sys.platform != 'darwin':
        raise SystemExit('Automatic startup uses macOS launchd. Run python3 rover.py run on other systems.')
    RUNTIME.mkdir(exist_ok=True)
    agents = Path.home() / 'Library/LaunchAgents'
    agents.mkdir(parents=True, exist_ok=True)
    destination = agents / f'{LABEL}.plist'
    plist = {
        'Label': LABEL, 'ProgramArguments': [str(PYTHON), str(ROOT / 'rover.py'), 'run'],
        'WorkingDirectory': str(ROOT), 'RunAtLoad': True, 'KeepAlive': True,
        'ThrottleInterval': 5, 'EnvironmentVariables': {'PATH': PATH, 'PYTHONDONTWRITEBYTECODE': '1'},
        'StandardOutPath': str(RUNTIME / 'supervisor.log'),
        'StandardErrorPath': str(RUNTIME / 'supervisor.log'),
    }
    contents = plistlib.dumps(plist)
    changed = not destination.exists() or destination.read_bytes() != contents
    if changed:
        destination.write_bytes(contents)
    target = f'gui/{os.getuid()}'
    service = f'{target}/{LABEL}'
    loaded = subprocess.run(['launchctl', 'print', service], capture_output=True).returncode == 0
    if loaded and changed:
        subprocess.run(['launchctl', 'bootout', service], check=True)
        loaded = False
    if not loaded:
        subprocess.run(['launchctl', 'bootstrap', target, str(destination)], check=True)
    for _ in range(30):
        if ready(8765, '<title>Control</title>', public=True) and ready(8768, '<title>Reconstruction</title>') and occupied(1355):
            print(URL)
            if open_browser:
                subprocess.run(['open', URL], check=True)
            return
        time.sleep(1)
    raise SystemExit(f'Rover is starting; see {RUNTIME / "supervisor.log"}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'run', 'status', 'stop'), nargs='?', default='start')
    parser.add_argument('--open', action='store_true', dest='open_browser')
    args = parser.parse_args()
    if args.action == 'run':
        run()
    elif args.action == 'start':
        start(args.open_browser)
    elif args.action == 'status':
        for name, port, title in [('control', 8765, '<title>Control</title>'), ('reconstruction', 8768, '<title>Reconstruction</title>')]:
            print(f'{name}: {"running" if ready(port, title, public=name == "control") else "stopped"}')
        print(URL)
    else:
        service = f'gui/{os.getuid()}/{LABEL}'
        subprocess.run(['launchctl', 'bootout', service], check=True)
        agent = Path.home() / 'Library/LaunchAgents' / f'{LABEL}.plist'
        agent.unlink(missing_ok=True)
        print('Rover startup service stopped and uninstalled.')


if __name__ == '__main__':
    main()
