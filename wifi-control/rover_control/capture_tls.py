"""Prepare local HTTPS certificates without changing the computer's trust store."""
from __future__ import annotations

import argparse
import ipaddress
import os
from pathlib import Path
import re
import shutil
import socket
import ssl
import subprocess
import sys
import tempfile
import time

_RENEW_WITHIN_SECONDS = 7 * 24 * 60 * 60


def discover_host() -> str | None:
    """Prefer the Mac's Wi-Fi interfaces; the UDP fallback sends no packets."""
    ipconfig = shutil.which("ipconfig")
    if not ipconfig:
        ipconfig = next((path for path in ("/usr/sbin/ipconfig", "/sbin/ipconfig")
                         if Path(path).is_file()), None)
    if ipconfig:
        for interface in ("en0", "en1"):
            try:
                result = subprocess.run(
                    [ipconfig, "getifaddr", interface], capture_output=True,
                    text=True, timeout=5, check=False,
                )
                address = ipaddress.IPv4Address(result.stdout.strip())
                if result.returncode == 0 and not address.is_loopback and not address.is_unspecified:
                    return str(address)
            except (ValueError, OSError, subprocess.SubprocessError):
                continue
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
            connection.settimeout(1)
            connection.connect(("192.0.2.1", 9))
            address = ipaddress.IPv4Address(connection.getsockname()[0])
            if not address.is_loopback and not address.is_unspecified:
                return str(address)
    except (ValueError, OSError):
        pass
    return None


def _normalise_host(host: str) -> str:
    host = host.strip().rstrip(".")
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        labels = host.split(".")
        if len(host) > 253 or not all(re.fullmatch(r"[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?", label)
                                      for label in labels):
            raise ValueError("Use a laptop IPv4 address or hostname, without a URL or port.")
        return host.lower()
    if address.version != 4:
        raise ValueError("Use the laptop's IPv4 address for phone capture.")
    return str(address)


def _certificate_usable(cert_file: Path, key_file: Path, host: str) -> bool:
    if not cert_file.is_file() or not key_file.is_file():
        return False
    try:
        certificate = ssl._ssl._test_decode_cert(str(cert_file))
        if ssl.cert_time_to_seconds(certificate["notAfter"]) <= time.time() + _RENEW_WITHIN_SECONDS:
            return False
        if ssl.cert_time_to_seconds(certificate["notBefore"]) > time.time():
            return False
        names = {value.lower() for kind, value in certificate.get("subjectAltName", ())
                 if kind in {"DNS", "IP Address"}}
        if not {host.lower(), "localhost", "127.0.0.1"}.issubset(names):
            return False
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(str(cert_file), str(key_file))
        return True
    except (KeyError, ValueError, OSError, ssl.SSLError):
        return False


def _write_public_ca(source: Path, destination: Path) -> None:
    # Copy only the public root certificate, never rootCA-key.pem.
    contents = source.read_bytes()
    if destination.is_file() and destination.read_bytes() == contents:
        destination.chmod(0o644)
        return
    with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as temporary:
        temporary.write(contents)
        temporary_path = Path(temporary.name)
    try:
        temporary_path.chmod(0o644)
        temporary_path.replace(destination)
    finally:
        temporary_path.unlink(missing_ok=True)


def prepare_tls(cert_dir: Path, host: str | None = None) -> dict:
    """Return certificate paths or an actionable error; never install a CA.

    Keys are host, cert_file, key_file, ca_file and error. Path values are strings
    so callers can expose selected public values in JSON. The key path and key
    contents must never be included in an HTTP response.
    """
    cert_dir = Path(cert_dir).expanduser().resolve()
    cert_file = cert_dir / "cert.pem"
    key_file = cert_dir / "key.pem"
    ca_file = cert_dir / "rootCA.crt"
    result = {"host": None, "cert_file": str(cert_file), "key_file": str(key_file),
              "ca_file": str(ca_file), "error": None}
    try:
        selected_host = host or discover_host()
        if not selected_host:
            raise ValueError("No active laptop IPv4 address was found. Connect to Wi-Fi or use --capture-host ADDRESS.")
        selected_host = _normalise_host(selected_host)
        result["host"] = selected_host
        mkcert = shutil.which("mkcert")
        if not mkcert and Path("/opt/homebrew/bin/mkcert").is_file():
            mkcert = "/opt/homebrew/bin/mkcert"
        if not mkcert:
            raise ValueError("Phone capture needs mkcert for local HTTPS. Install mkcert, then restart the server; laptop rover control remains available.")
        ca_result = subprocess.run([mkcert, "-CAROOT"], capture_output=True, text=True,
                                   timeout=10, check=True)
        ca_root = Path(ca_result.stdout.strip()).expanduser()
        public_ca = ca_root / "rootCA.pem"
        if not public_ca.is_file():
            raise ValueError("mkcert's public root certificate is missing. Configure the local mkcert CA, then restart; this server does not install or trust certificates automatically.")
        cert_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        cert_dir.chmod(0o700)
        ca_matches = ca_file.is_file() and ca_file.read_bytes() == public_ca.read_bytes()
        if not ca_matches or not _certificate_usable(cert_file, key_file, selected_host):
            with tempfile.TemporaryDirectory(dir=cert_dir) as staging:
                staging = Path(staging)
                staged_cert, staged_key = staging / "cert.pem", staging / "key.pem"
                subprocess.run(
                    [mkcert, "-cert-file", str(staged_cert), "-key-file", str(staged_key),
                     selected_host, "localhost", "127.0.0.1"],
                    capture_output=True, text=True, timeout=30, check=True,
                )
                if not _certificate_usable(staged_cert, staged_key, selected_host):
                    raise ValueError("The generated HTTPS certificate could not be validated.")
                staged_key.chmod(0o600)
                staged_cert.chmod(0o644)
                os.replace(staged_key, key_file)
                os.replace(staged_cert, cert_file)
        key_file.chmod(0o600)
        cert_file.chmod(0o644)
        _write_public_ca(public_ca, ca_file)
    except subprocess.CalledProcessError as error:
        # mkcert output contains public hostnames/paths, but it is unnecessary to
        # expose installation-specific diagnostics to the browser.
        result["error"] = f"mkcert could not prepare local HTTPS (exit {error.returncode}). Laptop rover control remains available."
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        result["error"] = str(error)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="Laptop LAN IPv4 address or hostname")
    parser.add_argument("--cert-dir", type=Path,
                        default=Path(__file__).resolve().parents[2] / ".capture-tls")
    args = parser.parse_args()
    result = prepare_tls(args.cert_dir, args.host)
    if result["error"]:
        print(result["error"], file=sys.stderr)
        return 1
    print(f"Capture host: {result['host']}")
    print(f"Server certificate: {result['cert_file']}")
    print(f"Public CA certificate: {result['ca_file']}")
    print("No trust settings were changed. Install only the public rootCA.crt on the phone through Android Settings.")
    print("Keep the laptop's CA private key and server private key on the laptop.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
