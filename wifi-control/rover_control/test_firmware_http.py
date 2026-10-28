"""Compile the actual frame gate/subclass against a small host API double."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class FirmwareHTTPTests(unittest.TestCase):
    def test_complete_frames_deadline_eof_and_snapshot_parser(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if compiler is None:
            self.skipTest("A host C++ compiler is required")
        root = Path(__file__).parent
        with tempfile.TemporaryDirectory(prefix="rover-http-host-") as folder:
            executable = Path(folder) / "http-tests"
            subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-I", str(root / "host_http_stubs"),
                            str(root / "test_firmware_http.cpp"), "-o", str(executable)],
                           check=True, capture_output=True, text=True, timeout=30)
            result = subprocess.run([str(executable)], check=True, capture_output=True,
                                    text=True, timeout=5)
        self.assertIn("firmware HTTP host checks passed; no hardware used", result.stdout)


if __name__ == "__main__":
    unittest.main()
