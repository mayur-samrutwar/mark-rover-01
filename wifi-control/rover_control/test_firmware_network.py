"""Compile and run the pure firmware policies on the host; never opens devices."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class FirmwareNetworkTests(unittest.TestCase):
    def test_recovery_grace_backoff_wrap_and_disconnect_latch(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if compiler is None:
            self.skipTest("A host C++ compiler is required")
        source = Path(__file__).with_suffix(".cpp")
        with tempfile.TemporaryDirectory(prefix="rover-network-host-") as folder:
            executable = Path(folder) / "network-tests"
            subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            str(source), "-o", str(executable)], check=True,
                           capture_output=True, text=True, timeout=30)
            result = subprocess.run([str(executable)], check=True, capture_output=True,
                                    text=True, timeout=5)
        self.assertIn("firmware network host checks passed; no hardware used", result.stdout)


if __name__ == "__main__":
    unittest.main()
