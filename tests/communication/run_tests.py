"""Compile both actual CAN drivers against host HAL/FreeRTOS stubs."""
from pathlib import Path
import shutil
import subprocess
import tempfile

repo = Path(__file__).resolve().parents[2]
test_dir = Path(__file__).resolve().parent
compiler = shutil.which("gcc")
if compiler is None:
    raise SystemExit("A host gcc compiler is required.")

targets = {
    "f103": ["telemetry", "heartbeat", "timeout_wrap", "stale_queue",
             "invalid_samples", "overflow", "bad_frames", "tx_busy"],
    "h743": ["pair", "timeout_wrap", "mismatch", "stale_queue",
             "bad_frames", "overflow", "tx_busy"],
}
with tempfile.TemporaryDirectory(prefix="motor-can-test-") as output:
    for target, cases in targets.items():
        executable = Path(output) / f"{target}.exe"
        source = repo / ("peer_h743" if target == "h743" else "user")
        command = [
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", str(test_dir / "stubs"),
            "-I", str(repo / "tests/thermal/stubs"),
            "-I", str(source), "-I", str(repo / "user"),
            str(test_dir / "test_can.c"), str(source / "drv_can.c"),
        ]
        if target == "h743":
            command.append("-DTEST_H743")
        else:
            command.append(str(repo / "user/drv_motor.c"))
        subprocess.run(command + ["-lm", "-o", str(executable)], check=True)
        for case in cases:
            subprocess.run([str(executable), case], check=True)
print(f"PASS: {sum(map(len, targets.values()))} CAN communication scenarios")
