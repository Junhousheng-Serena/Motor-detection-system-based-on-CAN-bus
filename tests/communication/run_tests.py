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
    "f103": ["boot_wait", "command_before_sensors", "remote_current_boundary",
             "enable", "disable", "duplicate_disable", "sequence_wrap", "out_of_order",
             "no_heartbeat", "startup_timeout", "command_timeout", "duplicates_timeout",
             "heartbeat_timeout", "recovery_latched", "tick_wrap", "stale_queue", "bad_frames",
             "lower_limit", "fault_snapshot", "fault_busy_retry", "invalid_samples", "overflow", "tx_busy"],
    "h743": ["defaults_periods", "api", "receive", "tick_wrap", "no_heartbeat", "stale_queue",
             "fault", "bad_frames", "overflow", "tx_busy"],
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
