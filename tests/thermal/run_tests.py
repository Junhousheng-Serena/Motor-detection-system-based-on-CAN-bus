"""Compile the actual drivers against host stubs; each process simulates reset."""
from pathlib import Path
import shutil
import subprocess
import tempfile

repo = Path(__file__).resolve().parents[2]
test_dir = Path(__file__).resolve().parent
compiler = shutil.which("gcc")
if not compiler:
    raise SystemExit("A host gcc compiler is required (not arm-none-eabi-gcc).")

cases = [
    "startup", "overheat", "hot_boot", "invalid_temperature",
    "infinite_temperature", "manual_stop", "adc_overheat", "adc_open",
    "adc_short", "adc_timeout", "adc_start_error", "adc_stop_error",
    "adc_split_filter", "adc_voltage_endpoints", "adc_report",
]
with tempfile.TemporaryDirectory(prefix="f103-thermal-test-") as output:
    executable = Path(output) / "test_thermal.exe"
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-I", str(test_dir / "stubs"), "-I", str(repo / "user"),
        str(test_dir / "test_thermal.c"),
        str(repo / "user/drv_motor.c"), str(repo / "user/drv_ntc.c"),
        "-lm", "-o", str(executable),
    ], check=True)
    for case in cases:
        subprocess.run([str(executable), case], check=True)
print(f"PASS: {len(cases)} thermal protection scenarios")
