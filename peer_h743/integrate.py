"""Install the H743 peer into an existing CubeMX project, preserving other code."""
from pathlib import Path
import argparse
import shutil

parser = argparse.ArgumentParser()
parser.add_argument("project", type=Path)
args = parser.parse_args()
project = args.project.resolve()
source = Path(__file__).resolve().parent
repo = source.parent

if b"STM32H743" not in (project / "Demo.ioc").read_bytes():
    raise SystemExit("The target must be the STM32H743 CubeMX project.")

main_path = project / "Core/Src/main.c"
main = main_path.read_bytes()
newline = b"\r\n" if b"\r\n" in main else b"\n"
main = main.replace(b"  FDCAN_Init();", b"  printf_init();" + newline + b"  FDCAN_Init();") \
    if b"  printf_init();" not in main else main
main = main.replace(
    b'xTaskCreate(vCanSend_Task, "CanSend", 256,NULL, 1, NULL)',
    b'xTaskCreate(Can_CommunicationTask, "CanComm", 384, NULL, 2, NULL)')
if b'xTaskCreate(Can_ReportTask,' not in main:
    anchor = b"\tvTaskStartScheduler();"
    if anchor not in main:
        raise SystemExit("Cannot locate scheduler startup; no files were changed.")
    addition = newline.join([
        b'  if (xTaskCreate(Can_ReportTask, "CanLog", 512, NULL, 1, NULL) != pdPASS)',
        b'  {', b'    Error_Handler();', b'  }', b''])
    main = main.replace(anchor, addition + anchor, 1)
if b'xTaskCreate(Can_CommunicationTask,' not in main:
    raise SystemExit("Cannot locate CAN task startup; no files were changed.")

cmake_path = project / "CMakeLists.txt"
cmake = cmake_path.read_bytes()
cmake_newline = b"\r\n" if b"\r\n" in cmake else b"\n"
if b"User/drv_print.c" not in cmake:
    if b"User/drv_can.c" not in cmake:
        raise SystemExit("Cannot locate CAN sources; no files were changed.")
    cmake = cmake.replace(b"User/drv_can.c",
                          b"User/drv_can.c" + cmake_newline + b"        User/drv_print.c", 1)

config_path = project / "Core/Inc/FreeRTOSConfig.h"
config = config_path.read_bytes().replace(
    b"#define configTOTAL_HEAP_SIZE                        4096",
    b"#define configTOTAL_HEAP_SIZE                        8192")

updates = {
    main_path: main,
    cmake_path: cmake,
    config_path: config,
    project / "User/drv_can.c": (source / "drv_can.c").read_bytes(),
    project / "User/drv_can.h": (source / "drv_can.h").read_bytes(),
    project / "User/can_protocol.h": (repo / "user/can_protocol.h").read_bytes(),
}
backup = repo / "build/h743_before_can"
for path, content in updates.items():
    if path.exists() and path.read_bytes() == content:
        continue
    saved = backup / path.relative_to(project)
    if path.exists() and not saved.exists():
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, saved)
    path.write_bytes(content)
    print(f"Updated {path.relative_to(project)}")
print(f"H743 peer installed in {project}")
