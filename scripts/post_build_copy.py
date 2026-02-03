import os
import re
import shutil


def _read_version(project_dir):
    main_cpp = os.path.join(project_dir, "src", "main.cpp")
    try:
        with open(main_cpp, "r", encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return None
    m = re.search(r"FIRMWARE_VERSION\s*=\s*\"([^\"]+)\"", text)
    if not m:
        return None
    return m.group(1)


def _copy(src, dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copyfile(src, dst)


def post_build(source, target, env):
    if not target:
        return
    build_bin = str(target[0])
    if not os.path.exists(build_bin):
        return
    project_dir = env["PROJECT_DIR"]
    version = _read_version(project_dir) or "unknown"
    web_dir = os.path.join(project_dir, "web-flasher")
    _copy(build_bin, os.path.join(web_dir, "firmware.bin"))
    _copy(build_bin, os.path.join(web_dir, f"stagemod_firmware-v{version}.bin"))


Import("env")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", post_build)
