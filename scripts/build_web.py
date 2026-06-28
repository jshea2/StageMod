#!/usr/bin/env python3
"""Pre-build script: gzip web/index.html → src/web_ui.h as a PROGMEM byte array."""
import gzip
import os

Import("env")

def build_web_header(*args, **kwargs):
    project_dir = env.get("PROJECT_DIR", ".")
    src = os.path.join(project_dir, "web", "index.html")
    dst = os.path.join(project_dir, "src", "web_ui.h")

    if not os.path.exists(src):
        print("  [build_web] web/index.html not found, skipping")
        return

    # Skip if header is newer than source
    if os.path.exists(dst) and os.path.getmtime(dst) >= os.path.getmtime(src):
        print("  [build_web] web_ui.h is up to date")
        return

    with open(src, "rb") as f:
        raw = f.read()
    compressed = gzip.compress(raw, compresslevel=9)

    lines = []
    lines.append("// Auto-generated from web/index.html — do not edit")
    lines.append("#pragma once")
    lines.append("#include <pgmspace.h>")
    lines.append("")
    lines.append(f"const size_t WEB_UI_GZ_LEN = {len(compressed)};")
    lines.append("const uint8_t WEB_UI_GZ[] PROGMEM = {")

    # Write 20 bytes per line
    for i in range(0, len(compressed), 20):
        chunk = compressed[i:i+20]
        hex_str = ", ".join(f"0x{b:02x}" for b in chunk)
        lines.append(f"  {hex_str},")

    lines.append("};")
    lines.append("")

    with open(dst, "w") as f:
        f.write("\n".join(lines))

    print(f"  [build_web] {len(raw)} bytes → {len(compressed)} bytes gzipped → {dst}")

build_web_header()
