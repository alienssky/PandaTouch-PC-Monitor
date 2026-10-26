#!/usr/bin/env python3
import argparse
from pathlib import Path

from PIL import Image


HEADER_TEMPLATE = '''#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#elif defined(LV_LVGL_H_INCLUDE_SYSTEM)
#include <lvgl.h>
#elif defined(LV_BUILD_TEST)
#include "../lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_{SYMBOL}
#define LV_ATTRIBUTE_{SYMBOL}
#endif

static const
LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_{SYMBOL}
uint8_t {NAME}_map[] = {{
{DATA}
}};

const lv_image_dsc_t {NAME} = {{
  .header = {{
    .magic = LV_IMAGE_HEADER_MAGIC,
    .cf = LV_COLOR_FORMAT_ARGB8888,
    .flags = 0,
    .w = {WIDTH},
    .h = {HEIGHT},
    .stride = {STRIDE},
    .reserved_2 = 0,
  }},
  .data_size = sizeof({NAME}_map),
  .data = {NAME}_map,
  .reserved = NULL,
};
'''


def format_bytes(data: bytes, bytes_per_line: int = 16) -> str:
    lines = []
    for i in range(0, len(data), bytes_per_line):
        chunk = data[i : i + bytes_per_line]
        lines.append("    " + ",".join(f"0x{b:02x}" for b in chunk) + ",")
    return "\n".join(lines)


def generate_lvgl_c(input_path: str, output_path: str, width: int, height: int, name: str) -> None:
    source = Path(input_path)
    if not source.exists():
        raise FileNotFoundError(f"Input image not found: {source}")

    with Image.open(source) as img:
        rgba = img.convert("RGBA")
        resized = rgba.resize((width, height), Image.Resampling.LANCZOS)
        bgra = resized.tobytes("raw", "BGRA")

    symbol_name = name.upper().replace("-", "_")
    safe_name = name.lower().replace("-", "_")
    if not safe_name.replace("_", "").isalnum():
        raise ValueError("Image name must be alphanumeric/underscore only.")

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    rendered = HEADER_TEMPLATE.format(
        SYMBOL=symbol_name,
        NAME=safe_name,
        WIDTH=width,
        HEIGHT=height,
        STRIDE=width * 4,
        DATA=format_bytes(bgra),
    )
    out.write_text(rendered, encoding="utf-8")
    print(f"Generated: {out}")
    print(f"Image size: {width}x{height}")
    print(f"Bytes: {len(bgra)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a LVGL C image object from a JPEG/PNG input at a custom size."
    )
    parser.add_argument("input", help="Path to the source JPEG or PNG image file")
    parser.add_argument("-o", "--output", required=True, help="Output C file path")
    parser.add_argument("-w", "--width", type=int, required=True, help="Target width in pixels")
    parser.add_argument("-h", "--height", type=int, required=True, help="Target height in pixels")
    parser.add_argument("-n", "--name", default="lv_image", help="LVGL image variable name")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    generate_lvgl_c(args.input, args.output, args.width, args.height, args.name)
