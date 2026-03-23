# WarpPig Slug — Bézier Font Rendering for Microcontrollers

GPU-quality antialiased text on ESP32-S3 and RP2350, powered by Eric Lengyel's
Slug Algorithm (public domain patent, MIT reference shaders).

## What This Is

A complete font rendering pipeline for WarpPig:

1. **Builder** (`warpig_slug.py`) — runs on Pi 5, converts TTF → `.slugfont` binary
2. **C Module** (`firmware/warpig_font/`) — WarpPig's 4th native module, renders
   `.slugfont` files to any framebuf at any size with proper antialiasing
3. **Font Library** (`fonts/slugfont/`) — pre-built fonts ready to deploy to boards

## Font Library

| Font | Style | Size | License | Best For |
|------|-------|------|---------|----------|
| 0xProto | Regular (NL) | 63 KB | SIL OFL | Terminal, OLED, sensor readouts (monospace) |
| 0xProto | Bold | 62 KB | SIL OFL | Highlighted values, alerts |
| Poppins | Regular | 54 KB | SIL OFL | Explorer LCD UI, clean Google-style sans |
| Poppins | Medium | 54 KB | SIL OFL | Labels, headings |
| Poppins | Bold | 52 KB | SIL OFL | Titles, button text |
| DejaVu Sans | Regular | 54 KB | Bitstream Vera | General purpose, has kerning |

All fonts include printable ASCII + common symbols (~87 glyphs each).

## MicroPython API

```python
import warpig_font

# Load font from flash
font = warpig_font.SlugFont("/fonts/0xProto.slugfont")

# Font info
font.info()              # {'glyphs': 87, 'bands': 10, 'ascender': 1130, ...}
font.metrics(24)         # (17, 7, 29) — ascender_px, descender_px, line_height

# Measure before rendering
w = font.text_width("Temp: 23.4C", 16)

# Render string into any bytearray
buf = bytearray(320 * 40)
font.render("WarpPig Slug!", 24, buf, 320)        # x=0, y=0
font.render("Temp: 23.4C", 16, buf, 320, 10, 30)  # x=10, y=30

# Single glyph
font.render_glyph(ord('A'), 32, buf, 320, 50, 5)
```

## Rendering Performance (projected, float32 C module)

| Board | Per Glyph (20px) | 20-char Line | Notes |
|-------|-----------------|--------------|-------|
| ESP32-S3 @ 240MHz | ~230 µs | ~5 ms | Single-cycle float32 multiply |
| RP2350 @ 150MHz | ~310 µs | ~6 ms | FPv5 hardware VSQRT |
| Pi 5 (benchmark) | ~16 µs | ~0.3 ms | Reference baseline |

## Building Custom Fonts

On the Pi 5:

```bash
cd /opt/warpig
.venv/bin/python3 warpig_slug.py  # runs demo with system fonts

# Build a .slugfont from any TTF
.venv/bin/python3 -c "
from warpig_slug import build_slugfont
stats = build_slugfont(
    'fonts/ttf/0xProto-Regular-NL.ttf',
    'fonts/slugfont/0xProto.slugfont',
    chars=None,   # None = printable ASCII
    n_bands=10    # more bands = faster render, larger file
)
print(stats)
"
```

## Building the Firmware

The C module slots in alongside warpig_http, warpig_httpd, warpig_rtos:

```bash
# ESP32-S3 — add to USER_C_MODULES in board makefile
# or copy to the micropython user modules path
cp -r firmware/warpig_font/ /opt/micropython_build/micropython/ports/esp32/boards/WARPIG_PROS3/modules/warpig_font/

source /opt/esp-idf/export.sh
cd /opt/micropython_build/micropython/ports/esp32
make BOARD=WARPIG_PROS3 USER_C_MODULES=/opt/warpig/firmware/warpig_font/micropython.mk -j$(nproc)

# RP2350 Explorer — add to cmake USER_C_MODULES list
cmake -S /opt/micropython_build/micropython/ports/rp2 \
    -B /opt/micropython_build/build-warpig-explorer \
    -DUSER_C_MODULES="/opt/warpig/firmware/warpig_font/micropython.cmake;..." \
    ...
```

## Deploying Fonts to Boards

```bash
# ESP32-S3 via mpremote
mpremote connect /dev/ttyACM1 mkdir /fonts
mpremote connect /dev/ttyACM1 cp fonts/slugfont/0xProto.slugfont :/fonts/0xProto.slugfont

# RP2350 Explorer via USB mass storage
sudo mount /dev/sdX1 /mnt/rp2350
cp fonts/slugfont/Poppins-Medium.slugfont /mnt/rp2350/fonts/

# AtomicBlonde via WebREPL
webrepl_deploy.py --host 192.168.1.233 fonts/slugfont/0xProto.slugfont
```

## How It Works

The Slug algorithm renders text directly from quadratic Bézier curves — no bitmap
atlas, no SDF textures, no rasterization step. For each pixel, it:

1. Casts horizontal and vertical rays through the glyph's curve data
2. Solves the quadratic equation to find ray-curve intersections
3. Uses the intersection positions for subpixel antialiasing
4. Combines both rays with Lengyel's weighted coverage formula

The `.slugfont` binary pre-computes spatial band indices so each pixel only tests
~3-4 curves instead of all 15-25 curves in a typical glyph.

All math uses float32 — matching the original GPU shader and giving single-cycle
multiply on both ESP32-S3 (Xtensa LX7) and RP2350 (Cortex-M33 FPv5).

## Credits

- Slug Algorithm: Eric Lengyel, Terathon Software (patent dedicated to public domain)
- Reference shaders: MIT License, Copyright 2017 Eric Lengyel
- 0xProto font: 0xType Project Authors, SIL Open Font License
- Poppins font: Indian Type Foundry / Google, SIL Open Font License
- DejaVu Sans: Bitstream Vera license
