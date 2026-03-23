# warpig_font — Bézier font rendering C module
# For MicroPython ESP32 port (ESP32-S3)

WARPIG_FONT_MOD_DIR := $(USERMOD_DIR)

# Source files
SRC_USERMOD_C += $(WARPIG_FONT_MOD_DIR)/warpig_font.c

# Include paths
CFLAGS_USERMOD += -I$(WARPIG_FONT_MOD_DIR)

# Ensure we use float (not double) for the rendering math.
# The ESP32-S3 Xtensa LX7 has single-cycle float32 multiply
# but float64 is ~15x slower. Slug was designed for float32.
CFLAGS_USERMOD += -Wno-unused-function
