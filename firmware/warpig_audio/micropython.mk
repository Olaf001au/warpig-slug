# warpig_audio — Synthesis audio engine for MicroPython
# For ESP32 make-based build

WARPIG_AUDIO_MOD_DIR := $(USERMOD_DIR)

SRC_USERMOD_C += $(WARPIG_AUDIO_MOD_DIR)/warpig_audio.c

CFLAGS_USERMOD += -I$(WARPIG_AUDIO_MOD_DIR)
CFLAGS_USERMOD += -Wno-unused-function
