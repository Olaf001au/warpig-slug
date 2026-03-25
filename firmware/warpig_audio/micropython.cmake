# warpig_audio — Synthesis audio engine for MicroPython
# Ported from PicoSound (IWILZ). Works on ESP32-S3 and RP2350.

add_library(usermod_warpig_audio INTERFACE)

target_sources(usermod_warpig_audio INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/warpig_audio.c
)

target_include_directories(usermod_warpig_audio INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_options(usermod_warpig_audio INTERFACE
    -Wno-unused-function
)

target_link_libraries(usermod INTERFACE usermod_warpig_audio)
