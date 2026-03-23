# warpig_font — Bézier font rendering C module
# For MicroPython RP2 port (RP2350, RP2040)

add_library(usermod_warpig_font INTERFACE)

target_sources(usermod_warpig_font INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/warpig_font.c
)

target_include_directories(usermod_warpig_font INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Use hardware float32 where available (FPv5 on M33, soft on M0+)
target_compile_options(usermod_warpig_font INTERFACE
    -Wno-unused-function
)

target_link_libraries(usermod INTERFACE usermod_warpig_font)
