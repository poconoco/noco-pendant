# Search for all source files in the current directory
aux_source_directory(. DIR_Snake_SRCS)

# SnakeFace.cpp needs WS2812.h (for LED_BRIGHTNESS), which in turn needs
# DEV_Config.h. Both live under lib/, two levels up from here.
include_directories(../../lib/WS2812)
include_directories(../../lib/Config)

# Adds a library target
add_library(Snake ${DIR_Snake_SRCS})
target_link_libraries(Snake PUBLIC pico_stdlib Face WS2812)
