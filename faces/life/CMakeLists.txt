# Search for all source files in the current directory
aux_source_directory(. DIR_Life_SRCS)

# LifeFace.cpp needs WS2812.h (for LED_BRIGHTNESS), which in turn needs
# DEV_Config.h. Both live under lib/, two levels up from here.
include_directories(../../lib/WS2812)
include_directories(../../lib/Config)

# Adds a library target
add_library(Life ${DIR_Life_SRCS})
target_link_libraries(Life PUBLIC pico_stdlib m Face WS2812)
