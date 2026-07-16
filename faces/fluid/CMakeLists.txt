# Search for all source files in the current directory (picks up both the
# Fluid.cpp engine and the FluidFace wrapper around it)
# and save the names to the DIR_Fluid_SRCS variable
aux_source_directory(. DIR_Fluid_SRCS)

# FluidFace.cpp needs WS2812.h (for LED_BRIGHTNESS), which in turn needs
# DEV_Config.h. Both live under lib/, two levels up from here.
include_directories(../../lib/WS2812)
include_directories(../../lib/Config)

# Adds a library target
add_library(Fluid ${DIR_Fluid_SRCS})
target_link_libraries(Fluid PUBLIC pico_stdlib m Face WS2812)
