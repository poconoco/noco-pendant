# Search for all source files in the current directory
# and save the names to the DIR_WS2812_SRCS variable
aux_source_directory(. DIR_WS2812_SRCS)

include_directories(../Config)

# Generate link library
add_library(WS2812 ${DIR_WS2812_SRCS})
target_link_libraries(WS2812 PUBLIC Config)