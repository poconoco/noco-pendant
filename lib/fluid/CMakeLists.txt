# Search for all source files in the current directory
# and save the names to the DIR_Fluid_SRCS variable
aux_source_directory(. DIR_Fluid_SRCS)

# Adds a library target
add_library(Fluid ${DIR_Fluid_SRCS})
target_link_libraries(Fluid PUBLIC pico_stdlib m)
