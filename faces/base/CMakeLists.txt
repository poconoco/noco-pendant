# Header-only: Face, ImuSample/FaceFrame, DoubleTapDetector, and FaceSwitcher
# are all fully defined in their headers, so this is an INTERFACE library
# (nothing to compile) rather than the aux_source_directory pattern used by
# the other libs here.
add_library(Face INTERFACE)
target_include_directories(Face INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
