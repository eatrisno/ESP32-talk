# ponytail: GLOB, so a newly added .cpp needs a clean build (pio run -t clean) to be picked up.
# CONFIGURE_DEPENDS is not allowed here: ESP-IDF also evaluates this file in CMake script mode.
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.cpp ${CMAKE_SOURCE_DIR}/lib/wt_core/src/*.cpp)
idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/lib/wt_core/src")
