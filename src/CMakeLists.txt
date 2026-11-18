FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.cpp ${CMAKE_SOURCE_DIR}/lib/wt_core/src/*.cpp)
idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/lib/wt_core/src")
