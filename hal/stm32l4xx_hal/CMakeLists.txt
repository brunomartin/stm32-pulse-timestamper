aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/Src hal_src)

get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
include_directories(${inc_dirs})

add_library(stm32l4xx_hal OBJECT ${hal_src})