set(app_src main.c)

get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
include_directories(${inc_dirs})

set(STM32_OBJECTS $<TARGET_OBJECTS:startup> $<TARGET_OBJECTS:system> $<TARGET_OBJECTS:stm32l4xx_hal>)

add_executable(hello_world ${app_src} ${STM32_OBJECTS})

string(APPEND CMAKE_EXE_LINKER_FLAGS " -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/hello_world.map")

_generate_object(hello_world .bin binary)
_generate_object(hello_world .hex ihex)
_firmware_size(hello_world)
