file(GLOB app_src Src/*.c)

include_directories(Inc)

set(STM32_OBJECTS $<TARGET_OBJECTS:startup> $<TARGET_OBJECTS:system> $<TARGET_OBJECTS:stm32l4xx_hal>)

add_executable(${TARGET} ${app_src} ${STM32_OBJECTS})

_generate_object(${TARGET} .bin binary)
_generate_object(${TARGET} .hex ihex)
_firmware_size(${TARGET})
