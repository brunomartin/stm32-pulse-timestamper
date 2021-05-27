file(GLOB_RECURSE app_src ${TARGET}/*.c)

include_directories(${TARGET}/Inc)

set(STM32_OBJECTS $<TARGET_OBJECTS:startup> $<TARGET_OBJECTS:system> $<TARGET_OBJECTS:stm32l4xx_hal>)

add_executable(${TARGET} ${app_src} ${STM32_OBJECTS})

string(APPEND CMAKE_EXE_LINKER_FLAGS " -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/app.map")

_generate_object(${TARGET} .bin binary)
_generate_object(${TARGET} .hex ihex)
_firmware_size(${TARGET})
