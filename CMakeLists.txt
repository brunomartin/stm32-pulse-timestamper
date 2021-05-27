cmake_minimum_required(VERSION 3.7)
project(arm_hello_world ASM C)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

# set up compiler specific options
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
# CPU architecture specific flags
set(CPU_FLAGS "-mthumb -mcpu=cortex-m4")

# enable hard Cortex-M4 Hard FP
set(GCC_FLAGS "${GCC_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16")

# enable hard Cortex-M4 Soft FP
# set(GCC_FLAGS "${GCC_FLAGS} -mfloat-abi=softfp -mfpu=fpv4-sp-d16")

set(GCC_FLAGS "-ffunction-sections -fdata-sections -fmessage-length=0 -fsigned-char -fshort-enums")

set(STM32L4_LD_SCRIPT "${CMAKE_SOURCE_DIR}/Drivers/BSP/STM32L476RGTx_FLASH.ld")

set(CMAKE_EXE_LINKER_FLAGS "-Xlinker --gc-sections")

# to define unless : undefined reference to `_sbrk'
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -specs=nano.specs -specs=nosys.specs")

# to enable printf for float format
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${CPU_FLAGS} -T\"${STM32L4_LD_SCRIPT}\"")

# CPU defs
add_definitions(${CPU_FLAGS} ${GCC_FLAGS})
# System defs
add_definitions(-DSTM32L476xx)
add_definitions(-DUSE_STM32L4XX_NUCLEO)

set(TARGET pulse_read_main)
set(TARGET pulse_read_irq)

MESSAGE(Target to build: ${TARGET})

set(CMSIS_DIR ${CMAKE_SOURCE_DIR}/Drivers/CMSIS)
set(STM32_HAL_DIR ${CMAKE_SOURCE_DIR}/Drivers/STM32L4xx_HAL_Driver)
set(BSP_DIR ${CMAKE_SOURCE_DIR}/Drivers/BSP)

include_directories(
  ${CMSIS_DIR}/Core/Include
  ${CMSIS_DIR}/Device/Include
  ${STM32_HAL_DIR}/Inc
  ${STM32_HAL_DIR}/Inc/Legacy
  ${BSP_DIR}
  ${CMAKE_SOURCE_DIR}/apps/${TARGET}/Inc
)

set(TARGET_BIN apps/${TARGET}.bin)

add_subdirectory(apps)
add_subdirectory(Drivers)

add_custom_command(OUTPUT arm_${TARGET}.flash
  COMMAND st-flash --reset erase
  COMMAND sleep 0.5
  COMMAND st-flash --reset write ${TARGET_BIN} 0x8000000
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  DEPENDS ${TARGET}
  COMMENT "flashing ${TARGET_BIN}..."
  VERBATIM
)

add_custom_target(flash DEPENDS arm_${TARGET}.flash)