cmake_minimum_required(VERSION 3.7)
project(arm_hello_world ASM C)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

# set up compiler specific options
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
# CPU architecture specific flags
set(CPU_FLAGS "-mthumb -mcpu=cortex-m4")
# set(CPU_FLAGS "${CPU_FLAGS} -specs=nano.specs -specs=nosys.specs")

# enable hard Cortex-M4 Hard FP
# set(GCC_FLAGS "${GCC_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16")

# enable hard Cortex-M4 Soft FP
set(GCC_FLAGS "${GCC_FLAGS} -mfloat-abi=softfp -mfpu=fpv4-sp-d16")

set(GCC_FLAGS "-ffunction-sections -fdata-sections -fmessage-length=0 -fsigned-char -fshort-enums")

set(STM32L4_LD_SCRIPT "${CMAKE_SOURCE_DIR}/hal/system/armgcc/STM32L476RGTx_FLASH.ld")

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

set(CMSIS_DIR ${CMAKE_SOURCE_DIR}/hal/cmsis)
set(STM32L4XX_HAL ${CMAKE_SOURCE_DIR}/hal/stm32l4xx_hal/Inc)
set(STM32L4XX_HAL_LEGACY ${STM32L4XX_HAL}/Legacy)
set(SYSTEM_DIR ${CMAKE_SOURCE_DIR}/hal/system)
set(APP_DIR ${CMAKE_SOURCE_DIR}/app/Inc)

include_directories(
  ${CMSIS_DIR}
  ${STM32L4XX_HAL}
  ${STM32L4XX_HAL_LEGACY}
  ${SYSTEM_DIR}
  ${APP_DIR}
  ${BSP_DIR}
)

add_subdirectory(app/)
add_subdirectory(hal/)

add_custom_command(OUTPUT arm_hello_world.flash
  COMMAND st-flash --reset erase
  COMMAND sleep 0.5
  COMMAND st-flash --reset write app/hello_world.bin 0x8000000
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  DEPENDS hello_world
  COMMENT "flashing app/arm_hello_world.bin..."
  VERBATIM
)

add_custom_target(flash DEPENDS arm_hello_world.flash)