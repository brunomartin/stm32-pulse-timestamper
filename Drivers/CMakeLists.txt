
set(startup_src CMSIS/Device/Source/startup_stm32l476xx.s)
set(system_src CMSIS/Device/Source/system_stm32l4xx.c BSP/stm32l4xx_nucleo.c)

get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
include_directories(${inc_dirs})

add_library(startup OBJECT ${startup_src})
add_library(system OBJECT ${system_src})

add_subdirectory(STM32L4xx_HAL_Driver)