file(GLOB_RECURSE hal_src *.c)

include_directories(Inc)

add_library(stm32l4xx_hal ${hal_src})