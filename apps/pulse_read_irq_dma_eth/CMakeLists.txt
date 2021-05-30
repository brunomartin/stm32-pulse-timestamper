file(GLOB app_src Src/*.c)

include_directories(Inc)

set(iolib_src
  ioLibrary_Driver/Ethernet/socket.c
  ioLibrary_Driver/Ethernet/wizchip_conf.c
  ioLibrary_Driver/Ethernet/W5500/w5500.c
  ioLibrary_Driver/Internet/DHCP/dhcp.c
  ioLibrary_Driver/Internet/DNS/dns.c
)

include_directories(
  ioLibrary_Driver/Ethernet
  ioLibrary_Driver/Internet/DHCP
  ioLibrary_Driver/Internet/DNS
)

set(STM32_OBJECTS $<TARGET_OBJECTS:startup> $<TARGET_OBJECTS:system> $<TARGET_OBJECTS:stm32l4xx_hal>)

add_executable(${TARGET} ${app_src} ${STM32_OBJECTS} ${iolib_src})

_generate_object(${TARGET} .bin binary)
_generate_object(${TARGET} .hex ihex)
_firmware_size(${TARGET})
