﻿# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.27

# compile ASM with C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-gcc.exe
# compile C with C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-gcc.exe
# compile CXX with C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-g++.exe
ASM_DEFINES = -DCYW43_LWIP=1 -DLIB_PICO_ASYNC_CONTEXT_THREADSAFE_BACKGROUND=1 -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_BOOTSEL_VIA_DOUBLE_RESET=1 -DLIB_PICO_CYW43_ARCH=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_MULTICORE=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RAND=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_UART=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UNIQUE_ID=1 -DLIB_PICO_UTIL=1 -DPICO_BOARD=\"pico_w\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_CYW43_ARCH_THREADSAFE_BACKGROUND=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_TARGET_NAME=\"sender\" -DPICO_USE_BLOCKED_RAM=0

ASM_INCLUDES = -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\build -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\.. -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_async_context\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_platform\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2040\hardware_regs\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_base\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_base\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\build\generated\pico_base -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\boards\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_lwip\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_rand\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_unique_id\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_flash\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2040\hardware_structs\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_claim\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_sync\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_bootrom\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_clocks\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_gpio\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_irq\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_sync\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_time\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_timer\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_util\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_resets\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_pll\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_vreg\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_watchdog\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_xosc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\lib\lwip\src\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_cyw43_arch\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\lib\cyw43-driver\src -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\lib\cyw43-driver\firmware -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_cyw43_driver\cybt_shared_bus -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_pio\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_dma\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_exception\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_cyw43_driver\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\build\pico-sdk\src\rp2_common\pico_cyw43_driver -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_stdlib\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_uart\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_divider\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_runtime\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_printf\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_bit_ops\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_divider\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_double\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_float\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_malloc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_binary_info\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_stdio\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_stdio_uart\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_int64_ops\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_mem_ops\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\boot_stage2\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_pwm\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_adc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_spi\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_rtc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_multicore\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_i2c\include

ASM_FLAGS = -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG -ffunction-sections -fdata-sections

C_DEFINES = -DCYW43_LWIP=1 -DLIB_PICO_ASYNC_CONTEXT_THREADSAFE_BACKGROUND=1 -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_BOOTSEL_VIA_DOUBLE_RESET=1 -DLIB_PICO_CYW43_ARCH=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_MULTICORE=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RAND=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_UART=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UNIQUE_ID=1 -DLIB_PICO_UTIL=1 -DPICO_BOARD=\"pico_w\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_CYW43_ARCH_THREADSAFE_BACKGROUND=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_TARGET_NAME=\"sender\" -DPICO_USE_BLOCKED_RAM=0

C_INCLUDES = -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\build -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\.. -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_async_context\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_platform\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2040\hardware_regs\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_base\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_base\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\build\generated\pico_base -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\boards\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_lwip\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_rand\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_unique_id\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_flash\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2040\hardware_structs\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_claim\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_sync\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_bootrom\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_clocks\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_gpio\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_irq\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_sync\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_time\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_timer\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_util\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_resets\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_pll\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_vreg\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_watchdog\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_xosc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\lib\lwip\src\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_cyw43_arch\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\lib\cyw43-driver\src -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\lib\cyw43-driver\firmware -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_cyw43_driver\cybt_shared_bus -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_pio\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_dma\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_exception\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_cyw43_driver\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\build\pico-sdk\src\rp2_common\pico_cyw43_driver -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_stdlib\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_uart\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_divider\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_runtime\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_printf\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_bit_ops\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_divider\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_double\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_float\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_malloc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_binary_info\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_stdio\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_stdio_uart\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_int64_ops\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_mem_ops\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\boot_stage2\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_pwm\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_adc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_spi\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_rtc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_multicore\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_i2c\include

C_FLAGS = -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG -ffunction-sections -fdata-sections

CXX_DEFINES = -DCYW43_LWIP=1 -DLIB_PICO_ASYNC_CONTEXT_THREADSAFE_BACKGROUND=1 -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_BOOTSEL_VIA_DOUBLE_RESET=1 -DLIB_PICO_CYW43_ARCH=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_MULTICORE=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RAND=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_UART=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UNIQUE_ID=1 -DLIB_PICO_UTIL=1 -DPICO_BOARD=\"pico_w\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_CYW43_ARCH_THREADSAFE_BACKGROUND=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_TARGET_NAME=\"sender\" -DPICO_USE_BLOCKED_RAM=0

CXX_INCLUDES = -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\build -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\.. -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_async_context\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_platform\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2040\hardware_regs\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_base\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_base\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\build\generated\pico_base -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\boards\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_lwip\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_rand\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_unique_id\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_flash\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2040\hardware_structs\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_claim\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_sync\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_bootrom\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_clocks\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_gpio\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_irq\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_sync\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_time\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_timer\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_util\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_resets\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_pll\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_vreg\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_watchdog\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_xosc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\lib\lwip\src\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_cyw43_arch\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\lib\cyw43-driver\src -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\lib\cyw43-driver\firmware -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_cyw43_driver\cybt_shared_bus -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_pio\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_dma\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_exception\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_cyw43_driver\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\brl4_wifi_udp_ap_station_sender\build\pico-sdk\src\rp2_common\pico_cyw43_driver -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_stdlib\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_uart\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_divider\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_runtime\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_printf\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_bit_ops\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_divider\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_double\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_float\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_malloc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\common\pico_binary_info\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_stdio\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_stdio_uart\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_int64_ops\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_mem_ops\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\boot_stage2\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_pwm\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_adc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_spi\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_rtc\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\pico_multicore\include -IC:\Cornell\23_FALL\ECE_5730\Labs\Pico\pico-sdk\src\rp2_common\hardware_i2c\include

CXX_FLAGS = -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG -ffunction-sections -fdata-sections -fno-exceptions -fno-unwind-tables -fno-rtti -fno-use-cxa-atexit

