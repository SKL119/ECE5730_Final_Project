# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/pico-sdk/tools/elf2uf2"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_sender/build/elf2uf2"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_sender/build/elf2uf2"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_sender/build/elf2uf2/tmp"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_sender/build/elf2uf2/src/ELF2UF2Build-stamp"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_sender/build/elf2uf2/src"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_sender/build/elf2uf2/src/ELF2UF2Build-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_sender/build/elf2uf2/src/ELF2UF2Build-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_sender/build/elf2uf2/src/ELF2UF2Build-stamp${cfgdir}") # cfgdir has leading slash
endif()
