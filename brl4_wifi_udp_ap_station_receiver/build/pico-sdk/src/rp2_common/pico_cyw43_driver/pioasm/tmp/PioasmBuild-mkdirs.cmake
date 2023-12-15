# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/pico-sdk/tools/pioasm"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_receiver/build/pioasm"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_receiver/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_receiver/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_receiver/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_receiver/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_receiver/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_receiver/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Cornell/23_FALL/ECE_5730/Labs/Pico/brl4_wifi_udp_ap_station_receiver/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
