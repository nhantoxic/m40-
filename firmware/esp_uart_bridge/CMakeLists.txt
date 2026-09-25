cmake_minimum_required(VERSION 3.16)

# Builds for esp32s2 (LOLIN S2 mini, phase P0) and esp32s3 (DevKitC / Rev A module).
#   idf.py set-target esp32s2
#   idf.py set-target esp32s3

# The PlatformIO build directory is outside the repository, so ESP-IDF cannot
# discover the parent .git checkout for PROJECT_VER. Keep a deterministic
# application version instead of invoking git-describe during configuration.
set(PROJECT_VER "1.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_uart_bridge)
