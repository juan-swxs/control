# Firmware del ESP32 de referencia: recibe Vr por ESP-NOW y lo saca en el MCP4725 #3
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components/MCP4725"
                         "${CMAKE_CURRENT_LIST_DIR}/../components/LAB_LINK")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ref_node)
