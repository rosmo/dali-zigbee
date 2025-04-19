cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENTS_DIRS components/)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(PROJECT_VER "5")

project(zigbee_dali)
