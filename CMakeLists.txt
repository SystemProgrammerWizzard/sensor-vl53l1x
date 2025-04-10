cmake_minimum_required(VERSION 3.22)
project(sensor)

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/vl53l1x
)

set(SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/main.c
    ${CMAKE_SOURCE_DIR}/vl53l1x/VL53L1X_api.c
    ${CMAKE_SOURCE_DIR}/vl53l1x/VL53L1X_calibration.c
    ${CMAKE_SOURCE_DIR}/vl53l1x/vl53l1_platform.c
)

add_executable(simple-sensor ${SOURCE_FILES})

install(TARGETS simple-sensor DESTINATION ${CMAKE_INSTALL_BINDIR})
