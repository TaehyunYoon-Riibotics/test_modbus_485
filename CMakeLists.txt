cmake_minimum_required(VERSION 3.10)
project(test_modbus_485 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBM REQUIRED libmodbus)

# include 경로
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${LIBM_INCLUDE_DIRS}
)

# 정적 라이브러리 타깃
add_library(modbus_utils STATIC
  src/modbus_utils.cpp
)

target_include_directories(modbus_utils PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(modbus_utils PUBLIC
  ${LIBM_LIBRARIES}
)

add_executable(serial_modbus_master src/serial_modbus_master.cpp)
target_link_libraries(serial_modbus_master PRIVATE modbus_utils)

add_executable(serial_modbus_slave src/serial_modbus_slave.cpp)
target_link_libraries(serial_modbus_slave PRIVATE modbus_utils)