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

# 예제 실행 파일
add_executable(modbus_example src/main.cpp)

target_include_directories(modbus_example PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(modbus_example PRIVATE
  modbus_utils
)

# 슬레이브: 요청-응답용 실행 파일
add_executable(modbus_slave src/modbus_slave.cpp)
target_link_libraries(modbus_slave PRIVATE
  ${LIBM_LIBRARIES}
)