# test_modbus_485

이 프로젝트는 `libmodbus` 기반의 Modbus-RTU C++ 유틸리티를 제공합니다.  
RS-485 터미널 설정 방법과 빌드·실행 절차는 아래와 같습니다.

---

## 📦 빌드 & 실행

```
cd test_modbus_485
mkdir -p build && cd build
cmake ..
cmake --build .

# Example CLI command
# Port: /dev/ttyS1, Slave ID: 1, Write Reg[20]=42, Read Reg[20]
./modbus_example /dev/ttyS1 1 20 42

# Only read Reg[10]
./modbus_example /dev/ttyS1 1 20 42 10
```
🔧 RS-485 포트 활성화
포트 권한 부여

```
# 일반 사용자라면 dialout 그룹에 추가 (재로그인 필요)
sudo usermod -aG dialout $USER

# 또는 임시로 모든 사용자에게 읽기/쓰기 허용
sudo chmod 666 /dev/ttyS1
baud/parity/data/stop/raw 모드 설정

sudo stty -F /dev/ttyS1 115200 cs8 -cstopb -parenb raw -ixon -ixoff
RS-485 자동 DE/RE 토글 설정

bash
echo 1 | sudo tee /sys/class/tty/ttyS1/device/rs485/enabled
echo 0 | sudo tee /sys/class/tty/ttyS1/device/rs485/rts_active_low
echo 1 | sudo tee /sys/class/tty/ttyS1/device/rs485/rts_after_send
echo 0 | sudo tee /sys/class/tty/ttyS1/device/rs485/rts_before_send
echo 0 | sudo tee /sys/class/tty/ttyS1/device/rs485/rx_during_tx
```
위 설정을 마친 후 /dev/ttyS1 을 사용하는 Modbus-RTU 프로그램이 자동으로 DE/RE 신호를 조절하며 정상 통신합니다.

📚 라이브러리 활용 예시
1) CMake 프로젝트에 가져다 쓰기
```
cmake_minimum_required(VERSION 3.10)
project(my_modbus_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# test_modbus_485 경로에 add_subdirectory() 로 포함
add_subdirectory(path/to/test_modbus_485 test_modbus_485_build)

add_executable(my_app src/my_app.cpp)
target_link_libraries(my_app PRIVATE modbus_utils)
target_include_directories(my_app PRIVATE
  ${CMAKE_SOURCE_DIR}/path/to/test_modbus_485/include
)
```
2) my_app.cpp 예제
```
#include "modbus_utils.h"
#include <iostream>

int main() {
  using namespace test_modbus_485;

  modbus_t* ctx = nullptr;
  ModbusUtils mb;

  // 포트 열기
  if (!mb.openRTU(ctx, "/dev/ttyS1", 115200, 'N', 8, 1, /*slave=*/2)) {
    std::cerr<<"포트 열기 실패\n";
    return 1;
  }

  // 레지스터 100번에 1234 쓰기
  if (!mb.writeRegister(ctx, 100, 1234)) {
    std::cerr<<"쓰기 실패\n";
  }

  // 레지스터 100번 읽기
  uint16_t val;
  if (mb.readRegister(ctx, 100, val)) {
    std::cout<<"Reg[100] = "<<val<<"\n";
  }

  // 종료
  mb.close(ctx);
  return 0;
}
```





