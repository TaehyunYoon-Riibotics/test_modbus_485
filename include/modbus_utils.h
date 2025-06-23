/*
cd test_modbus_485
mkdir -p build && cd build
cmake ..
cmake --build .

# 터미널 A: 슬레이브 실행
./modbus_slave

# 터미널 B: 마스터(쓰기+읽기) 실행
./modbus_example /dev/ttyS1 1 20 42
./modbus_example /dev/ttyS1 1 20 42 30

# 1) 포트 권한 부여 (일반 사용자라면)
sudo usermod -aG dialout $USER
# 또는 임시로
sudo chmod 666 /dev/ttyS1

# 2) baud/parity/data/stop/raw 모드 설정
sudo stty -F /dev/ttyS1 115200 cs8 -cstopb -parenb raw -ixon -ixoff

# 3) RS-485 자동 DE/RE 토글 설정 (sysfs 이용)
echo 1 | sudo tee /sys/class/tty/ttyS1/device/rs485/enabled
echo 0 | sudo tee /sys/class/tty/ttyS1/device/rs485/rts_active_low
echo 1 | sudo tee /sys/class/tty/ttyS1/device/rs485/rts_after_send
echo 0 | sudo tee /sys/class/tty/ttyS1/device/rs485/rts_before_send
echo 0 | sudo tee /sys/class/tty/ttyS1/device/rs485/rx_during_tx
*/

#ifndef MODBUS_UTILS_H
#define MODBUS_UTILS_H

#include <modbus.h>
#include <string>
#include <vector>
#include <iostream>
#include <cerrno>

namespace test_modbus_485 {

class ModbusUtils {
 public:
  ~ModbusUtils() = default;

  // RTU 컨텍스트 열기: device(e.g. "/dev/ttyUSB0"), baud, parity, data/stop bits, slave ID
  // 성공 시 ctx_out에 유효 포인터, 반환 true
  bool openRTU(modbus_t*& ctx_out,
               const std::string& device,
               int baud = 115200,
               char parity = 'N',
               int data_bit = 8,
               int stop_bit = 1,
               int slave_id = 1);

  // 컨텍스트 닫기/해제, true on success
  bool close(modbus_t*& ctx);

  // 홀딩 레지스터 읽기: addr부터 nb개, dest에 저장, 읽은 개수 or -1
  int  readRegisters(modbus_t* ctx,
                     int addr,
                     int nb,
                     std::vector<uint16_t>& dest);

  // 단일 홀딩 레지스터 쓰기: 성공 시 true
  bool writeRegister(modbus_t* ctx,
                     int addr,
                     uint16_t value);

  // 다중 홀딩 레지스터 쓰기: 쓰인 개수 or -1
  int  writeRegisters(modbus_t* ctx,
                      int addr,
                      const std::vector<uint16_t>& src);

  // 코일 읽기: addr부터 nb개, dest에 저장, 읽은 개수 or -1
  int  readCoils(modbus_t* ctx,
                 int addr,
                 int nb,
                 std::vector<uint8_t>& dest);

  // 단일 코일 쓰기: 성공 시 true
  bool writeCoil(modbus_t* ctx,
                 int addr,
                 bool status);

  // 다중 코일 쓰기: 쓰인 개수 or -1
  int  writeCoils(modbus_t* ctx,
                  int addr,
                  const std::vector<uint8_t>& src);

  // 편의: 단일 레지스터 읽기
  bool readRegister(modbus_t* ctx,
                    int addr,
                    uint16_t& out_value);

  // 편의: 단일 코일 읽기
  bool readCoil(modbus_t* ctx,
                int addr,
                bool& out_status);

 private:
  // 내부 ctx 유효성 확인
  bool ensureCtx(modbus_t* ctx, const char* fn);
};

}  // namespace test_modbus_485

#endif  // MODBUS_UTILS_H
