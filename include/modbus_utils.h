/*
cd test_modbus_485
mkdir -p build && cd build
cmake ..
cmake --build .

# 터미널 A: 슬레이브 실행
./modbus_slave

# 터미널 B: 마스터(쓰기+읽기) 실행
./modbus_example /dev/ttyS0 1 20 42
./modbus_example /dev/ttyS0 1 20 42 30

# 1) 포트 권한 부여 (일반 사용자라면)
sudo usermod -aG dialout $USER
# 또는 임시로
sudo chmod 666 /dev/ttyS0

# 2) baud/parity/data/stop/raw 모드 설정
sudo stty -F /dev/ttyS0 115200 cs8 -cstopb -parenb raw -ixon -ixoff

# 3) RS-485 자동 DE/RE 토글 설정 (sysfs 이용)
echo 1 | sudo tee /sys/class/tty/ttyS0/device/rs485/enabled
echo 0 | sudo tee /sys/class/tty/ttyS0/device/rs485/rts_active_low
echo 1 | sudo tee /sys/class/tty/ttyS0/device/rs485/rts_after_send
echo 0 | sudo tee /sys/class/tty/ttyS0/device/rs485/rts_before_send
echo 0 | sudo tee /sys/class/tty/ttyS0/device/rs485/rx_during_tx
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

  bool openRTU(modbus_t*& ctx_out,
               const std::string& device,
               int baud = 115200,
               char parity = 'N',
               int data_bit = 8,
               int stop_bit = 1,
               int slave_id = 1);

  bool close(modbus_t*& ctx);


  int getFd(modbus_t* ctx);

  int  readRegisters(modbus_t* ctx,
                     int addr,
                     int nb,
                     std::vector<uint16_t>& dest);

  bool writeRegister(modbus_t* ctx,
                     int addr,
                     uint16_t value);

  int  writeRegisters(modbus_t* ctx,
                      int addr,
                      const std::vector<uint16_t>& src);

  int  readCoils(modbus_t* ctx,
                 int addr,
                 int nb,
                 std::vector<uint8_t>& dest);

  bool writeCoil(modbus_t* ctx,
                 int addr,
                 bool status);

  int  writeCoils(modbus_t* ctx,
                  int addr,
                  const std::vector<uint8_t>& src);

  bool readRegister(modbus_t* ctx,
                    int addr,
                    uint16_t& out_value);

  bool readCoil(modbus_t* ctx,
                int addr,
                bool& out_status);

 private:
  bool ensureCtx(modbus_t* ctx, const char* fn);
};

}  // namespace test_modbus_485

#endif  // MODBUS_UTILS_H
