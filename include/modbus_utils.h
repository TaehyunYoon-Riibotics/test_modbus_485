// include/modbus_utils.h

#ifndef MODBUS_UTILS_H
#define MODBUS_UTILS_H

#include <modbus.h>
#include <string>
#include <vector>

namespace test_modbus_485 {

class ModbusUtils {
 public:
  ~ModbusUtils() = default;

  // RTU 컨텍스트 열기 및 설정 (baud, parity, data/stop bits, slave ID)
  // 성공 시 ctx_out에 유효 포인터, 반환 true
  bool openRTU(modbus_t*& ctx_out,
               const std::string& device,
               int baud = 115200,
               char parity = 'N',
               int data_bit = 8,
               int stop_bit = 1,
               int slave_id = 1);

  // 컨텍스트 닫기 및 해제
  bool close(modbus_t*& ctx);

  // 내부 포트 FD 반환 (RTU 모드인 경우), 실패 시 -1
  int getFd(modbus_t* ctx);

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
  bool ensureCtx(modbus_t* ctx, const char* fn);
};

}  // namespace test_modbus_485

#endif  // MODBUS_UTILS_H
