// src/modbus_utils.cpp

#include "modbus_utils.h"

#include <cerrno>
#include <iostream>
#include <sys/ioctl.h>
#ifdef __linux__
#  include <linux/serial.h>
#endif

using namespace test_modbus_485;

bool ModbusUtils::ensureCtx(modbus_t* ctx, const char* fn) {
  if (!ctx) {
    std::cerr << "[" << fn << "] null context\n";
    return false;
  }
  return true;
}

bool ModbusUtils::openRTU(modbus_t*& ctx_out,
                          const std::string& device,
                          int baud,
                          char parity,
                          int data_bit,
                          int stop_bit,
                          int slave_id) {
  // 1) libmodbus RTU 컨텍스트 생성
  ctx_out = ::modbus_new_rtu(device.c_str(), baud, parity, data_bit, stop_bit);
  if (!ctx_out) {
    std::cerr << "[openRTU] modbus_new_rtu failed\n";
    return false;
  }

  // 2) 디버그 끄기(필요시 TRUE로 바꿔 로그 확인)
  ::modbus_set_debug(ctx_out, FALSE);

  // 3) 슬레이브 ID 설정
  if (::modbus_set_slave(ctx_out, slave_id) == -1) {
    std::cerr << "[openRTU] set_slave failed: " << modbus_strerror(errno) << "\n";
    ::modbus_free(ctx_out);
    ctx_out = nullptr;
    return false;
  }

  // 4) 직렬 포트 열기
  if (::modbus_connect(ctx_out) == -1) {
    std::cerr << "[openRTU] connect failed: " << modbus_strerror(errno) << "\n";
    ::modbus_free(ctx_out);
    ctx_out = nullptr;
    return false;
  }

  // 5) RS-485 자동 DE/RE 토글 설정 (Linux 전용)
#ifdef __linux__
  {
    int fd = ::modbus_get_socket(ctx_out);
    if (fd >= 0) {
      struct serial_rs485 rs;
      if (ioctl(fd, TIOCGRS485, &rs) >= 0) {
        rs.flags |= SER_RS485_ENABLED
                 |  SER_RS485_RTS_ON_SEND
                 |  SER_RS485_RTS_AFTER_SEND;
        rs.delay_rts_before_send = 0;
        rs.delay_rts_after_send  = 0;
        if (ioctl(fd, TIOCSRS485, &rs) < 0) {
          perror("[openRTU] TIOCSRS485");
        }
      } else {
        perror("[openRTU] TIOCGRS485");
      }
    }
  }
#endif

  return true;
}

bool ModbusUtils::close(modbus_t*& ctx) {
  if (!ctx) return true;
  ::modbus_close(ctx);
  ::modbus_free(ctx);
  ctx = nullptr;
  return true;
}

int ModbusUtils::getFd(modbus_t* ctx) {
  if (!ensureCtx(ctx, "getFd")) return -1;
  return ::modbus_get_socket(ctx);
}

int ModbusUtils::readRegisters(modbus_t* ctx,
                               int addr,
                               int nb,
                               std::vector<uint16_t>& dest) {
  if (!ensureCtx(ctx, "readRegisters")) return -1;
  dest.assign(nb, 0);
  int rc = ::modbus_read_registers(ctx, addr, nb, dest.data());
  if (rc == -1) {
    std::cerr << "[readRegisters] " << modbus_strerror(errno) << "\n";
  }
  return rc;
}

bool ModbusUtils::writeRegister(modbus_t* ctx,
                                int addr,
                                uint16_t value) {
  if (!ensureCtx(ctx, "writeRegister")) return false;
  int rc = ::modbus_write_register(ctx, addr, value);
  if (rc == -1) {
    std::cerr << "[writeRegister] " << modbus_strerror(errno) << "\n";
    return false;
  }
  return true;
}

int ModbusUtils::writeRegisters(modbus_t* ctx,
                                int addr,
                                const std::vector<uint16_t>& src) {
  if (!ensureCtx(ctx, "writeRegisters")) return -1;
  int rc = ::modbus_write_registers(ctx, addr, src.size(), src.data());
  if (rc == -1) {
    std::cerr << "[writeRegisters] " << modbus_strerror(errno) << "\n";
  }
  return rc;
}

int ModbusUtils::readCoils(modbus_t* ctx,
                           int addr,
                           int nb,
                           std::vector<uint8_t>& dest) {
  if (!ensureCtx(ctx, "readCoils")) return -1;
  dest.assign(nb, 0);
  int rc = ::modbus_read_bits(ctx, addr, nb, dest.data());
  if (rc == -1) {
    std::cerr << "[readCoils] " << modbus_strerror(errno) << "\n";
  }
  return rc;
}

bool ModbusUtils::writeCoil(modbus_t* ctx,
                            int addr,
                            bool status) {
  if (!ensureCtx(ctx, "writeCoil")) return false;
  int rc = ::modbus_write_bit(ctx, addr, status ? 1 : 0);
  if (rc == -1) {
    std::cerr << "[writeCoil] " << modbus_strerror(errno) << "\n";
    return false;
  }
  return true;
}

int ModbusUtils::writeCoils(modbus_t* ctx,
                            int addr,
                            const std::vector<uint8_t>& src) {
  if (!ensureCtx(ctx, "writeCoils")) return -1;
  int rc = ::modbus_write_bits(ctx, addr, src.size(), src.data());
  if (rc == -1) {
    std::cerr << "[writeCoils] " << modbus_strerror(errno) << "\n";
  }
  return rc;
}

bool ModbusUtils::readRegister(modbus_t* ctx,
                               int addr,
                               uint16_t& out_value) {
  std::vector<uint16_t> v;
  int rc = readRegisters(ctx, addr, 1, v);
  if (rc == 1) {
    out_value = v[0];
    return true;
  }
  return false;
}

bool ModbusUtils::readCoil(modbus_t* ctx,
                           int addr,
                           bool& out_status) {
  std::vector<uint8_t> v;
  int rc = readCoils(ctx, addr, 1, v);
  if (rc == 1) {
    out_status = (v[0] != 0);
    return true;
  }
  return false;
}
