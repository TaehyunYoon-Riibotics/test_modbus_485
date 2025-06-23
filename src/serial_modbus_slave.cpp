// src/serial_modbus_slave.cpp

#include "modbus_utils.h"

#include <iostream>
#include <cassert>
#include <cstdint>

#ifndef MODBUS_FC_WRITE_SINGLE_REGISTER
# define MODBUS_FC_WRITE_SINGLE_REGISTER 0x06
# define MODBUS_FC_WRITE_MULTIPLE_REGISTERS 0x10
#endif

int main(int argc, char** argv) {
  const char* device   = (argc > 1 ? argv[1] : "/dev/ttyS0");
  const int   slave_id = 1;

  // 1) 포트 열기
  test_modbus_485::ModbusUtils mb;
  modbus_t* ctx = nullptr;
  if (!mb.openRTU(ctx, device, 115200, 'N', 8, 1, slave_id)) {
    std::cerr << "ERROR: cannot open RTU port\n";
    return 1;
  }

  // 2) 메모리 맵: 코일 4개, 레지스터 100개
  modbus_mapping_t* mb_map = modbus_mapping_new(
    /*nb_bits*/      4,
    /*nb_input_bits*/0,
    /*nb_regs*/     100,
    /*nb_input_regs*/0
  );
  assert(mb_map);

  std::cout << "Modbus RTU Slave: " << device
            << ", ID=" << slave_id
            << ", waiting for requests...\n";

  uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];
  while (true) {
    // 3) 요청 대기
    int rc = modbus_receive(ctx, query);
    if (rc <= 0) {
      // 타임아웃 또는 에러
      continue;
    }

    uint8_t fc = query[1];
    if (fc == MODBUS_FC_WRITE_SINGLE_REGISTER && rc >= 6) {
      int addr  = (query[2] << 8) | query[3];
      int value = (query[4] << 8) | query[5];
      if (addr >= 0 && addr < mb_map->nb_registers) {
        mb_map->tab_registers[addr] = value;
        std::cout << "[Slave] WRITE_SINGLE Reg[" << addr << "] = " << value << "\n";
      }
    }
    else if (fc == MODBUS_FC_WRITE_MULTIPLE_REGISTERS && rc >= 7) {
      int addr = (query[2] << 8) | query[3];
      int qty  = (query[4] << 8) | query[5];
      std::cout << "[Slave] WRITE_MULTIPLE Reg[" << addr
                << "] count=" << qty << ":";
      // regs[0] = RPM, regs[1] = angle
      if (addr + 0 < mb_map->nb_registers) {
        uint16_t raw_rpm = mb_map->tab_registers[addr + 0];
        std::cout << " RPM=" << raw_rpm;
      }
      if (addr + 1 < mb_map->nb_registers) {
        int16_t raw_ang = static_cast<int16_t>(mb_map->tab_registers[addr + 1]);
        double ang_deg = raw_ang / 100.0;
        std::cout << " Angle=" << ang_deg << "°";
      }
      std::cout << "\n";
    }

    // 4) 자동 응답 (read 요청이든 write 확인이든)
    modbus_reply(ctx, query, rc, mb_map);
  }

  // (도달하지 않음)
  modbus_mapping_free(mb_map);
  mb.close(ctx);
  return 0;
}
