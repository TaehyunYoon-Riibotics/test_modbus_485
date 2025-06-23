// serial_modbus_slave.cpp

#include "modbus_utils.h"

#include <iostream>
#include <cassert>
#include <cstdint>

#ifndef MODBUS_FC_WRITE_SINGLE_REGISTER
# define MODBUS_FC_WRITE_SINGLE_REGISTER       0x06
# define MODBUS_FC_WRITE_MULTIPLE_REGISTERS   0x10
# define MODBUS_FC_WRITE_SINGLE_COIL          0x05
# define MODBUS_FC_WRITE_MULTIPLE_COILS       0x0F
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
    // 3) 요청 수신
    int rc = modbus_receive(ctx, query);
    if (rc <= 0) {
      continue;  // 타임아웃 또는 에러
    }

    uint8_t fc = query[1];
    switch (fc) {
      case MODBUS_FC_WRITE_SINGLE_REGISTER: {
        if (rc >= 6) {
          int addr  = (query[2] << 8) | query[3];
          int value = (query[4] << 8) | query[5];
          if (addr < mb_map->nb_registers) {
            mb_map->tab_registers[addr] = value;
            std::cout << "[Slave] WRITE_SINGLE Reg[" << addr 
                      << "] = " << value << "\n";
          }
        }
        break;
      }

      case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
        if (rc >= 7) {
          int addr = (query[2] << 8) | query[3];
          int qty  = (query[4] << 8) | query[5];
          std::cout << "[Slave] WRITE_MULTIPLE Reg[" << addr
                    << "] count=" << qty << ":";
          if (addr + 0 < mb_map->nb_registers) {
            uint16_t rpm_raw = mb_map->tab_registers[addr + 0];
            std::cout << " RPM=" << rpm_raw;
          }
          if (addr + 1 < mb_map->nb_registers) {
            int16_t ang_raw = static_cast<int16_t>(mb_map->tab_registers[addr + 1]);
            std::cout << " Angle=" << (ang_raw/100.0) << "°";
          }
          std::cout << "\n";
        }
        break;
      }

      case MODBUS_FC_WRITE_SINGLE_COIL: {
        if (rc >= 6) {
          int addr   = (query[2] << 8) | query[3];
          int status = (query[4] == 0xFF) ? 1 : 0;
          if (addr < mb_map->nb_bits) {
            mb_map->tab_bits[addr] = status;
            std::cout << "[Slave] WRITE_SINGLE_COIL Co[" << addr 
                      << "] = " << status << "\n";
          }
        }
        break;
      }

      case MODBUS_FC_WRITE_MULTIPLE_COILS: {
        if (rc >= 7) {
          int addr = (query[2] << 8) | query[3];
          int qty  = (query[4] << 8) | query[5];
          std::cout << "[Slave] WRITE_MULTIPLE_COILS Co[" << addr
                    << "] count=" << qty << ":";
          for (int i = 0; i < qty && (addr+i) < mb_map->nb_bits; ++i) {
            std::cout << " " << int(mb_map->tab_bits[addr + i]);
          }
          std::cout << "\n";
        }
        break;
      }

      default:
        // read 요청 등은 modbus_reply()가 자동 처리
        break;
    }

    // 4) 항상 4개 코일 상태 출력
    std::cout << "[Slave] Coils: "
              << "valid="   << int(mb_map->tab_bits[0]) << ", "
              << "fwd="     << int(mb_map->tab_bits[1]) << ", "
              << "bwd="     << int(mb_map->tab_bits[2]) << ", "
              << "enable="  << int(mb_map->tab_bits[3]) << "\n";

    // 5) 자동 응답
    modbus_reply(ctx, query, rc, mb_map);
  }

  // 도달하지 않음
  modbus_mapping_free(mb_map);
  mb.close(ctx);
  return 0;
}
