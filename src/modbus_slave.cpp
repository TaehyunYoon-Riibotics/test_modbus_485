#include "modbus_utils.h"

#include <iostream>
#include <cassert>
#include <cstdint>

int main() {
  const char* device   = "/dev/ttyS0";
  const int   slave_id = 1;

  test_modbus_485::ModbusUtils mb;
  modbus_t* ctx = nullptr;
  if (!mb.openRTU(ctx, device, 115200, 'N', 8, 1, slave_id)) {
    std::cerr << "ERROR: Slave 초기화 실패\n";
    return 1;
  }

  modbus_mapping_t* mb_map = modbus_mapping_new(
    /*nb_bits=*/0,
    /*nb_input_bits=*/0,
    /*nb_regs=*/100,
    /*nb_input_regs=*/0
  );
  assert(mb_map);

  std::cout << "Modbus RTU Slave: " << device
            << ", ID=" << slave_id
            << ", waiting...\n";

  uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];

  while (true) {
    int rc = modbus_receive(ctx, query);
    if (rc > 0) {
      // WRITE_SINGLE_REGISTER
      if (query[1] == MODBUS_FC_WRITE_SINGLE_REGISTER && rc >= 6) {
        int addr  = (query[2] << 8) | query[3];
        int value = (query[4] << 8) | query[5];
        std::cout << "[Slave] Reg[" << addr << "] <= " << value << "\n";
      }
      // WRITE_MULTIPLE_REGISTERS
      else if (query[1] == MODBUS_FC_WRITE_MULTIPLE_REGISTERS && rc >= 7) {
        int addr = (query[2] << 8) | query[3];
        int qty  = (query[4] << 8) | query[5];
        std::cout << "[Slave] Write Multiple at Reg[" << addr
                  << "] count=" << qty << ":";
        for (int i = 0; i < qty; ++i) {
          std::cout << " " << mb_map->tab_registers[addr + i];
        }
        std::cout << "\n";
      }
      modbus_reply(ctx, query, rc, mb_map);
    }
  }

  modbus_mapping_free(mb_map);
  mb.close(ctx);
  return 0;
}
