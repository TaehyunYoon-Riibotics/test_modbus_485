// src/modbus_slave.cpp

#include <modbus.h>
#include <iostream>
#include <cassert>
#include <cstdint>

int main() {
  // 1) RTU 슬레이브(ID=1) 포트 열기
  const char* device = "/dev/ttyS0";       // <- 사용하실 포트로 변경
  const int   slave_id = 1;                // <- 필요시 ID 변경
  modbus_t* ctx = modbus_new_rtu(device, 115200, 'N', 8, 1);
  assert(ctx);
  if (modbus_set_slave(ctx, slave_id) == -1 || modbus_connect(ctx) == -1) {
    std::cerr << "ERROR: Slave 초기화 실패\n";
    return 1;
  }

  // 2) 레지스터 맵(100개) 생성
  modbus_mapping_t* mb_map = modbus_mapping_new(
    /*nb_bits*/     0,
    /*nb_input_bits*/0,
    /*nb_regs*/     100,
    /*nb_input_regs*/0
  );
  assert(mb_map);

  std::cout << "Modbus RTU Slave: " << device
            << ", ID=" << slave_id
            << ", waiting...\n";

  uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];

  // 3) 요청-응답 루프
  while (true) {
    int rc = modbus_receive(ctx, query);
    if (rc > 0) {
      // **WRITE SINGLE REGISTER** (Function Code 0x06)
      if (query[1] == MODBUS_FC_WRITE_SINGLE_REGISTER && rc >= 6) {
        int addr  = (query[2] << 8) | query[3];
        int value = (query[4] << 8) | query[5];
        std::cout << "[Slave] Reg[" << addr << "] <= " << value << "\n";
      }
      // **WRITE MULTIPLE REGISTERS** (Function Code 0x10)
      else if (query[1] == MODBUS_FC_WRITE_MULTIPLE_REGISTERS && rc >= 7) {
        int addr      = (query[2] << 8) | query[3];
        int qty       = (query[4] << 8) | query[5];
        std::cout << "[Slave] Write Multiple at Reg[" << addr
                  << "] count=" << qty << ":";
        // 실제 쓰여진 값들은 mb_map->tab_registers[addr..addr+qty-1]
        for (int i = 0; i < qty; ++i) {
          std::cout << " " << mb_map->tab_registers[addr + i];
        }
        std::cout << "\n";
      }

      //  응답: mapping 으로 자동 처리
      modbus_reply(ctx, query, rc, mb_map);
    }
  }

  // (실제로는 위에서 무한루프)
  modbus_mapping_free(mb_map);
  modbus_close(ctx);
  modbus_free(ctx);
  return 0;
}
