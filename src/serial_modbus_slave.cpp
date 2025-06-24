#include "modbus_utils.h"

#include <random>
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

  test_modbus_485::ModbusUtils mb;
  modbus_t* ctx = nullptr;
  if (!mb.openRTU(ctx, device, 115200, 'N', 8, 1, slave_id)) {
    std::cerr << "ERROR: cannot open RTU port\n";
    return 1;
  }

  modbus_mapping_t* mb_map = modbus_mapping_new(
    10, 0, 100, 0
  );
  assert(mb_map);

  // 랜덤 생성기 세팅
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<>  d_v_raw(0, 8000);
  std::uniform_int_distribution<>  d_i_raw(-10000, 10000);
  std::uniform_int_distribution<>  d_soc_raw(0, 1000);
  std::uniform_int_distribution<>  d_temp(-50, 125);
  std::uniform_int_distribution<>  d_errs(0, 0x3F);        // 6비트 오류
  std::uniform_int_distribution<>  d_cycle(0, 20000);

  std::cout << "Modbus RTU Slave: " << device
            << ", ID=" << slave_id
            << ", waiting for requests...\n";

  uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];
  while (true) {
    int rc = modbus_receive(ctx, query);
    if (rc <= 0) continue;

    uint8_t fc = query[1];
    if (fc == MODBUS_FC_WRITE_MULTIPLE_REGISTERS && rc >= 7) {
      int addr = (query[2]<<8)|query[3],
          qty  = (query[4]<<8)|query[5];
      std::cout << "[Slave] WRITE_REGS @"
                << addr << " cnt=" << qty
                << " → RPM=" << mb_map->tab_registers[0]
                << " Angle="<<(int16_t(mb_map->tab_registers[1])/100.0)
                << "°\n";
    }
    else if (fc == MODBUS_FC_WRITE_MULTIPLE_COILS && rc >= 7) {
      int addr = (query[2]<<8)|query[3],
          qty  = (query[4]<<8)|query[5];
      std::cout << "[Slave] WRITE_COILS @"
                << addr << " cnt=" << qty << ":";
      for (int i = 0; i < qty && addr+i < mb_map->nb_bits; ++i)
        std::cout << " " << int(mb_map->tab_bits[addr+i]);
      std::cout<<"\n";
    }

    // 랜덤 배터리/충전기 상태 생성
    int v_raw    = d_v_raw(gen);
    int i_raw    = d_i_raw(gen);
    int soc_raw  = d_soc_raw(gen);
    int temp_b   = d_temp(gen);
    int errs     = d_errs(gen);
    int cycle_ct = d_cycle(gen);

    // 배터리: 주소 10..13
    mb_map->tab_registers[10] = uint16_t(v_raw);
    mb_map->tab_registers[11] = uint16_t(int16_t(i_raw));
    mb_map->tab_registers[12] = uint16_t(soc_raw);
    mb_map->tab_registers[13] = uint16_t(int16_t(temp_b));

    // 충전기: 주소 20..21, cycle 22
    mb_map->tab_registers[20] = uint16_t(d_v_raw(gen));
    mb_map->tab_registers[21] = uint16_t(int16_t(d_i_raw(gen)));
    mb_map->tab_registers[22] = uint16_t(cycle_ct);

    // 오류비트: bits 4..9
    for (int b = 0; b < 6; ++b) {
      mb_map->tab_bits[4 + b] = ((errs >> b) & 1);
    }

    // 로깅
    std::cout << "[Slave] Batt→ V="<<(v_raw/10.0)
              <<"V I="<<(i_raw/100.0)
              <<"A SOC="<<(soc_raw/10.0)
              <<"% T="<<temp_b<<"°C\n"
              <<"[Slave] Charger→ V="<<(mb_map->tab_registers[20]/10.0)
              <<"V I="<<(int16_t(mb_map->tab_registers[21])/100.0)
              <<"A Cycle="<<cycle_ct<<"\n"
              <<"[Slave] ErrBits=";
    for (int b = 0; b < 6; ++b) std::cout << ((errs>>b)&1);
    std::cout<<"\n";

    // 자동 응답
    modbus_reply(ctx, query, rc, mb_map);
  }

  modbus_mapping_free(mb_map);
  mb.close(ctx);
  return 0;
}
