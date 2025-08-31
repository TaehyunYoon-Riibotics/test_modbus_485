// src/serial_modbus_slave.cpp

#include "modbus_utils.h"
#include <random>
#include <iostream>
#include <cassert>

int main(int argc, char** argv) {
    const char* device   = (argc > 1 ? argv[1] : "/dev/ttyS0");
    constexpr int slaveId = 1;
    constexpr int baud      = 115200;
    constexpr char parity   = 'N';
    constexpr int dataBits  = 8;
    constexpr int stopBits  = 1;

    test_modbus_485::ModbusUtils mb;
    modbus_t* ctx = nullptr;
    if (!mb.openRtu(ctx, device, baud, parity, dataBits, stopBits, slaveId)) {
        std::cerr << "ERROR: cannot open RTU port\n";
        return 1;
    }

    // Create mapping for 10 coils, 0 discrete inputs, 100 registers, 0 input registers
    modbus_mapping_t* mb_map = modbus_mapping_new(10, 0, 100, 0);
    assert(mb_map);

    std::cout << "Modbus RTU Slave on " << device
              << ", ID=" << slaveId << ", waiting for requests...\n";

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> d_v(0, 8000), d_i(-10000,10000),
                               d_soc(0,1000), d_t(-50,125), d_err(0,0x3F),
                               d_cycle(0,20000);

    uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];
    while (true) {
        int rc = modbus_receive(ctx, query);
        if (rc <= 0) {
            // 에러 혹은 타임아웃: 컨텍스트 리셋 시도
            mb.closeRtu(ctx);
            if (!mb.openRtu(ctx, device, baud, parity, dataBits, stopBits, slaveId)) {
                std::cerr << "[Slave] reconnection failed\n";
                break;
            }
            continue;
        }

        uint8_t functionCode = query[1];
        if (functionCode == MODBUS_FC_WRITE_MULTIPLE_REGISTERS && rc >= 7) {
            int addr = (query[2] << 8) | query[3];
            int qty  = (query[4] << 8) | query[5];
            std::cout << "[Slave] WRITE_REGS @" << addr << " cnt=" << qty
                      << " → RPM=" << mb_map->tab_registers[0]
                      << " Angle=" << (int16_t(mb_map->tab_registers[1]) / 100.0) << "°\n";
        }
        else if (functionCode == MODBUS_FC_WRITE_MULTIPLE_COILS && rc >= 7) {
            int addr = (query[2] << 8) | query[3];
            int qty  = (query[4] << 8) | query[5];
            std::cout << "[Slave] WRITE_COILS @" << addr << " cnt=" << qty << ":";
            for (int i = 0; i < qty && addr + i < mb_map->nb_bits; ++i)
                std::cout << ' ' << int(mb_map->tab_bits[addr + i]);
            std::cout << "\n";
        }

        // 랜덤하게 상태 갱신
        mb_map->tab_registers[10] = d_v(gen);
        mb_map->tab_registers[11] = uint16_t(int16_t(d_i(gen)));
        mb_map->tab_registers[12] = d_soc(gen);
        mb_map->tab_registers[13] = uint16_t(int16_t(d_t(gen)));

        mb_map->tab_registers[20] = d_v(gen);
        mb_map->tab_registers[21] = uint16_t(int16_t(d_i(gen)));
        mb_map->tab_registers[22] = d_cycle(gen);

        int errs = d_err(gen);
        for (int b = 0; b < 6; ++b) {
            mb_map->tab_bits[4 + b] = (errs >> b) & 1;
        }

        // reply with mapping
        if (modbus_reply(ctx, query, rc, mb_map) < 0) {
            std::cerr << "[Slave] reply failed, resetting connection\n";
            mb.closeRtu(ctx);
            if (!mb.openRtu(ctx, device, baud, parity, dataBits, stopBits, slaveId)) {
                std::cerr << "[Slave] reconnection failed\n";
                break;
            }
        }
    }

    modbus_mapping_free(mb_map);
    mb.closeRtu(ctx);
    return 0;
}
