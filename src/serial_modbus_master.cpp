// src/serial_modbus_master.cpp

#include "modbus_utils.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>
#include <iomanip>

using namespace std::chrono;
using msd = duration<double, std::milli>;

static bool resetConnection(test_modbus_485::ModbusUtils& mb,
                            modbus_t*& ctx,
                            const char* device,
                            int baud,
                            char parity,
                            int dataBits,
                            int stopBits,
                            int slaveId)
{
    mb.closeRtu(ctx);
    std::this_thread::sleep_for(milliseconds(100));
    return mb.openRtu(ctx, device, baud, parity, dataBits, stopBits, slaveId);
}

int main(int argc, char** argv) {
    const char* device = (argc > 1 ? argv[1] : "/dev/ttyS0");
    constexpr int baud      = 115200;
    constexpr char parity   = 'N';
    constexpr int dataBits  = 8;
    constexpr int stopBits  = 1;
    constexpr int slaveId   = 1;

    test_modbus_485::ModbusUtils mb;
    modbus_t* ctx = nullptr;
    if (!mb.openRtu(ctx, device, baud, parity, dataBits, stopBits, slaveId)) {
        std::cerr << "ERROR: cannot open RTU port\n";
        return 1;
    }

    const int start_rpm  = 3500;
    const int steps      = start_rpm + 1;
    const int runs       = 6500;
    const double amin    = -180.0, amax = +179.99;
    const int sleep_ms   = 20;
    const long long total_sleep = (long long)sleep_ms * steps * runs;

    msd t_wr_regs{0}, t_wr_coils{0}, t_rd_regs{0}, t_rd_coils{0};
    long long cnt_wr_regs = 0, cnt_wr_coils = 0, cnt_rd_regs = 0, cnt_rd_coils = 0;
    long long error_count = 0;

    auto t_start = steady_clock::now();

    for (int rep = 0; rep < runs; ++rep) {
        for (int i = 0; i < steps; ++i) {
            int rpm = start_rpm - i;
            double frac = double(i) / double(steps - 1);
            double angle_d = amin + frac * (amax - amin);
            int16_t angle_val = static_cast<int16_t>(std::round(angle_d * 100.0));

            // 1) writeMultipleRegisters timing
            {
                auto t0 = steady_clock::now();
                int rc = mb.writeMultipleRegisters(ctx, 0,
                    std::vector<uint16_t>{ uint16_t(rpm), uint16_t(angle_val) }
                );
                auto t1 = steady_clock::now();
                t_wr_regs += msd(t1 - t0);
                ++cnt_wr_regs;
                if (rc < 0) {
                    std::cerr << "\n[Master] writeMultipleRegisters failed at rep " << rep << " step " << i << "\n";
                    ++error_count;
                    if (!resetConnection(mb, ctx, device, baud, parity, dataBits, stopBits, slaveId)) {
                        std::cerr << "[Master] reconnection failed\n";
                        return 2;
                    }
                    continue;
                }
            }

            // 2) writeMultipleCoils timing (4 bits)
            {
                std::vector<uint8_t> coils = {
                    1,
                    uint8_t(rpm > 0),
                    uint8_t(rpm < 0),
                    1
                };
                auto t0 = steady_clock::now();
                int rc = mb.writeMultipleCoils(ctx, 0, coils);
                auto t1 = steady_clock::now();
                t_wr_coils += msd(t1 - t0);
                ++cnt_wr_coils;
                if (rc < 0) {
                    std::cerr << "\n[Master] writeMultipleCoils failed at rep " << rep << " step " << i << "\n";
                    ++error_count;
                    if (!resetConnection(mb, ctx, device, baud, parity, dataBits, stopBits, slaveId)) {
                        return 2;
                    }
                    continue;
                }
            }

            // 3) readHoldingRegisters timing (battery @ addr=10, nb=4)
            {
                auto t0 = steady_clock::now();
                std::vector<uint16_t> batt_regs;
                int rc = mb.readHoldingRegisters(ctx, 10, 4, batt_regs);
                auto t1 = steady_clock::now();
                t_rd_regs += msd(t1 - t0);
                ++cnt_rd_regs;
                if (rc == 4) {
                    float bv  = batt_regs[0] / 10.0f;
                    float bi  = int16_t(batt_regs[1]) / 100.0f;
                    float soc = batt_regs[2] / 10.0f;
                    int   bt  = int16_t(batt_regs[3]);
                    std::cout << "\rRPM=" << std::setw(5) << rpm
                              << "  Ang=" << std::setw(7) << angle_val/100.0 << "°"
                              << "  BattV=" << std::setw(5) << bv << "V"
                              << "  BattI=" << std::setw(6) << bi << "A"
                              << "  SOC="   << std::setw(6) << soc << "%"
                              << "  T="     << std::setw(3) << bt << "°C";
                } else {
                    std::cerr << "\n[Master] readHoldingRegisters failed at rep " << rep << " step " << i << "\n";
                    ++error_count;
                    if (!resetConnection(mb, ctx, device, baud, parity, dataBits, stopBits, slaveId)) {
                        return 2;
                    }
                    continue;
                }
            }

            // 4) readCoils timing (charger error bits @ addr=4, nb=6)
            {
                auto t0 = steady_clock::now();
                std::vector<uint8_t> errs;
                int rc = mb.readCoils(ctx, 4, 6, errs);
                auto t1 = steady_clock::now();
                t_rd_coils += msd(t1 - t0);
                ++cnt_rd_coils;
                if (rc == 6) {
                    std::cout << "  Errs=[";
                    for (int j = 0; j < 6; ++j)
                        std::cout << int(errs[j]) << (j < 5 ? "," : "]");
                } else {
                    std::cerr << "\n[Master] readCoils failed at rep " << rep << " step " << i << "\n";
                    ++error_count;
                    if (!resetConnection(mb, ctx, device, baud, parity, dataBits, stopBits, slaveId)) {
                        return 2;
                    }
                    continue;
                }
            }

            std::this_thread::sleep_for(milliseconds(sleep_ms));
        }
    }

    auto t_end = steady_clock::now();
    auto elapsed_ms   = duration_cast<milliseconds>(t_end - t_start).count();
    auto effective_ms = elapsed_ms - total_sleep;
    long long total_frames = (long long)steps * runs;

    double avg_wr_regs  = cnt_wr_regs  ? t_wr_regs.count()  / cnt_wr_regs  : 0.0;
    double avg_wr_coils = cnt_wr_coils ? t_wr_coils.count() / cnt_wr_coils : 0.0;
    double avg_rd_regs  = cnt_rd_regs  ? t_rd_regs.count()  / cnt_rd_regs  : 0.0;
    double avg_rd_coils = cnt_rd_coils ? t_rd_coils.count() / cnt_rd_coils : 0.0;

    std::cout << "\n\n[Master] Done\n"
              << "Total frames:        " << total_frames << "\n"
              << "Failed ops:          " << error_count    << "\n"
              << "Elapsed total time:  " << elapsed_ms     << " ms\n"
              << "Sleep total time:    " << total_sleep    << " ms\n"
              << "Effective run time:  " << effective_ms   << " ms\n\n"
              << "Average timings (ms):\n"
              << "  writeRegisters():   " << std::fixed << std::setprecision(3) << avg_wr_regs  << " ms\n"
              << "  writeCoils():       " << avg_wr_coils << " ms\n"
              << "  readRegisters():    " << avg_rd_regs  << " ms\n"
              << "  readCoils():        " << avg_rd_coils << " ms\n";

    mb.closeRtu(ctx);
    return 0;
}
