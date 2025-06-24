#include "modbus_utils.h"

#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>

int main(int argc, char** argv) {
  const char* device = (argc > 1 ? argv[1] : "/dev/ttyS0");

  test_modbus_485::ModbusUtils mb;
  modbus_t* ctx = nullptr;
  if (!mb.openRTU(ctx, device, 115200, 'N', 8, 1, /*slave_id*/1)) {
    std::cerr << "ERROR: cannot open RTU port\n";
    return 1;
  }

  const int start_rpm = 3500;
  const int steps     = start_rpm + 1;
  const double amin   = -180.0, amax = +179.99;
  const int sleep_ms  = 1;
  const long long total_sleep = (long long)sleep_ms * steps;

  int error_count = 0;
  auto t_start = std::chrono::steady_clock::now();

  for (int i = 0; i < steps; ++i) {
    int rpm = start_rpm - i;
    double frac = double(i) / double(steps - 1);
    double angle_d = amin + frac * (amax - amin);
    int16_t angle_val = static_cast<int16_t>(std::round(angle_d * 100.0));

    // 1) Drive/Steer 쓰기
    std::vector<uint16_t> regs = {
      static_cast<uint16_t>(rpm),
      static_cast<uint16_t>(angle_val)
    };
    if (mb.writeRegisters(ctx, 0, regs) < 0) {
      std::cerr << "\n[Master] WriteRegs failed at step " << i << "\n";
      ++error_count;
    }

    std::vector<uint8_t> coils = {
      1,
      uint8_t(rpm > 0),
      uint8_t(rpm < 0),
      1
    };
    if (mb.writeCoils(ctx, 0, coils) < 0) {
      std::cerr << "\n[Master] WriteCoils failed at step " << i << "\n";
      ++error_count;
    }

    // 2) 배터리 상태 읽기
    std::vector<uint16_t> batt_regs;
    if (mb.readRegisters(ctx, 10, 4, batt_regs) == 4) {
      float bv  = batt_regs[0] / 10.0f;
      float bi  = int16_t(batt_regs[1]) / 100.0f;
      float soc = batt_regs[2] / 10.0f;
      int   bt  = int16_t(batt_regs[3]);
      std::cout << "\r[Master] RPM="<<rpm
                <<"  Angle="<<(angle_val/100.0)
                <<"°  BattV="<<bv<<"V"
                <<" BattI="<<bi<<"A"
                <<" SOC="<<soc<<"%"
                <<" T="<<bt<<"°C    ";
    }

    // 3) 충전기 오류 읽기
    std::vector<uint8_t> errs;
    if (mb.readCoils(ctx, 4, 6, errs) == 6) {
      std::cout << "  ChargerErrs=[";
      for (int j = 0; j < 6; ++j) std::cout << int(errs[j]) << (j<5? ",":"");
      std::cout << "]";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }

  auto t_end = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
  auto effective_ms = elapsed_ms - total_sleep;
  double avg_frame_time = (steps > 0) ? double(effective_ms) / steps : 0.0;

  std::cout << "\n[Master] Done\n"
            << "Total frames:        " << steps << "\n"
            << "Failed ops:          " << error_count << "\n"
            << "Elapsed:             " << elapsed_ms << " ms\n"
            << "Sleep total:         " << total_sleep << " ms\n"
            << "Effective send time: " << effective_ms << " ms\n"
            << "Avg per frame:       " << avg_frame_time << " ms\n";

  mb.close(ctx);
  return 0;
}
