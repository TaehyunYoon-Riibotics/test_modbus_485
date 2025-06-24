#include "modbus_utils.h"

#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>
#include <iomanip>

int main(int argc, char** argv) {
  const char* device = (argc > 1 ? argv[1] : "/dev/ttyS0");

  test_modbus_485::ModbusUtils mb;
  modbus_t* ctx = nullptr;
  if (!mb.openRTU(ctx, device, 115200, 'N', 8, 1, /*slave_id*/1)) {
    std::cerr << "ERROR: cannot open RTU port\n";
    return 1;
  }

  const int start_rpm  = 3500;
  const int steps      = start_rpm + 1;  // 기존대로 0..3500
  const int runs       = 6500;           // 전체 반복 횟수 추가
  const double amin    = -180.0, amax = +179.99;
  const int sleep_ms   = 20;
  const long long total_sleep = (long long)sleep_ms * steps * runs;

  // 통계용: ms 단위 누적
  using clock = std::chrono::steady_clock;
  using msd   = std::chrono::duration<double, std::milli>;
  msd t_wr_regs{0}, t_wr_coils{0}, t_rd_regs{0}, t_rd_coils{0};
  long long cnt_wr_regs = 0, cnt_wr_coils = 0, cnt_rd_regs = 0, cnt_rd_coils = 0;
  long long error_count = 0;

  auto t_start = clock::now();

  for (int rep = 0; rep < runs; ++rep) {
    for (int i = 0; i < steps; ++i) {
      int rpm = start_rpm - i;
      double frac = double(i) / double(steps - 1);
      double angle_d = amin + frac * (amax - amin);
      int16_t angle_val = static_cast<int16_t>(std::round(angle_d * 100.0));

      // writeRegisters 측정
      {
        auto t0 = clock::now();
        int rc = mb.writeRegisters(ctx, 0,
          std::vector<uint16_t>{ uint16_t(rpm), uint16_t(angle_val) }
        );
        auto t1 = clock::now();
        t_wr_regs += msd(t1 - t0);
        ++cnt_wr_regs;
        if (rc < 0) {
          std::cerr << "\n[Master] WriteRegs failed at rep " << rep << " step " << i << "\n";
          ++error_count;
        }
      }

      // writeCoils 측정 (4 비트)
      {
        std::vector<uint8_t> coils = {
          1,
          uint8_t(rpm > 0),
          uint8_t(rpm < 0),
          1  // enable bit
        };
        auto t0 = clock::now();
        int rc = mb.writeCoils(ctx, 0, coils);
        auto t1 = clock::now();
        t_wr_coils += msd(t1 - t0);
        ++cnt_wr_coils;
        if (rc < 0) {
          std::cerr << "\n[Master] WriteCoils failed at rep " << rep << " step " << i << "\n";
          ++error_count;
        }
      }

      // readRegisters 측정 (배터리 상태: addr=10, nb=4)
      {
        auto t0 = clock::now();
        std::vector<uint16_t> batt_regs;
        int rc = mb.readRegisters(ctx, 10, 4, batt_regs);
        auto t1 = clock::now();
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
          std::cerr << "\n[Master] readRegisters(batt) failed at rep " << rep << " step " << i << "\n";
          ++error_count;
        }
      }

      // readCoils 측정 (충전기 오류 비트: addr=4, nb=6)
      {
        auto t0 = clock::now();
        std::vector<uint8_t> errs;
        int rc = mb.readCoils(ctx, 4, 6, errs);
        auto t1 = clock::now();
        t_rd_coils += msd(t1 - t0);
        ++cnt_rd_coils;
        if (rc == 6) {
          std::cout << "  Errs=[";
          for (int j = 0; j < 6; ++j) {
            std::cout << int(errs[j]) << (j < 5 ? "," : "]");
          }
        } else {
          std::cerr << "\n[Master] readCoils(err) failed at rep " << rep << " step " << i << "\n";
          ++error_count;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
  }

  auto t_end = clock::now();
  auto elapsed_ms   = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
  auto effective_ms = elapsed_ms - total_sleep;
  long long total_frames = (long long)steps * runs;

  // 평균(ms) 계산
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

  mb.close(ctx);
  return 0;
}
