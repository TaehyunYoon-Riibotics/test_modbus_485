#include "modbus_utils.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <cmath>   

int main(int argc, char** argv) {
  const char* device = (argc > 1 ? argv[1] : "/dev/ttyS0");

  // --- ModbusUtils 로 RTU 포트 열기 ---
  test_modbus_485::ModbusUtils mb;
  modbus_t* ctx = nullptr;
  if (!mb.openRTU(ctx, device, 115200, 'N', 8, 1, /*slave_id*/1)) {
    std::cerr << "ERROR: cannot open RTU port\n";
    return 1;
  }
  int fd = mb.getFd(ctx);
  if (fd < 0) {
    std::cerr << "ERROR: failed to get fd\n";
    mb.close(ctx);
    return 1;
  }
  std::cout << ">>> Sending 8-byte frames on " << device << " every 20 ms\n";

  // 이하 원래 로직 그대로...
  const int   start_rpm = 3500;
  const int   steps     = start_rpm + 1;
  const double angle_min = -180.00;
  const double angle_max = +179.99;

  for (int i = 0; i < steps; ++i) {
    int rpm = start_rpm - i;
    double frac = double(i) / double(steps - 1);
    double angle_deg = angle_min + frac * (angle_max - angle_min);
    int16_t angle_val = static_cast<int16_t>(round(angle_deg * 100.0));

    uint8_t buf[8] = {};
    buf[0] =  rpm        & 0xFF;
    buf[1] = (rpm >> 8)  & 0xFF;
    buf[2] =  angle_val         & 0xFF;
    buf[3] = (angle_val >> 8)   & 0xFF;
    buf[4] = 1;
    if (rpm >  0) buf[4] |= (1 << 1);
    if (rpm <  0) buf[4] |= (1 << 2);

    if (write(fd, buf, sizeof(buf)) != sizeof(buf)) {
      perror("write");
      break;
    }

    std::cout << "\rRPM=" << rpm
              << "   Angle=" << (angle_val / 100.0) << "°    " << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "\nDone\n";

  // --- 포트 닫기 ---
  mb.close(ctx);
  return 0;
}
