#include "modbus_utils.h"

#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>

int main(int argc, char** argv) {
  const char* device = (argc>1?argv[1]:"/dev/ttyS0");
  test_modbus_485::ModbusUtils mb;
  modbus_t* ctx = nullptr;
  if (!mb.openRTU(ctx, device, 115200, 'N', 8, 1, /*slave_id*/1)) {
    std::cerr<<"ERROR: cannot open RTU port\n"; return 1;
  }

  const int start_rpm = 3500;
  const int steps     = start_rpm + 1;
  const double amin   = -180.0, amax = +179.99;

  for (int i = 0; i < steps; ++i) {
    int rpm = start_rpm - i;
    double frac = double(i)/(steps-1);
    double angle_d = amin + frac*(amax-amin);
    int16_t angle_val = static_cast<int16_t>(std::round(angle_d * 100.0));

    // 1) 홀딩 레지스터에 RPM/Angle 쓰기
    std::vector<uint16_t> regs = {
      static_cast<uint16_t>(rpm),
      static_cast<uint16_t>(angle_val)
    };
    if (mb.writeRegisters(ctx, /*addr=*/0, regs) < 0) {
      std::cerr<<"WriteRegs failed\n"; break;
    }

    // 2) 코일에 Valid/Fwd/Bwd 쓰기
    std::vector<uint8_t> coils = {
      1,                           // valid
      uint8_t(rpm>0),              // fwd bit
      uint8_t(rpm<0)               // bwd bit
    };
    if (mb.writeCoils(ctx, /*addr=*/0, coils) < 0) {
      std::cerr<<"WriteCoils failed\n"; break;
    }

    std::cout<<"\rSent → RPM="<<rpm
             <<"  Angle="<<(angle_val/100.0)<<"°     "<<std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout<<"\nDone\n";
  mb.close(ctx);
  return 0;
}
