#include "modbus_utils.h"

#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <cstdint>

int main(int argc, char** argv) {
  const char* device = (argc>1?argv[1]:"/dev/ttyS0");
  test_modbus_485::ModbusUtils mb;
  modbus_t* ctx = nullptr;
  if (!mb.openRTU(ctx, device, 115200, 'N', 8, 1, /*slave_id*/1)) {
    std::cerr<<"ERROR: cannot open RTU port\n"; return 1;
  }

  while (true) {
    // 1) 홀딩 레지스터 읽기 (RPM, Angle)
    std::vector<uint16_t> regs;
    if (mb.readRegisters(ctx, 0, 2, regs) != 2) {
      std::cerr<<"ReadRegs failed\n"; continue;
    }
    int rpm        = int16_t(regs[0]);
    int16_t angle_v= int16_t(regs[1]);

    // 2) 코일 읽기 (Valid, Fwd, Bwd)
    std::vector<uint8_t> coils;
    if (mb.readCoils(ctx, 0, 3, coils) != 3) {
      std::cerr<<"ReadCoils failed\n"; continue;
    }
    bool valid   = coils[0];
    bool forward = coils[1];
    bool backward= coils[2];

    if (valid) {
      std::cout<<"Recv → RPM="<<rpm
               <<", Angle="<<(angle_v/100.0)<<"°"
               <<", Fwd="<<forward
               <<", Bwd="<<backward
               <<"\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  mb.close(ctx);
  return 0;
}
