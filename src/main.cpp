#include "modbus_utils.h"
#include <iostream>
#include <cstdlib>

static void usage(const char* prog) {
  std::cout << "Usage:\n"
               "  " << prog << " DEVICE SLAVE_ID WRITE_ADDR WRITE_VALUE [READ_ADDR]\n\n"
               "Examples:\n"
               "  " << prog << " /dev/ttyS0 1 20 42        # Reg[20]=42 쓰기 후 Reg[20] 읽기\n"
               "  " << prog << " /dev/ttyS0 1 20 42 10    # Reg[20]=42 쓰기, Reg[10] 읽기\n";
}

int main(int argc, char** argv) {
  using namespace test_modbus_485;

  if (argc < 5) {
    usage(argv[0]);
    return 1;
  }

  const std::string device    = argv[1];
  const int       slave_id    = std::atoi(argv[2]);
  const int       write_addr  = std::atoi(argv[3]);
  const uint16_t  write_value = static_cast<uint16_t>(std::atoi(argv[4]));
  const int       read_addr   = (argc >= 6 ? std::atoi(argv[5]) : write_addr);

  std::cout << "Device: " << device
            << ", Slave ID: " << slave_id
            << "\nWrite Reg[" << write_addr << "] = " << write_value
            << "\nRead  Reg[" << read_addr  << "]\n\n";

  ModbusUtils mb;
  modbus_t* ctx = nullptr;
  if (!mb.openRTU(ctx, device, 115200, 'N', 8, 1, slave_id)) {
    std::cerr << "ERROR: Cannot open " << device << "\n";
    return 2;
  }

  // 쓰기
  if (mb.writeRegister(ctx, write_addr, write_value)) {
    std::cout << "  -> Write OK\n";
  } else {
    std::cerr << "  -> Write FAILED\n";
  }

  // 읽기
  uint16_t read_value = 0;
  if (mb.readRegister(ctx, read_addr, read_value)) {
    std::cout << "  -> Read  OK: Reg[" << read_addr << "] = " << read_value << "\n";
  } else {
    std::cerr << "  -> Read  FAILED\n";
  }

  mb.close(ctx);
  std::cout << "Done\n";
  return 0;
}
