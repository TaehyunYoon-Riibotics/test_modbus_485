// src/serial_can_recv.cpp
// 8바이트 프레임을 읽어서 언팩한 뒤 출력

#include "modbus_utils.h"
#include <iostream>
#include <cstring>
#include <cstdint>

// 읽어야 할 바이트 수만큼 블로킹해서 읽어주는 헬퍼
static ssize_t read_n(int fd, void* buf, size_t n) {
  size_t got = 0;
  auto p = reinterpret_cast<uint8_t*>(buf);
  while (got < n) {
    ssize_t r = ::read(fd, p + got, n - got);
    if      (r > 0)  got += r;
    else if (r == 0) break;
    else {
      perror("read");
      return r;
    }
  }
  return got;
}

int main(int argc, char** argv) {
  const char* device = (argc > 1 ? argv[1] : "/dev/ttyS0");

  // 1) ModbusUtils 로 RTU 포트 열기
  test_modbus_485::ModbusUtils mb;
  modbus_t* ctx = nullptr;
  if (!mb.openRTU(ctx, device, 115200, 'N', 8, 1, /*slave_id*/1)) {
    std::cerr << "ERROR: cannot open RTU port " << device << "\n";
    return 1;
  }

  // 2) 내부 fd(파일 디스크립터) 가져오기
  int fd = mb.getFd(ctx);
  if (fd < 0) {
    std::cerr << "ERROR: failed to get fd from ModbusUtils\n";
    mb.close(ctx);
    return 1;
  }

  std::cout << ">>> Listening on " << device << "\n";
  while (true) {
    uint8_t buf[8];
    if (read_n(fd, buf, sizeof(buf)) != (ssize_t)sizeof(buf)) {
      continue;
    }

    uint16_t rpm       = buf[0] | (buf[1] << 8);
    int16_t  angle_val = buf[2] | (buf[3] << 8);
    bool     forward   = buf[4] & (1 << 1);
    bool     backward  = buf[4] & (1 << 2);

    std::cout << "Received → "
              << "RPM="   << rpm
              << ", Angle=" << (angle_val / 100.0) << "°"
              << ", Fwd="   << forward
              << ", Bwd="   << backward
              << "\n";
  }

  // 3) 포트 닫기
  mb.close(ctx);
  return 0;
}
