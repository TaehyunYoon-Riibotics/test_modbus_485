// src/serial_can_recv.cpp
// 8바이트 프레임을 읽어서 언팩한 뒤 출력

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cstdint>

static ssize_t read_n(int fd, void* buf, size_t n) {
  size_t got = 0;
  auto p = reinterpret_cast<uint8_t*>(buf);
  while (got < n) {
    ssize_t r = read(fd, p + got, n - got);
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
  int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) { perror("open"); return 1; }

  // 포트 설정(마찬가지로 115200 8N1 raw)
  struct termios tio;
  std::memset(&tio, 0, sizeof(tio));
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tio.c_cflag = CS8 | CLOCAL | CREAD;
  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &tio);

  std::cout << ">>> Listening on " << device << "\n";
  while (true) {
    uint8_t buf[8];
    if (read_n(fd, buf, 8) != 8) continue;

    uint16_t rpm       = buf[0] | (buf[1] << 8);
    int16_t  angle_val = buf[2] | (buf[3] << 8);
    bool     forward   = buf[4] & (1<<1);
    bool     backward  = buf[4] & (1<<2);

    std::cout << "Received → "
              << "RPM="   << rpm
              << ", Angle=" << (angle_val / 100.0) << "°"
              << ", Fwd="   << forward
              << ", Bwd="   << backward
              << "\n";
  }

  close(fd);
  return 0;
}
