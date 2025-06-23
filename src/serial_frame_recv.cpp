// src/serial_frame_recv.cpp

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
    if (r > 0) got += r;
    else if (r < 0) { perror("read"); return r; }
    else break;
  }
  return got;
}

int main(int argc, char** argv) {
  const char* device = (argc > 1 ? argv[1] : "/dev/ttyS0");
  int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) { perror("open"); return 1; }

  struct termios tio{};
  tio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &tio);

  std::cout << ">>> Listening on " << device << "\n";
  while (true) {
    uint8_t frame[8];
    if (read_n(fd, frame, 8) != 8) continue;

    uint16_t rpm = frame[0] | (frame[1]<<8);
    int16_t  ang = frame[2] | (frame[3]<<8);
    std::cout << "Received → RPM=" << rpm
              << ", Angle=" << (ang/100.0) << "°\n";
  }
  close(fd);
  return 0;
}
