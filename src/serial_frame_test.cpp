// src/serial_frame_test.cpp

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>

int main(int argc, char** argv) {
  const char* device = (argc > 1 ? argv[1] : "/dev/ttyS0");
  int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) { perror("open"); return 1; }

  struct termios tio{};
  tio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
  tio.c_iflag = 0; tio.c_oflag = 0; tio.c_lflag = 0;
  tio.c_cc[VMIN] = 1; tio.c_cc[VTIME] = 0;
  tcflush(fd, TCIOFLUSH);
  if (tcsetattr(fd, TCSANOW, &tio) < 0) { perror("tcsetattr"); close(fd); return 1; }

  std::cout << ">>> Sending on " << device << " every 20ms\n";

  const int max_rpm = 3500;
  const int16_t angle_min = -18000, angle_max = 17999;
  const double span = angle_max - angle_min;

  for (int rpm = max_rpm; rpm >= 0; --rpm) {
    uint8_t frame[8] = {};
    uint16_t rpm_v = static_cast<uint16_t>(rpm);
    frame[0] = rpm_v & 0xFF; frame[1] = rpm_v >> 8;

    double frac = double(max_rpm - rpm) / max_rpm;
    int16_t ang = int16_t(std::round(angle_min + frac * span));
    frame[2] = ang & 0xFF; frame[3] = ang >> 8;

    frame[4] = 1;           // valid speed
    if (rpm > 0)  frame[4] |= (1<<1);
    if (rpm < 0)  frame[4] |= (1<<2);

    if (write(fd, frame, 8) != 8) perror("write");

    std::cout << "\rRPM=" << rpm_v << " Angle=" << (ang/100.0) << "°   " << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "\nDone\n";
  close(fd);
  return 0;
}
