// src/serial_can_test.cpp
// 20 ms 간격으로 8바이트 프레임(Drive+Steer)을 RS-485 포트에 쏘는 테스트

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>

int main(int argc, char** argv) {
  const char* device = (argc > 1 ? argv[1] : "/dev/ttyS0");
  int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    perror("open");
    return 1;
  }

  // 1) 포트 설정: 115200, 8N1, raw 모드
  struct termios tio;
  std::memset(&tio, 0, sizeof(tio));
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tio.c_cflag = CS8 | CLOCAL | CREAD;
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_cc[VMIN]  = 1;
  tio.c_cc[VTIME] = 0;
  tcflush(fd, TCIOFLUSH);
  if (tcsetattr(fd, TCSANOW, &tio) < 0) {
    perror("tcsetattr");
    close(fd);
    return 1;
  }

  std::cout << ">>> Sending 8-byte frames on " << device << " every 20 ms\n";

  const int   start_rpm = 3500;
  const int   steps     = start_rpm + 1;
  const double angle_min = -180.00;
  const double angle_max = +179.99;

  for (int i = 0; i < steps; ++i) {
    int rpm = start_rpm - i;                        // 3500 → 0
    double frac = double(i) / double(steps - 1);    // 0.0 → 1.0
    double angle_deg = angle_min + frac * (angle_max - angle_min);
    int16_t angle_val = static_cast<int16_t>(std::round(angle_deg * 100.0));

    // 8-byte frame 구성
    uint8_t buf[8] = {};
    // BYTE0..1: RPM (UInt16)
    buf[0] =  rpm        & 0xFF;
    buf[1] = (rpm >> 8)  & 0xFF;
    // BYTE2..3: Steering Angle (Int16, 0.01° 단위)
    buf[2] =  angle_val         & 0xFF;
    buf[3] = (angle_val >> 8)   & 0xFF;
    // BYTE4: control bits
    //   bit0: always 1 (valid frame)
    //   bit1: forward (rpm>0)
    //   bit2: backward (rpm<0)
    buf[4] = 1;
    if (rpm >  0) buf[4] |= (1 << 1);
    if (rpm <  0) buf[4] |= (1 << 2);
    // BYTE5..7: reserved (0)

    // 전송
    ssize_t w = write(fd, buf, sizeof(buf));
    if (w != sizeof(buf)) {
      perror("write");
      break;
    }

    // 콘솔 표시
    std::cout << "\rRPM=" << rpm
              << "   Angle=" << (angle_val / 100.0) << "°    " << std::flush;

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "\nDone\n";
  close(fd);
  return 0;
}
