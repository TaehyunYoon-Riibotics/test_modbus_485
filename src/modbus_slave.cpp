// src/modbus_slave.cpp

#include <modbus.h>
#include <iostream>
#include <cassert>

int main() {
  // 1) RTU 슬레이브(ID=1) 포트 열기
  modbus_t* ctx = modbus_new_rtu("/dev/ttyS1", 115200, 'N', 8, 1);
  assert(ctx);
  if (modbus_set_slave(ctx, 1) == -1 || modbus_connect(ctx) == -1) {
    std::cerr << "Slave 초기화 실패\n";
    return 1;
  }

  // 2) 레지스터 맵(100개) 생성
  modbus_mapping_t* mb_map = modbus_mapping_new(0, 0, 100, 0);
  assert(mb_map);

  std::cout << "Modbus RTU Slave: /dev/ttyS1, ID=1, 대기중...\n";
  uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];

  // 3) 요청-응답 루프
  while (true) {
    int rc = modbus_receive(ctx, query);
    if (rc > 0) {
      modbus_reply(ctx, query, rc, mb_map);
    }
  }

  // (실제론 여기까지 도달하지 않음)
  modbus_mapping_free(mb_map);
  modbus_close(ctx);
  modbus_free(ctx);
  return 0;
}
