
#ifndef SERIAL_SERVO_BUS__H
#define SERIAL_SERVO_BUS__H

#include <cstdint>

bool LobotSerialServoMove(int fd, uint8_t id, int16_t position, uint16_t time); // 기본 제어
bool LobotSerialServoStopMove(int fd, uint8_t id); // 스탑
bool LobotSerialServoSetID(int fd, uint8_t oldID, uint8_t newID); // ID 세팅
bool LobotSerialServoSetMode(int fd, uint8_t id, uint8_t Mode, int16_t Speed); // 모드 세팅..? 뭐지?
bool LobotSerialServoLoad(int fd, uint8_t id); // 부하
bool LobotSerialServoUnload(int fd, uint8_t id); // 부하 안걸림

bool LobotSerialServoReadPosition(int fd, uint8_t id, uint16_t &position);
bool LobotSerialServoReadVin(int fd, uint8_t id, uint16_t &vin);

#endif // SERIAL_SERVO_BUS__H
