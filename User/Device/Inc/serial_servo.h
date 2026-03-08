//
// Created by CaoKangqi on 2026/3/7.
//

#ifndef G4_FRAMEWORK_SERIAL_SERVO_H
#define G4_FRAMEWORK_SERIAL_SERVO_H

#define FRAME_HEADER_1      0x55
#define FRAME_HEADER_2      0x55
#define CMD_SERVO_MOVE      0x03
#define CMD_GET_BATTERY     0x0F
#define RX_BUF_SIZE 32
#include <stdint.h>

void ServoMove(uint8_t id, uint16_t angle, uint16_t time_ms);
void ServoMoveMulti(uint8_t num_servos, uint8_t *ids, uint16_t *angles, uint16_t time_ms);

#endif //G4_FRAMEWORK_SERIAL_SERVO_H