//
// Created by carrotgrx on 2024/3/30.
//

#ifndef MOTOR_PID_H
#define MOTOR_PID_H
#include "main.h"

uint32_t Vertical(float med, float angle, float gyro);
uint32_t Velocity(uint32_t target, uint32_t encoderL, uint32_t encoderR);
uint32_t Turn(float gyro, uint8_t target);

#endif //MOTOR_PID_H
