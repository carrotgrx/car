#include "pid.h"

float Vertical_Kp, Vertical_Kd;
float Velocity_Kp, Velocity_Ki;
float Turn_Kp, Turn_Kd;
uint8_t stop;

// 直立环pid控制器
// 输入：期望角度，当前角度， 当前角速度
uint32_t Vertical(float med, float angle, float gyro){
    uint32_t temp;
    temp = Vertical_Kp * (med - angle) - Vertical_Kd * gyro;
    return temp;
}

// 速度环pid控制器
// 输入：期望速度，左编码器，右编码器
uint32_t Velocity(uint32_t target, uint32_t encoderL, uint32_t encoderR){
    static uint32_t errLowLast, encoderS;
    static float k = 0.7;
    uint32_t error, errLow, temp;
    error = target - (encoderL + encoderR) - target;
    errLow = (1 - k) * error + k * errLowLast;
    errLowLast = errLow;
    encoderS += errLow;
    encoderS = encoderS > 20000 ? 20000 : (encoderS < (-20000) ? (-20000) : encoderS);
    if (stop) {
        encoderS = 0;
        stop = 0;
    }
    temp = Velocity_Kp * error + Velocity_Ki * encoderS;
    return temp;
}

// 转向pid控制器
// 输入：角速度，角度值
uint32_t Turn(float gyro, uint8_t target){
    uint32_t temp;
    temp = Turn_Kp * target - Turn_Kd * gyro;
    return temp;
}
