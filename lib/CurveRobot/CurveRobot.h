//
// Created by nokkF on 16.05.2025.
//

#ifndef DRIVINGONCURVEROBOT_CURVEROBOT_H
#define DRIVINGONCURVEROBOT_CURVEROBOT_H

#include "inttypes.h"

class CurveRobot {
private:
    int state = 0;
    int targetState = state;
    int speed = 100;
    int duration;
    int current_speed = speed;
    int slow_speed = 35;
    int back_slow_speed = 30;
    int back_fast_speed = 50;
    int speed_left;
    int speed_right;
    int dir_left;
    int dir_right;
    int left_sensor_pin;
    int right_sensor_pin;
    int brake_k = 4;
    int speed_step = 2;
    int fast_time_threshold = 500;
    int fastTime = 0;
    int error = 0;
    int last_error = error;
    int integral = 0;
    int output = 0;
    float Kp;
    float Ki;
    float Kd;
public:
    CurveRobot();

    CurveRobot(const uint8_t &speed_left_pin, const uint8_t &speed_right_pin, const uint8_t &dir_left_pin,
               const uint8_t &dir_right_pin, const uint8_t &speed);

    CurveRobot(const uint8_t &speed_left_pin, const uint8_t &speed_right_pin, const uint8_t &dir_left_pin,
               const uint8_t &dir_right_pin);

    CurveRobot(const uint8_t &speed_left_pin, const uint8_t &speed_right_pin, const uint8_t &dir_left_pin,
               const uint8_t &dir_right_pin, const float& Kp, const float& Ki, const float& Kd);

    CurveRobot(const uint8_t &speed_left_pin, const uint8_t &speed_right_pin, const uint8_t &dir_left_pin,
               const uint8_t &dir_right_pin, const float& Kp, const float& Ki, const float& Kd, uint8_t& speed);

    void runForward();

    void steerLeft();

    void steerRight();

    void stepBack();

    void stopDriving();

    int getTargetState(int option = 0);

    int getState();

    void setDuration();

    void updatePID();

    void steerPID();

    void runForwardPID();
};

#endif //DRIVINGONCURVEROBOT_CURVEROBOT_H
