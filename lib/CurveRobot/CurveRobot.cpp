//
// Created by nokkF on 16.05.2025.
//

#include "CurveRobot.h"
#include <Arduino.h>

#define state_forward 0
#define turn_right 1
#define turn_left 2
#define stop 3

CurveRobot::CurveRobot() {}

CurveRobot::CurveRobot(const uint8_t &speed_left_pin, const uint8_t &speed_right_pin, const uint8_t &dir_left_pin,
                       const uint8_t &dir_right_pin, const uint8_t &speed) { // Конструктор с заданной скоростью
    this->speed_left = speed_left_pin;
    this->speed_right = speed_right_pin;
    this->dir_left = dir_left_pin;
    this->dir_right = dir_right_pin;
    this->speed = speed;
    this->slow_speed = speed * 0.35;
    this->back_slow_speed = slow_speed * 0.85;
    this->back_fast_speed = speed / 2;
    pinMode(speed_left_pin, OUTPUT);
    pinMode(speed_right_pin, OUTPUT);
    pinMode(dir_left_pin, OUTPUT);
    pinMode(dir_right_pin, OUTPUT);
}

CurveRobot::CurveRobot(const uint8_t &speed_left_pin, const uint8_t &speed_right_pin, const uint8_t &dir_left_pin,
                       const uint8_t &dir_right_pin) { // Конструктор со скоростью по умолчанию
    this->speed_left = speed_left_pin;
    this->speed_right = speed_right_pin;
    this->dir_left = dir_left_pin;
    this->dir_right = dir_right_pin;
    pinMode(speed_left_pin, OUTPUT);
    pinMode(speed_right_pin, OUTPUT);
    pinMode(dir_left_pin, OUTPUT);
    pinMode(dir_right_pin, OUTPUT);
}

CurveRobot::CurveRobot(const uint8_t &speed_left_pin, const uint8_t &speed_right_pin, const uint8_t &dir_left_pin,
                       const uint8_t &dir_right_pin, const float &Kp, const float &Ki, const float &Kd) {
    this->speed_left = speed_left_pin;
    this->speed_right = speed_right_pin;
    this->dir_left = dir_left_pin;
    this->dir_right = dir_right_pin;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    pinMode(speed_left_pin, OUTPUT);
    pinMode(speed_right_pin, OUTPUT);
    pinMode(dir_left_pin, OUTPUT);
    pinMode(dir_right_pin, OUTPUT);
}

CurveRobot::CurveRobot(const uint8_t &speed_left_pin, const uint8_t &speed_right_pin, const uint8_t &dir_left_pin,
                       const uint8_t &dir_right_pin, const float &Kp, const float &Ki, const float &Kd,
                       uint8_t &speed) {
    this->speed_left = speed_left_pin;
    this->speed_right = speed_right_pin;
    this->dir_left = dir_left_pin;
    this->dir_right = dir_right_pin;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->speed_left = speed_left_pin;
    this->speed_right = speed_right_pin;
    this->dir_left = dir_left_pin;
    this->dir_right = dir_right_pin;
    this->speed = speed;
    this->slow_speed = speed * 0.35;
    this->back_slow_speed = slow_speed * 0.85;
    this->back_fast_speed = speed / 2;
    pinMode(speed_left_pin, OUTPUT);
    pinMode(speed_right_pin, OUTPUT);
    pinMode(dir_left_pin, OUTPUT);
    pinMode(dir_right_pin, OUTPUT);
}

void CurveRobot::runForward() {
    state = state_forward;
    fastTime += 1;
    if (fastTime < fast_time_threshold) {
        current_speed = slow_speed;
    } else {
        current_speed = min(current_speed + speed_step, speed);
    }

    analogWrite(speed_left, current_speed);
    analogWrite(speed_right, current_speed);
    digitalWrite(dir_left, 1);
    digitalWrite(dir_right, 1);
}

void CurveRobot::steerRight() {
    state = turn_right;
    fastTime = 0;

    analogWrite(speed_right, 0);
    analogWrite(speed_left, speed);

    digitalWrite(dir_right, 1);
    digitalWrite(dir_left, 0);
}

void CurveRobot::steerLeft() {
    state = turn_left;
    fastTime = 0;

    analogWrite(speed_left, 0);
    analogWrite(speed_right, speed);

    digitalWrite(dir_left, 1);
    digitalWrite(dir_right, 1);
}

int CurveRobot::getTargetState(
        int option) { // Если option = 0, выполняется замер; Если option = 1, просто возвращается значение targetState
    if (option == 0) {
        boolean left = !digitalRead(left_sensor_pin);
        boolean right = !digitalRead(right_sensor_pin);
        if (!left && !right) {
            targetState = state_forward;
            return targetState;
        } else if (left) {
            targetState = turn_left;
            return targetState;
        } else if (right) {
            targetState = turn_right;
            return targetState;
        } else if (left && right) {
            targetState = stop;
            return targetState;
        }
    } else if (option == 1) {
        return targetState;
    }
}

void CurveRobot::stepBack() {
    if (!duration) {
        return;
    }

    int leftSpeed = (state == turn_left) ? back_slow_speed : back_fast_speed;
    int rightSpeed = (state == turn_right) ? back_slow_speed : back_fast_speed;
    analogWrite(speed_left, leftSpeed);
    analogWrite(speed_right, rightSpeed);
    digitalWrite(dir_left, 1);
    digitalWrite(dir_right, 1);
    delay(duration);
}

void CurveRobot::stopDriving() {
    analogWrite(speed_left, 0);
    analogWrite(speed_right, 0);
    digitalWrite(dir_left, 0);
    digitalWrite(dir_right, 0);
}

void CurveRobot::setDuration() {
    duration = (current_speed > slow_speed) ? current_speed : 0;
}

int CurveRobot::getState() {
    return state;
}

void CurveRobot::updatePID() {
    int right = !digitalRead(right_sensor_pin);
    int left = !digitalRead(left_sensor_pin);
    error = right - left;
    integral += error;
    constrain(integral, -100, 100);
    float derivative = error - last_error;
    last_error = error;
    output = Kp * error + Ki * integral + Kd * derivative;
    constrain(output, -100, 100);
}

void CurveRobot::steerPID() {
    state = turn_left;
    int leftSpeed = speed - output;
    constrain(leftSpeed, 0, 255);
    int rightSpeed = speed + output;
    constrain(rightSpeed, 0, 255);
    analogWrite(speed_left, leftSpeed);
    analogWrite(speed_right, rightSpeed);
    digitalWrite(dir_left, 1);
    digitalWrite(dir_right, 1);
}

void CurveRobot::runForwardPID() {
    analogWrite(speed_left, speed);
    analogWrite(speed_right, speed);
    digitalWrite(dir_left, 1);
    digitalWrite(dir_right, 1);
}