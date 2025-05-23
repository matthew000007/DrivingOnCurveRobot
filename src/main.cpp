#include <Arduino.h>
#include "../lib/CurveRobot/CurveRobot.h"

/*#define speed_left 6
#define speed_right 5
#define dir_left 7
#define dir_right 4
#define left_sensor_pin 8
#define right_sensor_pin 9

// Скорость, с которой движемся вперед (0 - 255)
#define speed_forward 35

// Коэффицент, с которым нужно затормозить одно из колес при повороте
#define brake_k 4
*/
// Коды для езды по прямой, повороту направо и налево соответственно
#define state_forward 0
#define turn_right 1
#define turn_left 2
#define stop 3

CurveRobot robot(5, 6, 4, 7, 235, 0.0, 0, 250, 2, 3);

/*void runForward() {
    // Говорим, что едем вперед
    state = state_forward;

    // Указываем скороть, с которой будем ехать впредед (чем выше значние speed_forward, тем выше скорость)
    analogWrite(speed_right, speed_forward);
    analogWrite(speed_right, speed_forward);

    // Высокий сигнал - едем вперед, низкий - назад
    digitalWrite(dir_left, HIGH);
    digitalWrite(dir_right, HIGH);
}

void steerRight() {
    state = turn_right;
    analogWrite(speed_left, speed_forward);
    analogWrite(speed_right, speed_forward / 4);
    digitalWrite(dir_left, 1);
    digitalWrite(dir_right, 1);
}

void steerLeft() {
    state = turn_left;
    analogWrite(speed_left, speed_forward / 4);
    analogWrite(speed_right, speed_forward);
    digitalWrite(dir_left, 1);
    digitalWrite(dir_right, 1);
}*/

void setup() {
    robot.reverseLeft(true);
    delay(2000);
    //robot.runForwardPID();
}

void loop() {
    robot.updatePID();
    if (robot.getPIDOutput() == 0) {
        robot.runForwardPID();
    } else {
        robot.steerPID();
    }
}