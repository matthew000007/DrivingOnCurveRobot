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

CurveRobot test_1(6, 5, 7, 8);
CurveRobot test_2(6, 5, 7, 8, 150);

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
    test_1.runForward();
    test_2.runForward();
}

void loop() {
    int test_1_target = test_1.getTargetState();
    int test_2_target = test_2.getTargetState();

    if (test_1.getState() == state_forward && test_1_target != state_forward) {
        test_1.setDuration();
        test_1.stepBack();
    }

    if (test_2.getState() == state_forward && test_2_target != state_forward) {
        test_2.setDuration();
        test_2.stepBack();
    }

    switch (test_1.getTargetState(1)) {
        case state_forward:
            test_1.runForward();
            break;
        case turn_right:
            test_1.steerRight();
            break;
        case turn_left:
            test_1.steerLeft();
            break;
    }

    switch (test_1.getTargetState(1)) {
        case state_forward:
            test_2.runForward();
            break;
        case turn_right:
            test_2.steerRight();
            break;
        case turn_left:
            test_2.steerLeft();
            break;
    }
}