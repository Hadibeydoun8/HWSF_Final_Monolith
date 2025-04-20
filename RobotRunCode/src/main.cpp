///////////////////////////////////////////////////////////////////////////////////
// Robot_Dive.INO
// Written By: Ricardo Tapia Vargas
//
// Team: Monsters Inc.
//
///////////////////////////////////////////////////////////////////////////////////


/* Defines pin configuration of robot */
#include <RobotIO.h>
#include "RSLK_Pins.h"
#include "SimpleRSLK.h"

Romi_Motor_Power left_motor;
Romi_Motor_Power right_motor;

unsigned long last_wisker_send = 0;  // NEW: timestamp tracker
const unsigned long WISKER_INTERVAL = 200; // ms


int motorSpeed = 40;
int badDriver = 0;

WiskerDataUnion b_wisker_data{};
WiskerDataUnion b_old_wisker_data{};
uint16_t c_wisker_triggers = 0;

RobotControlUnion b_robot_control{};


void read_wisker_data();
void write_controller_data();

void setup() {
    Serial.begin(115200);

    setupRSLK();


    left_motor.begin(MOTOR_L_SLP_PIN,
         MOTOR_L_DIR_PIN,
         MOTOR_L_PWM_PIN);

    right_motor.begin(MOTOR_R_SLP_PIN,
              MOTOR_R_DIR_PIN,
              MOTOR_R_PWM_PIN);



    pinMode(BP_SW_PIN_0, INPUT_PULLUP);
    pinMode(BP_SW_PIN_1, INPUT_PULLUP);
    pinMode(BP_SW_PIN_2, INPUT_PULLUP);
    pinMode(BP_SW_PIN_3, INPUT_PULLUP);
    pinMode(BP_SW_PIN_4, INPUT_PULLUP);
    pinMode(BP_SW_PIN_5, INPUT_PULLUP);

    /* Left button on Launchpad */
    pinMode(LP_LEFT_BTN, INPUT_PULLUP);
    /* Red led in rgb led */
    pinMode(RED_LED, OUTPUT);

    digitalWrite(RED_LED, HIGH);

    left_motor.enableMotor();
    right_motor.enableMotor();

    left_motor.directionForward();
    right_motor.directionForward();

    left_motor.setSpeed(motorSpeed);
    right_motor.setSpeed(motorSpeed);





    // delay(2000);  // Drive backward for 2 seconds


}

void loop() {
    // Handle incoming motor control
    if (Serial.available() > 0) {
        const uint8_t in_char = Serial.read();
        if (in_char == SYNC_BYTE) {
            Serial.readBytes(reinterpret_cast<char *>(&b_robot_control.data), sizeof(b_robot_control.data));
            write_controller_data();
        }
    }

    // Check if it's time to send wisker data
    unsigned long now = millis();
    if (now - last_wisker_send >= WISKER_INTERVAL) {
        last_wisker_send = now;

        read_wisker_data();
        Serial.write(SYNC_BYTE); // sync write byte
        Serial.write(b_wisker_data.data, sizeof(b_old_wisker_data.data));
    }
}

void read_wisker_data() {
    b_old_wisker_data = b_wisker_data;
    b_wisker_data.wisker_data.w1 = digitalRead(BP_SW_PIN_0);
    b_wisker_data.wisker_data.w2 = digitalRead(BP_SW_PIN_1);
    b_wisker_data.wisker_data.w3 = digitalRead(BP_SW_PIN_2);
    b_wisker_data.wisker_data.w4 = digitalRead(BP_SW_PIN_3);
    b_wisker_data.wisker_data.w5 = digitalRead(BP_SW_PIN_4);
    b_wisker_data.wisker_data.w6 = digitalRead(BP_SW_PIN_5);
}

void reset_wisker_count() {
    c_wisker_triggers = 0;
}

void write_controller_data() {
    b_robot_control.robot_control.left_motor_dir ?
        left_motor.directionBackward() : left_motor.directionForward();

    b_robot_control.robot_control.right_motor_dir ?
        right_motor.directionBackward() : right_motor.directionForward();

    left_motor.setSpeed(b_robot_control.robot_control.left_motor_speed);
    right_motor.setSpeed(b_robot_control.robot_control.right_motor_speed);

}
