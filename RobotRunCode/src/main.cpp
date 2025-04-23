///////////////////////////////////////////////////////////////////////////////////
// Written By: Watson and Team
//
// Team: Watson and Friends
//
///////////////////////////////////////////////////////////////////////////////////

#include <RobotIO.h>
#include "RSLK_Pins.h"
#include "SimpleRSLK.h"

Romi_Motor_Power left_motor;
Romi_Motor_Power right_motor;

unsigned long last_wisker_send = 0;         // Timestamp of last whisker data send
constexpr unsigned long WISKER_INTERVAL = 200;  // Send whisker data every 200 ms


WiskerDataUnion b_wisker_data{};            // Current whisker state
WiskerDataUnion b_old_wisker_data{};        // Last whisker state for edge detection
uint8_t c_wisker_triggers = 0;              // Incremented whenever a whisker is newly pressed

RobotControlUnion b_robot_control{};        // Holds most recent motor command received

bool previous_dance_state = false;          // Tracks when dance mode was last active

void dance_mode();                          // Forward declaration
void rouge_mode();                          // Forward declaration


void read_wisker_data();
void write_controller_data();

void setup() {
    Serial.begin(115200);                   // Start USB serial communication

    setupRSLK();                            // Set up platform-specific RSLK components

    // Initialize motor driver pins
    left_motor.begin(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);
    right_motor.begin(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);

    // Configure whisker sensor pins
    pinMode(BP_SW_PIN_0, INPUT_PULLUP);
    pinMode(BP_SW_PIN_1, INPUT_PULLUP);
    pinMode(BP_SW_PIN_2, INPUT_PULLUP);
    pinMode(BP_SW_PIN_3, INPUT_PULLUP);
    pinMode(BP_SW_PIN_4, INPUT_PULLUP);
    pinMode(BP_SW_PIN_5, INPUT_PULLUP);

    pinMode(LP_LEFT_BTN, INPUT_PULLUP);    // Launchpad side button
    pinMode(RED_LED, OUTPUT);              // LED indicator
    digitalWrite(RED_LED, HIGH);           // Turn on LED

    left_motor.enableMotor();              // Enable H-bridge motor driver
    right_motor.enableMotor();

    left_motor.directionForward();         // Default direction
    right_motor.directionForward();

    left_motor.setSpeed(0);                // Default stopped
    right_motor.setSpeed(0);
}

void loop() {
    // Listen for incoming motor command from ROS via serial
    if (Serial.available() > 0) {
        const uint8_t in_char = Serial.read();
        if (in_char == SYNC_BYTE) {  // Check for framing byte
            Serial.readBytes(reinterpret_cast<char *>(&b_robot_control.data), sizeof(b_robot_control.data));
            // Enter dance mode if newly triggered
            if (b_robot_control.robot_control.rouge_mode) {
                rouge_mode();
            }
            else if (b_robot_control.robot_control.dance_mode) {
                dance_mode();
            }
            // Update motors only when not dancing
            else {
                write_controller_data();
            }
        }
    }

    // Periodically send whisker sensor data to ROS
    unsigned long now = millis();
    if (now - last_wisker_send >= WISKER_INTERVAL) {
        last_wisker_send = now;

        read_wisker_data();  // Poll sensors and update count

        Serial.write(SYNC_BYTE);                                 // Frame header
        Serial.write(b_wisker_data.data, sizeof(b_old_wisker_data.data));  // Send binary struct
    }
}

void read_wisker_data() {
    b_old_wisker_data = b_wisker_data;  // Save previous state

    // Read all six whisker inputs
    b_wisker_data.wisker_data.w1 = digitalRead(BP_SW_PIN_0);
    b_wisker_data.wisker_data.w2 = digitalRead(BP_SW_PIN_1);
    b_wisker_data.wisker_data.w3 = digitalRead(BP_SW_PIN_2);
    b_wisker_data.wisker_data.w4 = digitalRead(BP_SW_PIN_3);
    b_wisker_data.wisker_data.w5 = digitalRead(BP_SW_PIN_4);
    b_wisker_data.wisker_data.w6 = digitalRead(BP_SW_PIN_5);

    // Increment trigger counter for newly pressed whiskers
    if (b_wisker_data.wisker_data.w1 != b_old_wisker_data.wisker_data.w1 && !b_wisker_data.wisker_data.w1) c_wisker_triggers++;
    if (b_wisker_data.wisker_data.w2 != b_old_wisker_data.wisker_data.w2 && !b_wisker_data.wisker_data.w2) c_wisker_triggers++;
    if (b_wisker_data.wisker_data.w3 != b_old_wisker_data.wisker_data.w3 && !b_wisker_data.wisker_data.w3) c_wisker_triggers++;
    if (b_wisker_data.wisker_data.w4 != b_old_wisker_data.wisker_data.w4 && !b_wisker_data.wisker_data.w4) c_wisker_triggers++;
    if (b_wisker_data.wisker_data.w5 != b_old_wisker_data.wisker_data.w5 && !b_wisker_data.wisker_data.w5) c_wisker_triggers++;
    if (b_wisker_data.wisker_data.w6 != b_old_wisker_data.wisker_data.w6 && !b_wisker_data.wisker_data.w6) c_wisker_triggers++;

    b_wisker_data.wisker_data.wisker_count = c_wisker_triggers;  // Update total whisker hits
}

void dance_mode() {
    c_wisker_triggers = 0;                 // Reset trigger counter on dance activation
    unsigned long t = millis() % 2000;

    if (t < 500) {
        left_motor.directionForward();
        right_motor.directionBackward();
    } else if (t < 1000) {
        left_motor.directionBackward();
        right_motor.directionForward();
    } else if (t < 1500) {
        left_motor.directionForward();
        right_motor.directionForward();
    } else {
        left_motor.directionBackward();
        right_motor.directionBackward();
    }

    left_motor.setSpeed(70);
    right_motor.setSpeed(70);
}

void rouge_mode() {
    c_wisker_triggers = 0;  // Reset trigger counter when entering rouge mode
    unsigned long t = millis() % 3000;

    if (t < 400) {
        // Quick spin right
        left_motor.directionForward();
        right_motor.directionBackward();
        left_motor.setSpeed(100);
        right_motor.setSpeed(100);
    } else if (t < 1000) {
        // Sudden dash forward
        left_motor.directionForward();
        right_motor.directionForward();
        left_motor.setSpeed(120);
        right_motor.setSpeed(120);
    } else if (t < 1600) {
        // Freeze—dramatic pause
        left_motor.setSpeed(0);
        right_motor.setSpeed(0);
    } else if (t < 2200) {
        // Sharp reverse burst
        left_motor.directionBackward();
        right_motor.directionBackward();
        left_motor.setSpeed(90);
        right_motor.setSpeed(90);
    } else {
        // Wiggle left
        left_motor.directionBackward();
        right_motor.directionForward();
        left_motor.setSpeed(80);
        right_motor.setSpeed(80);
    }
}


void write_controller_data() {
    // Set direction based on boolean field from ROS
    b_robot_control.robot_control.left_motor_dir ?
        left_motor.directionBackward() : left_motor.directionForward();

    b_robot_control.robot_control.right_motor_dir ?
        right_motor.directionBackward() : right_motor.directionForward();

    // Apply speed values
    left_motor.setSpeed(b_robot_control.robot_control.left_motor_speed);
    right_motor.setSpeed(b_robot_control.robot_control.right_motor_speed);
}
