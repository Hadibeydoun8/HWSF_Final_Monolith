#include <Arduino.h>
#include <RobotIO.h>  // Defines the ControllerData struct and union

// Define pin mappings for joystick buttons
#define PIN_BUTTON_SELECT 2

#define PIN_BUTTON_RIGHT 3
#define PIN_BUTTON_UP    4
#define PIN_BUTTON_DOWN  5
#define PIN_BUTTON_LEFT  6

// Define analog input pins for joystick axes
#define PIN_X_JOYSTICK A0
#define PIN_Y_JOYSTICK A1

// Union instance to store and transmit controller state
ControllerData b_controller_data{};

void read_controller_data();  // Function prototype

void setup() {
    Serial.begin(115200);  // Start USB serial communication

    // Set button pins as input with internal pull-ups
    pinMode(PIN_BUTTON_SELECT, INPUT_PULLUP);
    pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);
    pinMode(PIN_BUTTON_UP,    INPUT_PULLUP);
    pinMode(PIN_BUTTON_DOWN,  INPUT_PULLUP);
    pinMode(PIN_BUTTON_LEFT,  INPUT_PULLUP);

    // Set joystick axes as analog inputs (default behavior)
    pinMode(PIN_X_JOYSTICK, INPUT);
    pinMode(PIN_Y_JOYSTICK, INPUT);
}

void loop() {
    read_controller_data();  // Refresh struct with current inputs

    Serial.write(0xAA);  // SYNC_BYTE: start of packet for framing
    Serial.write(reinterpret_cast<uint8_t *>(&b_controller_data), sizeof(ControllerData));  // Send packed data

    delay(300);  // Throttle update rate to ~3 Hz
}

void read_controller_data() {
    // Read analog joystick positions (0–1023 range)
    b_controller_data.x_pos = analogRead(PIN_X_JOYSTICK);
    b_controller_data.y_pos = analogRead(PIN_Y_JOYSTICK);

    // Read button states: LOW = pressed due to pull-up config
    b_controller_data.control = digitalRead(PIN_BUTTON_SELECT) == LOW;
    b_controller_data.up      = digitalRead(PIN_BUTTON_UP)     == LOW;
    b_controller_data.down    = digitalRead(PIN_BUTTON_DOWN)   == LOW;
    b_controller_data.left    = digitalRead(PIN_BUTTON_LEFT)   == LOW;
    b_controller_data.right   = digitalRead(PIN_BUTTON_RIGHT)  == LOW;
}
