#include <Arduino.h>
#include <RobotIO.h>

#define PIN_BUTTON_SELECT 2

#define PIN_BUTTON_RIGHT 3
#define PIN_BUTTON_UP 4
#define PIN_BUTTON_DOWN 5
#define PIN_BUTTON_LEFT 6


#define PIN_X_JOYSTICK A0
#define PIN_Y_JOYSTICK A1


ControllerData b_controller_data{};

void read_controller_data();

void setup() {
    Serial.begin(115200);

    pinMode(PIN_BUTTON_SELECT, INPUT_PULLUP);
    pinMode(PIN_BUTTON_RIGHT,INPUT_PULLUP);
    pinMode(PIN_BUTTON_UP,INPUT_PULLUP);
    pinMode(PIN_BUTTON_DOWN,INPUT_PULLUP);
    pinMode(PIN_BUTTON_LEFT,INPUT_PULLUP);

    pinMode(PIN_X_JOYSTICK,INPUT);
    pinMode(PIN_Y_JOYSTICK,INPUT);
}

void loop() {
    read_controller_data();

    Serial.write(0xAA); // sync byte
    Serial.write(reinterpret_cast<uint8_t *>(&b_controller_data), sizeof(ControllerData));

    delay(300); // Update ~3 times per second
}

void read_controller_data() {
    b_controller_data.x_pos = analogRead(PIN_X_JOYSTICK);
    b_controller_data.y_pos = analogRead(PIN_Y_JOYSTICK);
    b_controller_data.control = digitalRead(PIN_BUTTON_SELECT) == LOW;
    b_controller_data.up = digitalRead(PIN_BUTTON_UP) == LOW;
    b_controller_data.down = digitalRead(PIN_BUTTON_DOWN) == LOW;
    b_controller_data.left = digitalRead(PIN_BUTTON_LEFT) == LOW;
    b_controller_data.right = digitalRead(PIN_BUTTON_RIGHT) == LOW;

}
