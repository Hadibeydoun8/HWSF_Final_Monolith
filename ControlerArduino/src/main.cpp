#include <Arduino.h>
// #include <WiskerData.h>

#define PIN_BUTTON_SELECT 2

#define PIN_BUTTON_RIGHT 3
#define PIN_BUTTON_UP 4
#define PIN_BUTTON_DOWN 5
#define PIN_BUTTON_LEFT 6


#define PIN_X_JOYSTICK A0
#define PIN_Y_JOYSTICK A1


struct __attribute__ ((packed)) ControllerData {
    int16_t x_pos;
    int16_t y_pos;

    bool control;

    bool up;
    bool down;
    bool left;
    bool right;
};

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

    // Serial.print("x_pos: ");
    // Serial.print(b_controller_data.x_pos);
    // Serial.print(", y_pos: ");
    // Serial.print(b_controller_data.y_pos);
    //
    // Serial.print(", control: ");
    // Serial.print(b_controller_data.control);
    //
    // Serial.print(", up: ");
    // Serial.print(b_controller_data.up);
    // Serial.print(", down: ");
    // Serial.print(b_controller_data.down);
    // Serial.print(", left: ");
    // Serial.print(b_controller_data.left);
    // Serial.print(", right: ");
    // Serial.println(b_controller_data.right);
    // // Serial.write(reinterpret_cast<uint8_t *>(&b_wisker_data), sizeof(WiskerData));
    //
    //

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
