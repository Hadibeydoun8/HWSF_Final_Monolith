
///////////////////////////////////////////////////////////////////////////////////
//Robot_Dive.INO
//Written By: Ricardo Tapia Vargas
//
//Team: Monsters Inc.
//
///////////////////////////////////////////////////////////////////////////////////


#include "Romi_Motor_Power.h"
/* Defines pin configuration of robot */
#include "RSLK_Pins.h"
#include "SimpleRSLK.h"
#include <WiskerData.h>

Romi_Motor_Power left_motor;
Romi_Motor_Power right_motor;
int motorSpeed = 0;
int badDriver = 0;

WiskerData b_wisker_data{};

void read_wisker_data();

void setup() {
  Serial.begin(115200);

  left_motor.begin(MOTOR_L_SLP_PIN,
           MOTOR_L_DIR_PIN,
           MOTOR_L_PWM_PIN);

  right_motor.begin(MOTOR_R_SLP_PIN,
            MOTOR_R_DIR_PIN,
            MOTOR_R_PWM_PIN);

  pinMode(BP_SW_PIN_0,INPUT_PULLUP);
  pinMode(BP_SW_PIN_1,INPUT_PULLUP);
  pinMode(BP_SW_PIN_2,INPUT_PULLUP);
  pinMode(BP_SW_PIN_3,INPUT_PULLUP);
  pinMode(BP_SW_PIN_4,INPUT_PULLUP);
  pinMode(BP_SW_PIN_5,INPUT_PULLUP);

  /* Left button on Launchpad */
  pinMode(LP_LEFT_BTN, INPUT_PULLUP);
  /* Red led in rgb led */
  pinMode(RED_LED,OUTPUT);
  Serial.println("Wilbur reporting for duty!");
  Serial.println("Use WASD to move and H to Halt!");

   left_motor.setSpeed(0);
   right_motor.setSpeed(0);

 
}

void loop() {
    left_motor.enableMotor();
    right_motor.enableMotor();

    read_wisker_data();

    Serial.write(reinterpret_cast<uint8_t*>(&b_wisker_data), sizeof(WiskerData));

    // print the bump switch data
    // Serial.print("Wisker 1: ");
    // Serial.print(b_wisker_data.w1);
    // Serial.print(" Wisker 2: ");
    // Serial.print(b_wisker_data.w2);
    // Serial.print(" Wisker 3: ");
    // Serial.print(b_wisker_data.w3);
    // Serial.print(" Wisker 4: ");
    // Serial.print(b_wisker_data.w4);
    // Serial.print(" Wisker 5: ");
    // Serial.print(b_wisker_data.w5);
    // Serial.print(" Wisker 6: ");
    // Serial.print(b_wisker_data.w6);


    delay(300); // Update ~3 times per second
}

void read_wisker_data() {
    b_wisker_data.w1 = digitalRead(BP_SW_PIN_0);
    b_wisker_data.w2 = digitalRead(BP_SW_PIN_1);
    b_wisker_data.w3 = digitalRead(BP_SW_PIN_2);
    b_wisker_data.w4 = digitalRead(BP_SW_PIN_3);
    b_wisker_data.w5 = digitalRead(BP_SW_PIN_4);
    b_wisker_data.w6 = digitalRead(BP_SW_PIN_5);
}
