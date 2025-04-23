#pragma once
#include <stdint.h>

// Sync byte used to frame serial communication packets
#define SYNC_BYTE 0xAA

// Forward declarations (not strictly necessary here, but helpful if splitting headers)
struct __attribute__((__packed__)) WiskerData;
struct __attribute__((__packed__)) ControllerData;
struct __attribute__((__packed__)) RobotControl;

union __attribute__((__packed__)) WiskerDataUnion;
union __attribute__((__packed__)) ControllerDataUnion;
union __attribute__((__packed__)) RobotControlUnion;

//---------------------------------------------
// Struct: WiskerData
// Purpose: Holds the state of each whisker sensor (bool = pressed or not)
//          Plus a total whisker_count used to trigger autonomous behavior
//---------------------------------------------
struct __attribute__((packed)) WiskerData {
  bool w1;  // Individual whisker sensor states
  bool w2;
  bool w3;
  bool w4;
  bool w5;
  bool w6;
  int16_t wisker_count; // Counter of the number of wiskers pressed since the last reset
};

//---------------------------------------------
// Struct: ControllerData
// Purpose: Transmits all inputs from the controller:
//          joystick X/Y analog values and D-pad + button booleans
//---------------------------------------------
struct __attribute__((packed)) ControllerData {
  int16_t x_pos;   // Raw ADC X-axis value (0–1023)
  int16_t y_pos;   // Raw ADC Y-axis value (0–1023)

  bool control;    // Select / center / action button
  bool up;         // D-pad directions
  bool down;
  bool left;
  bool right;
};

//---------------------------------------------
// Struct: RobotControl
// Purpose: Transmits motion commands from ROS 2 to the robot.
//          Includes per-wheel speeds, directions, and whether to enter dance mode.
//---------------------------------------------
struct __attribute__((packed)) RobotControl {
  int16_t left_motor_speed;   // Speed value: 0–100
  int16_t right_motor_speed;

  bool left_motor_dir;        // true = forward, false = reverse
  bool right_motor_dir;

  bool dance_mode;            // true = engage dance mode behavior
  bool rouge_mode;            // true = engage rouge mode behavior
};

//---------------------------------------------
// Union: ControllerDataUnion
// Purpose: Allows the same memory block to be accessed either
//          as a struct (for field access) or byte array (for serial transmission)
//---------------------------------------------
union __attribute__ ((packed)) ControllerDataUnion {
  ControllerData controller_data;
  uint8_t data[sizeof(ControllerData)];
};

// Same principle applied to control and sensor data:
union __attribute__((packed)) RobotControlUnion {
  RobotControl robot_control;
  uint8_t data[sizeof(RobotControl)];
};

union __attribute__((packed)) WiskerDataUnion {
  WiskerData wisker_data;
  uint8_t data[sizeof(WiskerData)];
};

//---------------------------------------------
// Compile-time checks
// Purpose: These static_asserts ensure struct sizes match expectations exactly.
//          This guards against compiler-specific padding changes or field reordering.
//---------------------------------------------
static_assert(sizeof(WiskerData) == 8, "WiskerData size mismatch");  // 6 bools (1 byte total) + 2-byte int16_t + packing
static_assert(sizeof(WiskerDataUnion) == 8, "WiskerDataUnion size mismatch");

static_assert(sizeof(ControllerData) == 9, "Controller Data size mismatch");
// 2 int16_t (4 bytes) + 5 bools (1 byte if packed tightly) = 9 bytes
static_assert(sizeof(ControllerDataUnion) == 9, "ControllerDataUnion size mismatch");

static_assert(sizeof(RobotControlUnion) == 8, "RobotControl size mismatch");
// 2 int16_t (4 bytes) + 3 bools (assuming packed to 1 byte) = 7 bytes
