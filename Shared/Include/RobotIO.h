#pragma once
#include <stdint.h>

#define SYNC_BYTE 0xAA

struct __attribute__((__packed__)) WiskerData;
struct __attribute__((__packed__)) ControllerData;
struct __attribute__((__packed__)) RobotControl;

union __attribute__((__packed__)) WiskerDataUnion;
union __attribute__((__packed__)) ControllerDataUnion;
union __attribute__((__packed__)) RobotControlUnion;


struct __attribute__((packed)) WiskerData {
  bool w1;
  bool w2;
  bool w3;
  bool w4;
  bool w5;
  bool w6;
  int16_t wisker_count;
};

struct __attribute__((packed)) ControllerData {
  int16_t x_pos;
  int16_t y_pos;

  bool control;

  bool up;
  bool down;
  bool left;
  bool right;
};

struct __attribute__((packed)) RobotControl {
  int16_t left_motor_speed;
  int16_t right_motor_speed;

  bool left_motor_dir;
  bool right_motor_dir;

  bool dance_mode;
};

union __attribute__ ((packed)) ControllerDataUnion {
  ControllerData controller_data;
  uint8_t data[sizeof(ControllerData)];
};

union __attribute__((packed)) RobotControlUnion {
  RobotControl robot_control;
  uint8_t data[sizeof(RobotControl)];
};

union __attribute__((packed)) WiskerDataUnion {
  WiskerData wisker_data;
  uint8_t data[sizeof(WiskerData)];
};

static_assert(sizeof(WiskerData) == 8, "WiskerData size mismatch");
static_assert(sizeof(WiskerDataUnion) == 8, "WiskerData size mismatch");

static_assert(sizeof(ControllerData) == 9, "Controller Data size mismatch");

static_assert(sizeof(ControllerDataUnion) == 9, "RobotControl size mismatch");

static_assert(sizeof(RobotControlUnion) == 7, "RobotControl size mismatch");

