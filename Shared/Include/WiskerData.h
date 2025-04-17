#pragma once

struct __attribute__((packed)) WiskerData {
    bool w1;
    bool w2;
    bool w3;
    bool w4;
    bool w5;
    bool w6;
};


struct __attribute__ ((packed)) ControllerData {
    int16_t x_pos;
    int16_t y_pos;

    bool control;

    bool up;
    bool down;
    bool left;
    bool right;
};