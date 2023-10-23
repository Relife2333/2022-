#ifndef mecanum_kun_h
#define mecanum_kun_h
#include "Arduino.h"
#include "motor_kun.h"
class mecanum_kun
{
private:
    uint8_t Head_L_pin1, Hear_R_pin1, Tail_L_pin1, Tail_R_pin1,
        Head_L_pin2, Head_R_pin2, Tail_L_pin2, Tail_R_pin2;

public:
    mecanum_kun(void);
    void begin(uint8_t Head_L_pin1, uint8_t Head_L_pin2, uint8_t Head_R_pin1, uint8_t Head_R_pin2, uint8_t Tail_L_pin1,
               uint8_t Tail_L_pin2, uint8_t Tail_R_pin1, uint8_t Tail_R_pin2);
    void go_x(int speed);
    void go_y(int speed);
    void go_circle(int speed);
    void go(uint8_t Head_L_speed, uint8_t Head_R_speed, uint8_t Tail_L_speed, uint8_t Tail_R_speed);
    void go_angle(int speed, int speed_L,int speed_R);
    // void test(int speed);
};

#endif
