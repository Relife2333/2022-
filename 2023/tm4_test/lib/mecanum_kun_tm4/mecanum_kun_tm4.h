#ifndef mecanum_kun_tm4_h
#define mecanum_kun_tm4_h
#include "Arduino.h"

class mecanum_kun_tm4
{
private:


public:
    // mecanum_kun_tm4();
    void begin();
    void go_x(int speed);
    void go_y(int speed);
    void go_circle(int speed);
    void go(int speed_HL,int speed_HR,int speed_TL,int speed_TR);
    void go_angle_1(int speed,int speed_L,int speed_R);
    void go_angle_2(int speed,int speed_L,int speed_R);

    // void test(int speed);
};

#endif
