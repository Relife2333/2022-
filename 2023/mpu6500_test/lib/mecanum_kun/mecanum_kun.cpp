#include "mecanum_kun.h"
motor_kun Head_L_wheel;
motor_kun Head_R_wheel;
motor_kun Tail_L_wheel;
motor_kun Tail_R_wheel;
mecanum_kun ::mecanum_kun(void)
{
}
void mecanum_kun ::begin(uint8_t Head_L_pin1, uint8_t Head_L_pin2, uint8_t Head_R_pin1, uint8_t Head_R_pin2, uint8_t Tail_L_pin1,
                         uint8_t Tail_L_pin2, uint8_t Tail_R_pin1, uint8_t Tail_R_pin2)

{
    this->Head_L_pin1 = Head_L_pin1;
    this->Head_L_pin2 = Head_L_pin2;
    this->Hear_R_pin1 = Hear_R_pin1;
    this->Head_R_pin2 = Head_R_pin2;
    this->Tail_L_pin1 = Tail_L_pin1;
    this->Tail_L_pin2 = Tail_L_pin2;
    this->Tail_R_pin1 = Tail_R_pin1;
    this->Tail_R_pin2 = Tail_R_pin2;
    Head_L_wheel.begin_drv8833(Head_L_pin1, Head_L_pin2, 0, 1);
    Head_R_wheel.begin_drv8833(Head_R_pin1, Head_R_pin2, 2, 3);
    Tail_L_wheel.begin_drv8833(Tail_L_pin1, Tail_L_pin2, 4, 5);
    Tail_R_wheel.begin_drv8833(Tail_R_pin1, Tail_R_pin2, 6, 7);
}
void mecanum_kun ::go_y(int speed)
{
    Head_L_wheel.motor_drv8833_run(speed);
    Head_R_wheel.motor_drv8833_run(speed);
    Tail_L_wheel.motor_drv8833_run(speed);
    Tail_R_wheel.motor_drv8833_run(speed);
}
void mecanum_kun ::go_x(int speed)
{
    Head_L_wheel.motor_drv8833_run(-speed);
    Head_R_wheel.motor_drv8833_run(speed);
    Tail_L_wheel.motor_drv8833_run(speed);
    Tail_R_wheel.motor_drv8833_run(-speed);
}
void mecanum_kun ::go_circle(int speed)
{
    Head_L_wheel.motor_drv8833_run(-speed);
    Head_R_wheel.motor_drv8833_run(speed);
    Tail_L_wheel.motor_drv8833_run(-speed);
    Tail_R_wheel.motor_drv8833_run(speed);
}
// void mecanum_kun ::test(int speed)
// {
//     Head_L_wheel.motor_drv8833_run(speed);
//     Head_R_wheel.motor_drv8833_run(speed);
//     Tail_L_wheel.motor_drv8833_run(speed);
// }
void mecanum_kun ::go(uint8_t Head_L_speed, uint8_t Head_R_speed, uint8_t Tail_L_speed, uint8_t Tail_R_speed)
{
    Head_L_wheel.motor_drv8833_run(Head_L_speed);
    Head_R_wheel.motor_drv8833_run(Head_R_speed);
    Tail_L_wheel.motor_drv8833_run(Tail_L_speed);
    Tail_R_wheel.motor_drv8833_run(Tail_R_speed);
}
void mecanum_kun ::go_angle(int speed, int speed_L,int speed_R)
{
    // float angle_temp = angle * 3.1415926 / 180;
    // float sin_angle = sin(angle_temp);
    // float cos_angle = cos(angle_temp);
    // float Head_L_speed = speed * sin_angle + speed * cos_angle;
    // float Head_R_speed = speed * sin_angle - speed * cos_angle;
    // float Tail_L_speed = speed * sin_angle - speed * cos_angle;
    // float Tail_R_speed = speed * sin_angle + speed * cos_angle;

    Head_L_wheel.motor_drv8833_run(speed+speed_L);
    Head_R_wheel.motor_drv8833_run(speed+speed_R);
    Tail_L_wheel.motor_drv8833_run(speed+speed_L);
    Tail_R_wheel.motor_drv8833_run(speed+speed_R);
}
