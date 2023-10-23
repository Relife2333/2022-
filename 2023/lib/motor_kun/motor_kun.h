#ifndef motor_kun_h
#define motor_kun_h
#include "Arduino.h"
class motor_kun
{
private:
    uint8_t
    pin_1
    ,pin_2
    ,direction_pin_1
    ,direction_pin_2
    ,pwm_pin
    ,channel
    ,drv8833_mode
    ,pin_1_channel
    ,pin_2_channel
    ,bit_num;
    uint32_t freq;
    int pwm_max;
public:
    motor_kun(void);
    void begin_drv8833(uint8_t pin_1_temp,uint8_t pin_2_temp,uint8_t pin_1_channel_temp,uint8_t pin_2_channel_temp,uint8_t drv8833_mode_temp=1);//mode：1为快衰减，0为慢衰减
    void begin_tb6612(uint8_t direction_pin_1_temp,uint8_t direction_pin_2_temp,uint8_t pwm_pin_temp,uint8_t channel_temp);
    void motor_drv8833_run(int speed);
    void motor_tb6612_run(int speed);
    void set_freq(uint32_t freq_temp);
    void set_bit_num(uint32_t bit_num_temp);
};
#endif
