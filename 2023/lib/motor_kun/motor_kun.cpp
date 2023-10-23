#include "motor_kun.h"
motor_kun::motor_kun(void) : bit_num(10),
                             freq(1000),
                             pwm_max(1023)
{
}
void motor_kun::begin_drv8833(uint8_t pin_1_temp, uint8_t pin_2_temp, uint8_t pin_1_channel_temp, uint8_t pin_2_channel_temp, uint8_t drv8833_mode_temp)
{
    drv8833_mode = drv8833_mode_temp;
    pin_1 = pin_1_temp;
    pin_2 = pin_2_temp;
    pin_1_channel = pin_1_channel_temp;
    pin_2_channel = pin_2_channel_temp;
    ledcSetup(pin_1_channel, freq, bit_num);
    ledcAttachPin(pin_1, pin_1_channel);
    ledcWrite(pin_1_channel, 0);
    ledcSetup(pin_2_channel, freq, bit_num);
    ledcAttachPin(pin_2, pin_2_channel);
    ledcWrite(pin_2_channel, 0);
}
void motor_kun::motor_drv8833_run(int speed)
{
    if (speed>pwm_max)
    {
        speed=pwm_max;
    }else if(speed<-pwm_max){
        speed=-pwm_max;
    }
    if (drv8833_mode)
    {
        if(speed>=0){
            ledcWrite(pin_1_channel, speed);
            ledcWrite(pin_2_channel, 0);
        }else{
            speed=-speed;
            ledcWrite(pin_1_channel, 0);
            ledcWrite(pin_2_channel, speed);
        }
    }else{
        if(speed>=0){
            ledcWrite(pin_1_channel, pwm_max);
            ledcWrite(pin_2_channel, pwm_max-speed);
        }else{
            ledcWrite(pin_1_channel, pwm_max+speed);
            ledcWrite(pin_2_channel, pwm_max);
        }
    }
}
void motor_kun::begin_tb6612(uint8_t direction_pin_1_temp, uint8_t direction_pin_2_temp, uint8_t pwm_pin_temp, uint8_t channel_temp)
{
    direction_pin_1 = direction_pin_1_temp;
    direction_pin_2 = direction_pin_2_temp;
    pwm_pin = pwm_pin_temp;
    channel = channel_temp;
    pinMode(direction_pin_1, OUTPUT);
    digitalWrite(direction_pin_1, LOW);
    pinMode(direction_pin_2, OUTPUT);
    digitalWrite(direction_pin_2, LOW);
    ledcSetup(channel, freq, bit_num);
    ledcAttachPin(pwm_pin, channel);
    ledcWrite(channel, 0);
}
void motor_kun::motor_tb6612_run(int speed)
{
    if (speed >= 0)
    {
        digitalWrite(direction_pin_1, HIGH);
        digitalWrite(direction_pin_2, LOW);
        ledcWrite(channel, speed);
    }
    else if (speed < 0)
    {
        ledcWrite(channel, -speed);
        digitalWrite(direction_pin_1, LOW);
        digitalWrite(direction_pin_2, HIGH);
    }
}
void motor_kun::set_freq(uint32_t freq_temp)
{
    freq = freq_temp;
    ledcSetup(channel, freq, bit_num);
}

void motor_kun::set_bit_num(uint32_t bit_num_temp)
{
    uint8_t i;
    int temp = 1;
    for (i = 0; i < bit_num_temp; i++)
    {
        temp *= 2;
    }
    pwm_max = temp - 1;
    bit_num = bit_num_temp;
    ledcSetup(channel, freq, bit_num);
}
