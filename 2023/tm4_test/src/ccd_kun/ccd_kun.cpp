#include "ccd_kun.h"
ccd_kun::ccd_kun() : exp_time(10000)
{
}
void ccd_kun::begin(uint8_t si_pin_temp, uint8_t clk_pin_temp, uint8_t ao_pin_temp)
{
    si_pin = si_pin_temp;
    clk_pin = clk_pin_temp;
    ao_pin = ao_pin_temp;
    pinMode(si_pin, OUTPUT);
    pinMode(clk_pin, OUTPUT);
    pinMode(ao_pin, INPUT);
    // analogReadResolution(12);
    // analogSetAttenuation(ADC_11db);
}
void ccd_kun::set_exp_time(uint32_t exp_time_temp)
{
    exp_time = exp_time_temp;
}
void ccd_kun::read_loop(uint8_t *pixel)
{
    uint8_t i = 0;
    digitalWrite(si_pin, HIGH);  // SI拉高电平
    digitalWrite(clk_pin, HIGH); // 时钟高电平
    digitalWrite(si_pin, LOW);   // SI低电平
    digitalWrite(clk_pin, LOW);  // 时钟低电平
    for (i = 0; i < 128; i++)
    {
        digitalWrite(clk_pin, HIGH);
        digitalWrite(clk_pin, LOW);
    }                            // 从这里结束曝光
    delayMicroseconds(exp_time); // 曝光时间
    digitalWrite(si_pin, HIGH);
    digitalWrite(clk_pin, HIGH);
    digitalWrite(si_pin, LOW);
    digitalWrite(clk_pin, LOW);
    for (i = 0; i < 128; i++)
    {
        digitalWrite(clk_pin, HIGH);
        pixel[i] = (float)analogRead(ao_pin) / 4096 * 255;
        digitalWrite(clk_pin, LOW);
    }
    digitalWrite(clk_pin, HIGH);
    digitalWrite(clk_pin, LOW);
}
void ccd_kun::read_timer(uint8_t *pixel)
{
    uint8_t i = 0;
    digitalWrite(si_pin, HIGH);
    digitalWrite(clk_pin, HIGH);
    digitalWrite(si_pin, LOW);
    digitalWrite(clk_pin, LOW);
    for (i = 0; i < 128; i++)
    {
        digitalWrite(clk_pin, HIGH);
        pixel[i] = (float)analogRead(ao_pin) / 4096 * 255;
        digitalWrite(clk_pin, LOW);
    }
    digitalWrite(clk_pin, HIGH);
    digitalWrite(clk_pin, LOW);
}
