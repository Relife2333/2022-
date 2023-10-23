#ifndef ccd_kun_h
#define ccd_kun_h
#include "Arduino.h"
class ccd_kun
{
    private:
        uint32_t exp_time;
        uint8_t si_pin,ao_pin,clk_pin;
        // uint8_t pixel[128];
    public:
    ccd_kun();
    void begin(uint8_t si_pin_temp,uint8_t clk_pin_temp,uint8_t ao_pin_temp);
    void set_exp_time(uint32_t exp_time_temp);
    void read_loop(uint8_t *pixel);
    void read_timer(uint8_t *pixel);
};
#endif
