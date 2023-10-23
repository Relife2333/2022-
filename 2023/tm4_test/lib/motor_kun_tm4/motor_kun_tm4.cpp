#include "motor_kun_tm4.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
motor_kun::motor_kun(void)
{
}
void motor_kun::begin_drv8833(uint8_t pin_1_temp, uint8_t pin_2_temp, uint8_t pin_1_channel_temp, uint8_t pin_2_channel_temp, uint8_t drv8833_mode_temp)
{


	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);//使能Timer0

    //将PF1设置为CCP功能引脚，用于Timer0
	GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PF1_T0CCP1 );

	//设置Timer0的TimerB位PWM功能
	TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR |TIMER_CFG_B_PWM);

	TimerControlLevel(TIMER0_BASE, TIMER_B, false);

	TimerLoadSet(TIMER0_BASE,  TIMER_B,  10000);//设置初值为10000
	TimerMatchSet(TIMER0_BASE,  TIMER_B, 9900);//设置匹配值
	TimerEnable(TIMER0_BASE,  TIMER_B);//使能Timer0的TimerB
}
void motor_kun::motor_drv8833_run(int speed)
{

}
void motor_kun::set_freq(uint32_t freq_temp)
{

}

void motor_kun::set_bit_num(uint32_t bit_num_temp)
{

}
