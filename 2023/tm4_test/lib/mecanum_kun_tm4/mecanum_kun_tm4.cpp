#define PART_TM4C123GH6PM 1
#include "mecanum_kun_tm4.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
void PWM_Init()
{
    // 因为设置了时钟总线是40MHz，所以在这里分一下频设置为4分频，那么PWM时钟就是10MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
    // 使能时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // 使能PWM模块1时钟
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // 使能GPIOF时钟
    // 分配pwm信号
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);
    // 使能引脚复用
    GPIOPinConfigure(GPIO_PB6_M0PWM0); // PF2->PWM模块1信号6
    GPIOPinConfigure(GPIO_PB7_M0PWM1); // PF3->PWM模块1信号7
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);
    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    // 配置PWM发生器
    // 模块1->发生器3->上下计数，不同步
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // 配置PWM周期，此时PWM输出频率为10M/2k = 50000Hz。计数器为16位，故第三个参数不能超过65535
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 2000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 2000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 2000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 2000);
    // 配置PWM占空比为1%
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 0 - 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 0 - 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) * 0 - 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) * 0 - 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2) * 0 - 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2) * 0 - 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) * 0 - 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) * 0 - 1);
    // 使能PWM模块1输出
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
    // 使能PWM发生器
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}
void PWM_Set_Duty(uint32_t ui32Base, uint32_t ui32PWMOut, float duty)
{
    uint32_t ui32Gen;
    uint32_t ui32OutBits;

    switch (ui32PWMOut)
    {
    case PWM_OUT_0:
        ui32Gen = PWM_GEN_0, ui32OutBits = PWM_OUT_0_BIT;
        break;
    case PWM_OUT_1:
        ui32Gen = PWM_GEN_0, ui32OutBits = PWM_OUT_1_BIT;
        break;
    case PWM_OUT_2:
        ui32Gen = PWM_GEN_1, ui32OutBits = PWM_OUT_2_BIT;
        break;
    case PWM_OUT_3:
        ui32Gen = PWM_GEN_1, ui32OutBits = PWM_OUT_3_BIT;
        break;
    case PWM_OUT_4:
        ui32Gen = PWM_GEN_2, ui32OutBits = PWM_OUT_4_BIT;
        break;
    case PWM_OUT_5:
        ui32Gen = PWM_GEN_2, ui32OutBits = PWM_OUT_5_BIT;
        break;
    case PWM_OUT_6:
        ui32Gen = PWM_GEN_3, ui32OutBits = PWM_OUT_6_BIT;
        break;
    case PWM_OUT_7:
        ui32Gen = PWM_GEN_3, ui32OutBits = PWM_OUT_7_BIT;
        break;
    }
    // 配置占空比
    PWMPulseWidthSet(ui32Base, ui32PWMOut, PWMGenPeriodGet(ui32Base, ui32Gen) * duty - 1);
    PWMOutputState(ui32Base, ui32OutBits, true);
    // 使能发生器模块
    PWMGenEnable(ui32Base, ui32Gen);
}
void Head_L_wheel(int speed)
{
    if(speed>0)
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_0, (float)speed/1000);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_1, 0);
    }
    else if(speed<0)
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_0, 0);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_1, -(float)speed/1000);
    }
    else
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_0, 0);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_1, 0);
    }
}
void Head_R_wheel(int speed)
{
    if(speed>0)
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_2, (float)speed/1000);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_3, 0);
    }
    else if(speed<0)
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_2, 0);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_3, -(float)speed/1000);
    }
    else
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_2, 0);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_3, 0);
    }
}
void Tail_L_wheel(int speed)
{
    if(speed>0)
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_4, (float)speed/1000);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_5, 0);
    }
    else if(speed<0)
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_4, 0);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_5, -(float)speed/1000);
    }
    else
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_4, 0);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_5, 0);
    }
}
void Tail_R_wheel(int speed)
{
    if(speed>0)
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_6, (float)speed/1000);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_7, 0);
    }
    else if(speed<0)
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_6, 0);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_7, -(float)speed/1000);
    }
    else
    {
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_6, 0);
        PWM_Set_Duty(PWM0_BASE, PWM_OUT_7, 0);
    }
}
// mecanum_kun_tm4::mecanum_kun_tm4()
// {

// }
void mecanum_kun_tm4 ::begin()
{
    PWM_Init();
}
void mecanum_kun_tm4 ::go_y(int speed)
{
    Head_L_wheel(speed);
    Head_R_wheel(speed);
    Tail_L_wheel(speed);
    Tail_R_wheel(speed);
}
void mecanum_kun_tm4 ::go_x(int speed)
{
    Head_L_wheel(-speed);
    Head_R_wheel(speed);
    Tail_L_wheel(speed);
    Tail_R_wheel(-speed);
}
void mecanum_kun_tm4 ::go_circle(int speed)
{
    Head_L_wheel(-speed);
    Head_R_wheel(speed);
    Tail_L_wheel(-speed);
    Tail_R_wheel(speed);
}


void mecanum_kun_tm4 ::go(int speed_HL,int speed_HR,int speed_TL,int speed_TR)
{
    Head_L_wheel(speed_HL);
    Head_R_wheel(speed_HR);
    Tail_L_wheel(speed_TL);
    Tail_R_wheel(speed_TR);
}
void mecanum_kun_tm4 ::go_angle_1(int speed,int speed_L,int speed_R)
{
    Head_L_wheel(speed);
    Head_R_wheel(speed);
    Tail_L_wheel(speed_L);
    Tail_R_wheel(speed_R);
}
void mecanum_kun_tm4 ::go_angle_2(int speed,int speed_L,int speed_R)
{
    Head_L_wheel(speed+speed_L);
    Head_R_wheel(speed+speed_R);
    Tail_L_wheel(speed+speed_L);
    Tail_R_wheel(speed+speed_R);
}
