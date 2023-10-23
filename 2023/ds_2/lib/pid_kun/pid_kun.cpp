#include "pid_kun.h"
/*
	float target_val;       //目标值
	float err;              //偏差值
	float err_last;         //上一个偏差值
    float err_last_last;    //上上一个偏差值
	float Kp,Ki,Kd;         //比例、积分、微分系数
	float integral;         //积分值
    float integral_max;     //积分最大值
    float integral_min;     //积分最小值
	float output_val;       //输出值
    float output_val_max;   //输出最大值
    float output_val_min;   //输出最小值
*/
pid_kun::pid_kun()
{

}

// pid_kun::pid_kun(float Kp_temp,float Ki_temp,float Kd_temp,
// float output_val_max_temp,float output_val_min_temp,
// float integral_max_temp,float integral_min_temp):
// Kp(Kp_temp),Ki(Ki_temp),Kd(Kd_temp),
// integral_max(integral_max_temp),integral_min(integral_min_temp),
// output_val_max(output_val_max_temp),output_val_min(output_val_min_temp),
// err(0),err_last(0),err_last_last(0),integral_limit_min(-30),integral_limit_max(30)
// {

// }
void pid_kun::set_integral_limit(float integral_limit_min,float integral_limit_max)
{
    integral_limit_min=integral_limit_min;
    integral_limit_max=integral_limit_max;
}
float pid_kun::PID_position(float current_val)
{
    err=current_val-target_val;

    integral+=Ki*err;

    if(integral>integral_max)
    {
        integral=integral_max;
    }else if(integral<integral_min)
    {
        integral=integral_min;
    }
    if(err<=(integral_limit_min-target_val)||err>=(integral_limit_max+target_val))
    {
        integral=0;
    }
    output_val=
    Kp*err+
    integral+
    Kd*(err-err_last);

    if(output_val>output_val_max)
    {
        output_val=output_val_max;
    }else if(output_val<output_val_min)
    {
        output_val=output_val_min;
    }

    err_last=err;
    return output_val;
}
void pid_kun::set_pid(float Kp_temp,float Ki_temp,float Kd_temp)
{
    Kp=Kp_temp;
    Ki=Ki_temp;
    Kd=Kd_temp;
}
void pid_kun::set_integral_range(float integral_max_temp,float integral_min_temp)
{
    integral_max=integral_max_temp;
    integral_min=integral_min_temp;
}
void pid_kun::set_output_val_range(float output_val_max_temp,float output_val_min_temp)
{
    output_val_max=output_val_max_temp;
    output_val_min=output_val_min_temp;
}
void pid_kun::set_target_val(float target_val_temp)
{
    target_val=target_val_temp;
}
float pid_kun::PID_increment(float current_val)
{
    err=current_val;
    output_val=Kp*(err-err_last)+Ki*err+Kd*(err_last_last+err-err_last-err_last);
    err_last_last=err_last;
    err_last=err;
    if(output_val>output_val_max)
    {
        output_val=output_val_max;
    }else if(output_val<output_val_min)
    {
        output_val=output_val_min;
    }
    return output_val;
}
float pid_kun::PID_position(float current_val,float target_val_temp)
{
    target_val=target_val_temp;
    return PID_position(current_val);
}
float pid_kun::PID_increment(float current_val,float target_val_temp)
{
    target_val=target_val_temp;
    return PID_increment(current_val);
}
