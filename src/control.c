#include "control.h"



extern as5600_sensor as5600;
control_data control;

void Control_initialize(void)
{
    control.duty_cycle = 0.0;
    control.position = &as5600.position;
    control.old_position = &as5600.old_position;
    control.speed = 0.0;
    control.acceleration = 0.0;
    control.ref = 220;
    control.period = 0.05;
    
    
}

void Control_PID(float kp, float ki, float kd)
{
    uint8_t uart_sent_data[15] = {0};
    volatile int dummy = 0;

    static float error;
    float prev_error;
    float deriv_error;
    static float integral_error;
    volatile float pwm_output;

    prev_error = error;
    error = control.ref - *control.position;
    integral_error += error*control.period;
    deriv_error = (error-prev_error)/control.period;

    pwm_output = kp*error + ki*integral_error + kd*deriv_error;
    if (pwm_output > 100.0)
    {
        pwm_output = 100.0;
    }
    if (pwm_output < -100.0)
    {
        pwm_output = -100;
    }
    Control_UpdateDirection(pwm_output);
    //Control_SetDutyPeriod(pwm_output);
    Control_SetDutyPeriod(pwm_output);
    
    uart_sent_data[0]= (uint8_t)(((uint16_t)(*control.position) & 0xFF00) >> 8);
    uart_sent_data[1]= (uint16_t)(*control.position) & 0x00FF;
    uart_sent_data[2]= (uint8_t)(((float)*control.position - (uint16_t)*control.position)*100);
    uart_sent_data[3]= (uint8_t)(((uint16_t)(control.ref) & 0xFF00) > 8);
    uart_sent_data[4]= (uint16_t)(control.ref) & 0x00FF;
    uart_sent_data[5]= (uint8_t)(((float)control.ref - (uint16_t)control.ref)*100);
    uart_sent_data[6]= (int)error >> 24;
    uart_sent_data[7]= (int)error >> 16;;
    uart_sent_data[8]= (int)error >> 8;;
    uart_sent_data[9]= (int)error;;
    uart_sent_data[10]= (uint8_t)(((float)error - (int32_t)error)*100);
    uart_sent_data[11]= (uint8_t)pwm_output;

    dummy = (int)error;
    UART2_Write(&uart_sent_data[0],15);
    
    uart_sent_data[10]= dummy;
    
    
    
    

    
}
void Control_UpdateSpeedAcceleration(void)
{

}

void Control_UpdateDirection(float pwm_duty)
{
    //Direction change
    if (pwm_duty > 0.0) //clockwise
    {
        Motor_dir_A_Clear();
        Motor_dir_B_Set();
    }
    if (pwm_duty < 0.0) ////counterclockwise
    {
       
        Motor_dir_A_Set();
        Motor_dir_B_Clear();
    }
}

void Control_SetDutyPeriod(float pwm_duty)
{
    uint16_t duty_period = 0;
    duty_period = abs(pwm_duty)*(DUTY_MAX_PERIOD-1)/100;
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_1,duty_period);
}