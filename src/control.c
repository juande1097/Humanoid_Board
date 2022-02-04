#include "control.h"



extern as5600_sensor as5600;
control_data control;
uint8_t Data_Read_U2[5] = {0};
static PID_data PID_actual_data;

void UART2_callback(uintptr_t context)
{
    control.ref = (((uint16_t)Data_Read_U2[0]) << 8) + Data_Read_U2[1];
    PID_actual_data.prev_error = 0;
    PID_actual_data.deriv_error = 0;
    PID_actual_data.integral_error = 0;
    UART2_Read(&Data_Read_U2[0],5);
}

void Control_initialize(void)
{
    control.duty_cycle = 0.0;
    control.position = &as5600.position;
    control.old_position = &as5600.old_position;
    control.speed = 0.0;
    control.acceleration = 0.0;
    control.ref = 300;
    control.period = 0.05;
    
    UART2_ReadCallbackRegister(&UART2_callback,0);
    
    UART2_Read(&Data_Read_U2[0],5);
}

void Control_PID(float kp, float ki, float kd)
{
    


    PID_actual_data.prev_error = PID_actual_data.error;
    PID_actual_data.error = control.ref - *control.position;
    PID_actual_data.integral_error += PID_actual_data.error*control.period;
    PID_actual_data.deriv_error = (PID_actual_data.error-PID_actual_data.prev_error)/control.period;

    PID_actual_data.pwm_output = kp*PID_actual_data.error + ki*PID_actual_data.integral_error + kd*PID_actual_data.deriv_error;
    if (PID_actual_data.pwm_output > 100.0)
    {
        PID_actual_data.pwm_output = 100.0;
    }
    if (PID_actual_data.pwm_output < -100.0)
    {
        PID_actual_data.pwm_output = -100;
    }
    Control_UpdateDirection(PID_actual_data.pwm_output);
    //Control_SetDutyPeriod(pwm_output);
    Control_SetDutyPeriod(PID_actual_data.pwm_output);
    
    Control_SendData();
    
   
    
}
void Control_SuperTwisting(void)
{
    
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

void Control_SendData(void)
{
    uint8_t uart_sent_data[15] = {0};
    uart_sent_data[0]= (uint8_t)(((uint16_t)(*control.position) & 0xFF00) >> 8);
    uart_sent_data[1]= (uint16_t)(*control.position) & 0x00FF;
    uart_sent_data[2]= (uint8_t)(((float)*control.position - (uint16_t)*control.position)*100);
    uart_sent_data[3]= (uint8_t)(((uint16_t)(control.ref) & 0xFF00) > 8);
    uart_sent_data[4]= (uint16_t)(control.ref) & 0x00FF;
    uart_sent_data[5]= (uint8_t)(((float)control.ref - (uint16_t)control.ref)*100);
    uart_sent_data[6]= (int)PID_actual_data.error >> 24;
    uart_sent_data[7]= (int)PID_actual_data.error >> 16;
    uart_sent_data[8]= (int)PID_actual_data.error >> 8;
    uart_sent_data[9]= (int)PID_actual_data.error;
    uart_sent_data[10]= (uint8_t)(((float)PID_actual_data.error - (int32_t)PID_actual_data.error)*100);
    uart_sent_data[11]= (uint8_t)PID_actual_data.pwm_output;

    UART2_Write(&uart_sent_data[0],15);
}