#include "control.h"



extern as5600_sensor sensor_1;
extern as5600_sensor sensor_2;
extern as5600_sensor sensor_3;
extern as5048a_sensor sensor_4;
extern as5048a_sensor sensor_5;
extern as5048a_sensor sensor_6;

extern STA_data motor_control_1;
extern STA_data motor_control_2;
extern STA_data motor_control_3;
extern STA_data motor_control_4;
extern STA_data motor_control_5;
extern STA_data motor_control_6;
//control_data control;
uint8_t Data_Read_U2[12] = {0};
static uint8_t uart_sent_data[96] = {0};

//static PID_data PID_actual_data;
//static STA_data SMC_ST_data;



void UART2_callback(uintptr_t context)
{
    motor_control_1.ref = (((uint16_t)Data_Read_U2[0]) << 8) + Data_Read_U2[1];
    motor_control_2.ref = (((uint16_t)Data_Read_U2[2]) << 8) + Data_Read_U2[3];
    motor_control_3.ref = (((uint16_t)Data_Read_U2[4]) << 8) + Data_Read_U2[5];
    motor_control_4.ref = (((uint16_t)Data_Read_U2[6]) << 8) + Data_Read_U2[7];
    motor_control_5.ref = (((uint16_t)Data_Read_U2[8]) << 8) + Data_Read_U2[9];
    motor_control_6.ref = (((uint16_t)Data_Read_U2[10]) << 8) + Data_Read_U2[11];
    
    //PID_actual_data.prev_error = 0;
    //PID_actual_data.deriv_error = 0;
    //PID_actual_data.integral_error = 0;
    UART2_Read(&Data_Read_U2[0],12);
}

void Control_initialize(uint16_t reference,STA_data *SMC_ST_data ,as5600_sensor *sensor, uint8_t motor_number, float const_c1, float const_c2, float const_b)
{
    SMC_ST_data->ref = reference;
    SMC_ST_data->position = &sensor->position;
    SMC_ST_data->period = CONTROL_PERIOD;
    SMC_ST_data->error = 0;
    SMC_ST_data->prev_error = 0;
    SMC_ST_data->deriv_error = 0;
    SMC_ST_data->const_c1 = const_c1; //245 400    //el bueno para el 2 1500 0.7 2 con vibracion
    SMC_ST_data->const_c2 = const_c2; //0.95 0.7
    SMC_ST_data->const_b =  const_b; //0.3 2
    SMC_ST_data->sigma = 0;
    SMC_ST_data->omega = 0;
    SMC_ST_data->prev_omega = 0;
    SMC_ST_data->pwm_output = 0;
    SMC_ST_data->motor_number = motor_number;
    
    UART2_ReadCallbackRegister(&UART2_callback,0);
    
    UART2_Read(&Data_Read_U2[0],12);
}
void Control_initialize_As5048(uint16_t reference, STA_data *SMC_ST_data ,as5048a_sensor *sensor, uint8_t motor_number, float const_c1, float const_c2, float const_b)
{
    SMC_ST_data->ref = reference;
    SMC_ST_data->position = &sensor->position;
    SMC_ST_data->period = CONTROL_PERIOD;
    SMC_ST_data->error = 0;
    SMC_ST_data->prev_error = 0;
    SMC_ST_data->deriv_error = 0;
    SMC_ST_data->const_c1 = const_c1; //245 400    //el bueno para el 2 1500 0.7 2 con vibracion
    SMC_ST_data->const_c2 = const_c2; //0.95 0.7
    SMC_ST_data->const_b =  const_b; //0.3 2
    SMC_ST_data->sigma = 0;
    SMC_ST_data->omega = 0;
    SMC_ST_data->prev_omega = 0;
    SMC_ST_data->pwm_output = 0;
    SMC_ST_data->motor_number = motor_number;
    
    UART2_ReadCallbackRegister(&UART2_callback,0);
    
    //UART2_Read(&Data_Read_U2[0],6);
}

/*void Control_PID(float kp, float ki, float kd)
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
    
}*/
void Control_SuperTwisting(STA_data *SMC_ST_data)
{
    SMC_ST_data->prev_error = SMC_ST_data->error;
    SMC_ST_data->prev_omega = SMC_ST_data->omega;
    SMC_ST_data->error = SMC_ST_data->ref - *SMC_ST_data->position;   
    SMC_ST_data->deriv_error = (SMC_ST_data->error - SMC_ST_data->prev_error)/SMC_ST_data->period;
    SMC_ST_data->sigma = SMC_ST_data->deriv_error + SMC_ST_data->const_c1*SMC_ST_data->error;
    SMC_ST_data->omega = (SMC_ST_data->period)*(SMC_ST_data->const_b)*(Control_Sign(SMC_ST_data->sigma)) + SMC_ST_data->prev_omega;
    SMC_ST_data->pwm_output = (SMC_ST_data->const_c2)*(sqrt(abs(SMC_ST_data->sigma)))*(Control_Sign(SMC_ST_data->sigma)) + SMC_ST_data->omega;
    
    if (SMC_ST_data->pwm_output > 100.0)
    {
        SMC_ST_data->pwm_output = 100.0;
    }
    if (SMC_ST_data->pwm_output < -100.0)
    {
        SMC_ST_data->pwm_output = -100;
    }
    //Control_UpdateDirection(SMC_ST_data.pwm_output);
    //Control_SetDutyPeriod(pwm_output);
    Control_SetDutyPeriod_IBT_4(*SMC_ST_data);
    
    
}
void Control_UpdateSpeedAcceleration(void)
{

}


void Control_SetDutyPeriod(float pwm_duty)
{
    uint16_t duty_period = 0;
    duty_period = abs(pwm_duty)*(DUTY_MAX_PERIOD-1)/100;
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_1,duty_period);
}
void Control_SetDutyPeriod_IBT_4(STA_data SMC_ST_data)
{
    uint16_t duty_period = 0;
    switch (SMC_ST_data.motor_number)
    {
        case 1:
            if (SMC_ST_data.pwm_output > 0.0) //clockwise
            {
                duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                //MCPWM_ChannelPrimaryDutySet(MCPWM_CH_1,duty_period);
                //Motor_dir_A_Set();
                //Motor_dir_B_Clear();
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_1,0);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2,duty_period);
            }
            if (SMC_ST_data.pwm_output < 0.0) ////counterclockwise
            {
                duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                //MCPWM_ChannelPrimaryDutySet(MCPWM_CH_1,duty_period);
                //Motor_dir_A_Clear();
                //Motor_dir_B_Set();
                //duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_1,duty_period);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2,0);
            }
            break;
        case 2:
            if (SMC_ST_data.pwm_output > 0.0) //clockwise
            {
                duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_3,0);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_4,duty_period);
            }
            if (SMC_ST_data.pwm_output < 0.0) ////counterclockwise
            {
               duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_3,duty_period);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_4,0);
            }
            break;    
        case 3:
            if (SMC_ST_data.pwm_output > 0.0) //clockwise
            {
                duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_5,0);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_6,duty_period);
            }
            if (SMC_ST_data.pwm_output < 0.0) ////counterclockwise
            {
               duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_5,duty_period);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_6,0);
            }
            break;  
        case 4:
            if (SMC_ST_data.pwm_output > 0.0) //clockwise
            {
                duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_7,0);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_8,duty_period);
            }
            if (SMC_ST_data.pwm_output < 0.0) ////counterclockwise
            {
               duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_7,duty_period);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_8,0);
            }
            break; 
        case 5:
            if (SMC_ST_data.pwm_output > 0.0) //clockwise
            {
                duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_9,0);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_11,duty_period);
            }
            if (SMC_ST_data.pwm_output < 0.0) ////counterclockwise
            {
               duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_9,duty_period);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_11,0);
            }
            break; 
        case 6:
            if (SMC_ST_data.pwm_output > 0.0) //clockwise
            {
                duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                Motor_dir_B_Set();
                Motor_dir_A_Clear();
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_12,duty_period);
            }
            if (SMC_ST_data.pwm_output < 0.0) ////counterclockwise
            {
                duty_period = abs(SMC_ST_data.pwm_output)*(DUTY_MAX_PERIOD-1)/100;
                Motor_dir_A_Set();
                Motor_dir_B_Clear();
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_12,duty_period);
            }
            break; 
            
            
    }
    
    
}

int Control_Sign(float data)
{
    int sign = 0;
    if (data > 0)
    {
        sign = 1;
    }
    if (data < 0)
    {
        sign = -1;
    }
    return sign;
}

void Control_SendData()
{
    //Position of the motor 1
    uart_sent_data[0] = (int32_t)(*motor_control_1.position*100) >>24;
    uart_sent_data[1] = (int32_t)(*motor_control_1.position*100) >>16;
    uart_sent_data[2] = (int32_t)(*motor_control_1.position*100) >>8;
    uart_sent_data[3] = (int32_t)(*motor_control_1.position*100);
    //Reference of the motor
    uart_sent_data[4] = (int32_t)(motor_control_1.ref*100) >>24;
    uart_sent_data[5] = (int32_t)(motor_control_1.ref*100) >>16;
    uart_sent_data[6] = (int32_t)(motor_control_1.ref*100) >>8;
    uart_sent_data[7] = (int32_t)(motor_control_1.ref*100);
    //Error of the motor
    uart_sent_data[8]  = (int32_t)(motor_control_1.error*100) >>24;
    uart_sent_data[9]  = (int32_t)(motor_control_1.error*100) >>16;
    uart_sent_data[10] = (int32_t)(motor_control_1.error*100) >>8;
    uart_sent_data[11] = (int32_t)(motor_control_1.error*100);
    //Duty Output
    uart_sent_data[12]  = (int32_t)(motor_control_1.pwm_output*100) >>24;
    uart_sent_data[13]  = (int32_t)(motor_control_1.pwm_output*100) >>16;
    uart_sent_data[14]  = (int32_t)(motor_control_1.pwm_output*100) >>8;
    uart_sent_data[15]  = (int32_t)(motor_control_1.pwm_output*100);
    
    //Position of the motor 2
    uart_sent_data[16] = (int32_t)(*motor_control_2.position*100) >>24;
    uart_sent_data[17] = (int32_t)(*motor_control_2.position*100) >>16;
    uart_sent_data[18] = (int32_t)(*motor_control_2.position*100) >>8;
    uart_sent_data[19] = (int32_t)(*motor_control_2.position*100);
    //Reference of the motor
    uart_sent_data[20] = (int32_t)(motor_control_2.ref*100) >>24;
    uart_sent_data[21] = (int32_t)(motor_control_2.ref*100) >>16;
    uart_sent_data[22] = (int32_t)(motor_control_2.ref*100) >>8;
    uart_sent_data[23] = (int32_t)(motor_control_2.ref*100);
    //Error of the motor
    uart_sent_data[24]  = (int32_t)(motor_control_2.error*100) >>24;
    uart_sent_data[25]  = (int32_t)(motor_control_2.error*100) >>16;
    uart_sent_data[26] = (int32_t)(motor_control_2.error*100) >>8;
    uart_sent_data[27] = (int32_t)(motor_control_2.error*100);
    //Duty Output
    uart_sent_data[28]  = (int32_t)(motor_control_2.pwm_output*100) >>24;
    uart_sent_data[29]  = (int32_t)(motor_control_2.pwm_output*100) >>16;
    uart_sent_data[30]  = (int32_t)(motor_control_2.pwm_output*100) >>8;
    uart_sent_data[31]  = (int32_t)(motor_control_2.pwm_output*100);
    
    //Position of the motor 3
    uart_sent_data[32] = (int32_t)(*motor_control_3.position*100) >>24;
    uart_sent_data[33] = (int32_t)(*motor_control_3.position*100) >>16;
    uart_sent_data[34] = (int32_t)(*motor_control_3.position*100) >>8;
    uart_sent_data[35] = (int32_t)(*motor_control_3.position*100);
    //Reference of the motor
    uart_sent_data[36] = (int32_t)(motor_control_3.ref*100) >>24;
    uart_sent_data[37] = (int32_t)(motor_control_3.ref*100) >>16;
    uart_sent_data[38] = (int32_t)(motor_control_3.ref*100) >>8;
    uart_sent_data[39] = (int32_t)(motor_control_3.ref*100);
    //Error of the motor
    uart_sent_data[40]  = (int32_t)(motor_control_3.error*100) >>24;
    uart_sent_data[41]  = (int32_t)(motor_control_3.error*100) >>16;
    uart_sent_data[42]  = (int32_t)(motor_control_3.error*100) >>8;
    uart_sent_data[43]  = (int32_t)(motor_control_3.error*100);
    //Duty Output
    uart_sent_data[44]  = (int32_t)(motor_control_3.pwm_output*100) >>24;
    uart_sent_data[45]  = (int32_t)(motor_control_3.pwm_output*100) >>16;
    uart_sent_data[46]  = (int32_t)(motor_control_3.pwm_output*100) >>8;
    uart_sent_data[47]  = (int32_t)(motor_control_3.pwm_output*100);
    
    //Position of the motor 4
    uart_sent_data[48] = (int32_t)(*motor_control_4.position*100) >>24;
    uart_sent_data[49] = (int32_t)(*motor_control_4.position*100) >>16;
    uart_sent_data[50] = (int32_t)(*motor_control_4.position*100) >>8;
    uart_sent_data[51] = (int32_t)(*motor_control_4.position*100);
    //Reference of the motor
    uart_sent_data[52] = (int32_t)(motor_control_4.ref*100) >>24;
    uart_sent_data[53] = (int32_t)(motor_control_4.ref*100) >>16;
    uart_sent_data[54] = (int32_t)(motor_control_4.ref*100) >>8;
    uart_sent_data[55] = (int32_t)(motor_control_4.ref*100);
    //Error of the motor
    uart_sent_data[56]  = (int32_t)(motor_control_4.error*100) >>24;
    uart_sent_data[57]  = (int32_t)(motor_control_4.error*100) >>16;
    uart_sent_data[58]  = (int32_t)(motor_control_4.error*100) >>8;
    uart_sent_data[59]  = (int32_t)(motor_control_4.error*100);
    //Duty Output
    uart_sent_data[60]  = (int32_t)(motor_control_4.pwm_output*100) >>24;
    uart_sent_data[61]  = (int32_t)(motor_control_4.pwm_output*100) >>16;
    uart_sent_data[62]  = (int32_t)(motor_control_4.pwm_output*100) >>8;
    uart_sent_data[63]  = (int32_t)(motor_control_4.pwm_output*100);
    
    //Position of the motor 5
    uart_sent_data[64] = (int32_t)(*motor_control_5.position*100) >>24;
    uart_sent_data[65] = (int32_t)(*motor_control_5.position*100) >>16;
    uart_sent_data[66] = (int32_t)(*motor_control_5.position*100) >>8;
    uart_sent_data[67] = (int32_t)(*motor_control_5.position*100);
    //Reference of the motor
    uart_sent_data[68] = (int32_t)(motor_control_5.ref*100) >>24;
    uart_sent_data[69] = (int32_t)(motor_control_5.ref*100) >>16;
    uart_sent_data[70] = (int32_t)(motor_control_5.ref*100) >>8;
    uart_sent_data[71] = (int32_t)(motor_control_5.ref*100);
    //Error of the motor
    uart_sent_data[72]  = (int32_t)(motor_control_5.error*100) >>24;
    uart_sent_data[73]  = (int32_t)(motor_control_5.error*100) >>16;
    uart_sent_data[74]  = (int32_t)(motor_control_5.error*100) >>8;
    uart_sent_data[75]  = (int32_t)(motor_control_5.error*100);
    //Duty Output
    uart_sent_data[76]  = (int32_t)(motor_control_5.pwm_output*100) >>24;
    uart_sent_data[77]  = (int32_t)(motor_control_5.pwm_output*100) >>16;
    uart_sent_data[78]  = (int32_t)(motor_control_5.pwm_output*100) >>8;
    uart_sent_data[79]  = (int32_t)(motor_control_5.pwm_output*100);
    
    //Position of the motor 6
    uart_sent_data[80] = (int32_t)(*motor_control_6.position*100) >>24;
    uart_sent_data[81] = (int32_t)(*motor_control_6.position*100) >>16;
    uart_sent_data[82] = (int32_t)(*motor_control_6.position*100) >>8;
    uart_sent_data[83] = (int32_t)(*motor_control_6.position*100);
    //Reference of the motor
    uart_sent_data[84] = (int32_t)(motor_control_6.ref*100) >>24;
    uart_sent_data[85] = (int32_t)(motor_control_6.ref*100) >>16;
    uart_sent_data[86] = (int32_t)(motor_control_6.ref*100) >>8;
    uart_sent_data[87] = (int32_t)(motor_control_6.ref*100);
    //Error of the motor
    uart_sent_data[88]  = (int32_t)(motor_control_6.error*100) >>24;
    uart_sent_data[89]  = (int32_t)(motor_control_6.error*100) >>16;
    uart_sent_data[90]  = (int32_t)(motor_control_6.error*100) >>8;
    uart_sent_data[91]  = (int32_t)(motor_control_6.error*100);
    //Duty Output
    uart_sent_data[92]  = (int32_t)(motor_control_6.pwm_output*100) >>24;
    uart_sent_data[93]  = (int32_t)(motor_control_6.pwm_output*100) >>16;
    uart_sent_data[94]  = (int32_t)(motor_control_6.pwm_output*100) >>8;
    uart_sent_data[95]  = (int32_t)(motor_control_6.pwm_output*100);

    UART2_Write(&uart_sent_data[0],96);
}

/*uint8_t uart_sent_data[15] = {0};
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
 */