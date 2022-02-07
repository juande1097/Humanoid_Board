/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "AS5600.h"
#include "config/default/peripheral/mcpwm/plib_mcpwm.h"
#include "config/default/peripheral/mcpwm/plib_mcpwm_common.h"
#include "control.h"


as5600_sensor as5600;
uint8_t uart_sent_data[20] = {0};

uint16_t pwm_duty = 3000; //max 6000 %45 2700, 
uint16_t counter2 = 0;
uint8_t subiendo = 0; //0 subiendo
uint16_t time_change = 200; //cada 1s
uint16_t duty_step = 60; // 1%

/**********Peripheral call backs **********/
void I2C1_callback(uintptr_t context)
{
    AS5600_UpdateData();
    Control_SuperTwisting(1000,0.9,10); //2000,0.95,0.3
    //Control_PID(40,0.5,0.5); // Control_PID(0.068,0.08,0.0025); (100,0.045,10), (40,0.5,0.5)
    //AS5600_UpdateSerialData();
    //UART2_Write(&uart_sent_data[0],15);
}

void Timer1_callback(uint32_t status, uintptr_t context) //50ms
{
    LED5_Toggle();
    AS5600_ReadStatusPosition();
    counter2++;
    
    AS5600_UpdateDirection(as5600.direction); //Update the direction of the motor
    
    
    //MCPWM_ChannelPrimaryDutySet(MCPWM_CH_1,pwm_duty);
    
    
}


/**********Module Specific Functions**********/
void AS5600_Initialize(void)        ////Initializes the AD4111
{
    as5600.position = 0.0;
    as5600.old_position = 0.0;
    as5600.turns = 0;
    as5600.displacement = 0.0;
    as5600.speed = 0.0;
    as5600.direction = 0;
    as5600.magnet_error =0;
    as5600.variable_readed = NOTING_READED;
    //AS5600_UpdateDirection(as5600.direction); //Update the direction of the motor
    
    I2C1_CallbackRegister(&I2C1_callback,0);  
    TMR1_CallbackRegister(&Timer1_callback,0);  
    
    AS5600_ReadPosition();
    
}
void AS5600_UpdateData(void)
{
    
    switch (as5600.variable_readed)
    {
        case NOTING_READED:
        
        break;
        
        case POSITION:
            as5600.position = (float)(((((uint16_t)as5600.i2c_data_received[0]) <<8) | as5600.i2c_data_received[1])*TURN_DEGREES) / (AS5600_RESOLUTION);
            as5600.variable_readed = NOTING_READED;
            TMR1_Start();
        
        break;
        
        case STATUS_POSITION:
            as5600.old_position =  as5600.position;
            as5600.position = (float)(((((uint16_t)as5600.i2c_data_received[1]) <<8) | as5600.i2c_data_received[2])*TURN_DEGREES) / (AS5600_RESOLUTION);
            
            //check for complete turn and calculate speed
            if ((as5600.position - as5600.old_position) < -0.5) //clockwise return to 0 to add a turn 
            {
                as5600.turns += 1;
                as5600.speed = (1- as5600.old_position + as5600.position)*SPEED_CONSTANT;
            }
            else if ((as5600.position - as5600.old_position) > 0.5) //counterclockwise return to 1 to add a turn 
            {
                as5600.turns -= 1;
                as5600.speed = (-1- as5600.old_position + as5600.position)*SPEED_CONSTANT;
            }
            else
            {
                as5600.speed = (as5600.position - as5600.old_position)*SPEED_CONSTANT; //speed on RPM
            }
            as5600.displacement = as5600.turns + as5600.position;
                        
            //Magnet error check
            if ( ((as5600.i2c_data_received[0] >> 3 & 0x01) == 1)  || ((as5600.i2c_data_received[0] >> 4 & 0x01) == 1) || ((as5600.i2c_data_received[0] >> 5 & 0x01) == 0))
            {
                as5600.magnet_error = 1;
            }
            else if ( ((as5600.i2c_data_received[0] >> 5 & 0x01) == 1) )
            {
               as5600.magnet_error = 0; 
            }
            as5600.variable_readed = NOTING_READED;
            
        break;
        
        case CONFIG_OUTPUT_STATUS:
        
        break;
        
        default:
                
        break;        
    }
}

void AS5600_ReadStatusPosition(void) //Read position and status variable of the as5600_sensor 
{
    uint8_t start_address = AS5600_STATUS_REG;
    I2C1_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &as5600.i2c_data_received[0], 3);
    as5600.variable_readed = STATUS_POSITION;
}

void AS5600_ReadPosition(void) //Read position variable of the as5600_sensor 
{
    uint8_t start_address = AS5600_RAW_ANGLE_REG;
    I2C1_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &as5600.i2c_data_received[0], 2);
    as5600.variable_readed = POSITION;
    
}

void AS5600_UpdateDirection(uint16_t direction)
{
    //Direction change
    if (as5600.direction == 0) //clockwise
    {
        Motor_dir_A_Clear();
        Motor_dir_B_Set();
    }
    if (as5600.direction == 1) ////counterclockwise
    {
       
        Motor_dir_A_Set();
        Motor_dir_B_Clear();
    }
}

void AS5600_UpdateSerialData (void)
{
    uart_sent_data[0] = as5600.direction;
    uart_sent_data[1] = (uint8_t)(as5600.position*100);
    uart_sent_data[2] = (uint8_t)(as5600.turns >> 8);
    uart_sent_data[3] = (uint8_t)as5600.turns;
    uart_sent_data[4] = (uint8_t)((uint16_t)as5600.displacement >> 8);
    uart_sent_data[5] = (uint8_t)as5600.displacement;
    uart_sent_data[6] = (uint8_t)((uint16_t)(((float)as5600.displacement - (uint16_t)as5600.displacement)*1000) >> 8);
    uart_sent_data[7] = (uint8_t)((as5600.displacement - (uint16_t)as5600.displacement)*1000);
    uart_sent_data[8] = (uint8_t)as5600.speed;
    uart_sent_data[9] = (uint8_t)((as5600.speed - (uint16_t)as5600.speed)*100);
    uart_sent_data[10] = (uint8_t)as5600.magnet_error; 
    uart_sent_data[11] = as5600.i2c_data_received[1];
    uart_sent_data[12] = as5600.i2c_data_received[2];
    uart_sent_data[13] = (uint8_t)((pwm_duty*100)/6000);
    uart_sent_data[14] = ((((float)pwm_duty*100.0)/6000.0) - (uint8_t)((pwm_duty*100)/6000))*100;
}

