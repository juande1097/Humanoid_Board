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
#include "control.h"


as5048a_sensor sensor_1;


STA_data motor_control_1;




uint8_t uart_sent_data[20] = {0};
uint8_t i2c_data2[5]={0};

float pwm_duty_2 = 0; //max 6000 %45 2700, 
uint32_t counter = 0;
uint16_t counter_send = 0;
uint16_t counter_change = 0;
uint32_t idx = 0;
float periods[4]={0.04,0.06,0.1,0};
uint8_t subiendo = 0; //0 subiendo
uint16_t time_change = 200; //cada 1s
uint16_t duty_step = 60; // 1%
uint8_t button_status = 1;
uint16_t pwm_duty_period = 0;
uint16_t umbral = 0;

/**********Peripheral call backs **********/

void SPI1_callback(uintptr_t context)
{
    AS5048_CS_1_Set();
    LED4_Toggle();
    AS5048A_UpdateData(&sensor_1);
    
    //Control_StateFeedback(&motor_control_1);
    //Control_SlidingMode(&motor_control_1);
    Control_SuperTwisting(&motor_control_1);
    
}

void Timer1_callback(uint32_t status, uintptr_t context) //1ms
{
    //LED5_Toggle();
    //AS5600_ReadStatusPosition(&sensor_1,1);
    //AS5600_ReadStatusPosition(&sensor_2,4);
    //AS5600_ReadStatusPosition(&sensor_3,2);
    AS5048A_ReadStatusPosition(&sensor_1);
    
    counter++;
    if(counter >= 10)
    {
        counter =0;
        //Control_SendData();
        AS5048A_UpdateSerialData();
        
    }
    //signal generation for implementation
    /*if (BTN4_Get() == 0)
    {
        button_status = 0;
    }
    
    
        counter_send++;
        if (counter_send > 10)
        {
            counter_send = 0;

            
            if (button_status == 0)
            {
                counter++;
                counter_change++;
                pwm_duty_2 = 50*(sin(periods[idx]*counter+4.712389)+1);
                pwm_duty_period = abs(pwm_duty_2)*(DUTY_MAX_PERIOD-1)/100;
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_1,0);
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2,pwm_duty_period);
            }
            //Control_SendData();
            AS5048A_UpdateSerialData();
        }

        if (counter_change >= 314)
        {
            if (idx < 3)
            {
                idx++;
            }

            counter_change=0;
        }*/
        
//    counter++;
//    if(counter >= 100)
//    {
//        counter =0;
//        Control_SendData();
//        
//    }
  
}


/**********Module Specific Functions**********/
void AS5600_Initialize(void)        ////Initializes the AD4111
{
    sensor_1.position = 0.0;
    sensor_1.old_position = 0.0;
    sensor_1.turns = 0;
    sensor_1.displacement = 0.0;
    sensor_1.speed = 0.0;
    sensor_1.direction = 0;
    sensor_1.flag_error  =0;
    sensor_1.variable_readed = NOTING_READED;
    
     
    
    //Control_initialize(50,&motor_control_1,&sensor_1,1, 500, 0.9, 2);
    //Control_initialize(50,&motor_control_2,&sensor_2,2, 400, 0.7, 2);
    //Control_initialize(50,&motor_control_3,&sensor_3,3, 400, 0.7, 2);
    
    //Control_initialize_As5048(0.5,&motor_control_1,&sensor_1,1, 10, 0, 0); //state feedback
    //Control_initialize_As5048(1,&motor_control_1,&sensor_1,1, 50, 0.1, 0); //sliding mode
    Control_initialize_As5048(0.5,&motor_control_1,&sensor_1,1, 400, 0.7, 2); //super twisting
    
    //I2C1_CallbackRegister(&I2C1_callback,0);  
    //I2C2_CallbackRegister(&I2C2_callback,0); 
    //I2C4_CallbackRegister(&I2C4_callback,0);
    TMR1_CallbackRegister(&Timer1_callback,0);
    SPI1_CallbackRegister(&SPI1_callback,0);
    
    //AS5600_ReadPosition(&sensor_1);
    //AS5600_ReadPosition(&sensor_2);
    //AS5600_ReadPosition(&sensor_3);
    
    TMR1_Start();
    
}
void AS5600_UpdateData(as5600_sensor *sensor)
{
    
    switch (sensor->variable_readed)
    {
        case NOTING_READED:
        
        break;
        
        case POSITION:
            sensor->position = (float)(((((uint16_t)sensor->i2c_data_received[0]) <<8) | sensor->i2c_data_received[1])*TURN_DEGREES) / (AS5600_RESOLUTION);
            sensor->variable_readed = NOTING_READED;
            TMR1_Start();
        
        break;
        
        case STATUS_POSITION:
            sensor->old_position =  sensor->position;
            sensor->position = (float)(((((uint16_t)sensor->i2c_data_received[1]) <<8) | sensor->i2c_data_received[2])*TURN_DEGREES) / (AS5600_RESOLUTION);
            
            //check for complete turn and calculate speed
            if ((sensor->position - sensor->old_position) < -0.5) //clockwise return to 0 to add a turn 
            {
                sensor->turns += 1;
                sensor->speed = (1- sensor->old_position + sensor->position)*SPEED_CONSTANT;
            }
            else if ((sensor->position - sensor->old_position) > 0.5) //counterclockwise return to 1 to add a turn 
            {
                sensor->turns -= 1;
                sensor->speed = (-1- sensor->old_position + sensor->position)*SPEED_CONSTANT;
            }
            else
            {
                sensor->speed = (sensor->position - sensor->old_position)*SPEED_CONSTANT; //speed on RPM
            }
            sensor->displacement = sensor->turns + sensor->position;
                        
            //Magnet error check
            if ( ((sensor->i2c_data_received[0] >> 3 & 0x01) == 1)  || ((sensor->i2c_data_received[0] >> 4 & 0x01) == 1) || ((sensor->i2c_data_received[0] >> 5 & 0x01) == 0))
            {
                sensor->magnet_error = 1;
            }
            else if ( ((sensor->i2c_data_received[0] >> 5 & 0x01) == 1) )
            {
               sensor->magnet_error = 0; 
            }
            sensor->variable_readed = NOTING_READED;
            
        break;
        
        case CONFIG_OUTPUT_STATUS:
        
        break;
        
        default:
                
        break;        
    }
}

void AS5600_ReadStatusPosition(as5600_sensor *sensor, uint8_t channel) //Read position and status variable of the sensor_1_sensor 
{
    uint8_t start_address = AS5600_STATUS_REG;
    switch(channel)
    {
        case 1:
            I2C1_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 3);
            break;
        case 2:
            I2C2_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 3);
            break;    
        case 3:
            //I2C3_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 3);
            break;   
        case 4:
            I2C4_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 3);
            break;   
    }
    sensor->variable_readed = STATUS_POSITION;
}

void AS5600_ReadPosition(as5600_sensor *sensor) //Read position variable of the as5600_sensor 
{
    uint8_t start_address = AS5600_RAW_ANGLE_REG;
    I2C1_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 2);
    sensor->variable_readed = POSITION;
    
}

void AS5048A_UpdateData(as5048a_sensor *sensor)
{
    
    switch (sensor->variable_readed)
    {
        case NOTING_READED:
        
        break;
        
        case POSITION:
            sensor->old_position =  sensor->position;
            sensor->position = (float)(((sensor->spi_data_received[1]&0x3FFF)*TURN_DEGREES)) / (AS5048a_RESOLUTION);
            
            
            //check for complete turn and calculate speed
            if ((sensor->position < 5 && sensor->old_position > 355)) //clockwise return to 0 to add a turn 
            {
                
                sensor->turns += 1;
                sensor->speed = (1- sensor->old_position + sensor->position)*SPEED_CONSTANT;
                    
                
                
            }
            else if ((sensor->position > 355  && sensor->old_position < 5)) //counterclockwise return to 1 to add a turn 
            {
                //sensor->turns -= 1;
                sensor->speed = (-1- sensor->old_position + sensor->position)*SPEED_CONSTANT;
            }
            else
            {
                sensor->speed = (sensor->position - sensor->old_position)*SPEED_CONSTANT; //speed on RPM
            }
            sensor->displacement = sensor->turns + sensor->position;
                        
            sensor->variable_readed = NOTING_READED;
            
        
        break;
        
        case CLEARFLAG_POSITION:
            sensor->old_position =  sensor->position;
            sensor->position = (float)(((sensor->spi_data_received[3]&0x3FFF)*TURN_DEGREES)) / (AS5048a_RESOLUTION);
            
            if (sensor->position > 200 && sensor->position < 300)
            {
                umbral = 0;
            }

            //check for complete turn and calculate speed
            if ((sensor->position < 5 && sensor->position > 0.055 && sensor->old_position > 355)) //clockwise return to 0 to add a turn 
            {
                if (umbral == 0)
                {
                    sensor->turns += 1;
                    sensor->speed = (1- sensor->old_position + sensor->position)*SPEED_CONSTANT;
                    umbral =1;
                }    
            }
            else if ((sensor->position > 355  && sensor->old_position < 5)) //counterclockwise return to 1 to add a turn 
            {
                //sensor->turns -= 1;
                sensor->speed = (-1- sensor->old_position + sensor->position)*SPEED_CONSTANT;
            }
            else
            {
                sensor->speed = (sensor->position - sensor->old_position)*SPEED_CONSTANT; //speed on RPM
            }
            sensor->displacement = sensor->turns + sensor->position/360;
            sensor->angle = sensor->turns*360 + sensor->position;
                
            //Flag Error
            sensor->flag_error = sensor->spi_data_received[1];
            
            sensor->variable_readed = NOTING_READED;
            
        break;
        
        case CONFIG_OUTPUT_STATUS:
        
        break;
        
        default:
                
        break;        
    }
}

void AS5048A_ReadPosition(as5048a_sensor *sensor)
{
    uint16_t spi_data_write[4] = {0};
    //spi_data_write[0]=AS5048A_SPI_CMD_READ | AS5048A_CLEA_RERROR_REG;
    spi_data_write[0]=AS5048A_SPI_CMD_READ | AS5048A_ANGLE_REG;
    spi_data_write[0] |= getParity(spi_data_write[0]) << 15;
    spi_data_write[1]= spi_data_write[0];
    AS5048_CS_1_Clear();
    SPI1_WriteRead(&spi_data_write[0],6,&sensor->spi_data_received[0],6);
    sensor->variable_readed = POSITION;
}
void AS5048A_ReadStatusPosition(as5048a_sensor *sensor)
{
    uint16_t spi_data_write[4] = {0};
    spi_data_write[0]=AS5048A_SPI_CMD_READ | AS5048A_CLEA_RERROR_REG;
    spi_data_write[2]=AS5048A_SPI_CMD_READ | AS5048A_ANGLE_REG;
    spi_data_write[2] |= getParity(spi_data_write[2]) << 15;
    //spi_data_write[1] = spi_data_write[0]; 
    AS5048_CS_1_Clear();
    SPI1_WriteRead(&spi_data_write[0],8,&sensor->spi_data_received[0],8);
    sensor->variable_readed = CLEARFLAG_POSITION;
}

void AS5600_UpdateSerialData (void)
{
    /*uart_sent_data[0] = sensor_1.direction;
    uart_sent_data[1] = (uint8_t)(sensor_1.position*100);
    uart_sent_data[2] = (uint8_t)(sensor_1.turns >> 8);
    uart_sent_data[3] = (uint8_t)sensor_1.turns;
    uart_sent_data[4] = (uint8_t)((uint16_t)sensor_1.displacement >> 8);
    uart_sent_data[5] = (uint8_t)sensor_1.displacement;
    uart_sent_data[6] = (uint8_t)((uint16_t)(((float)sensor_1.displacement - (uint16_t)sensor_1.displacement)*1000) >> 8);
    uart_sent_data[7] = (uint8_t)((sensor_1.displacement - (uint16_t)sensor_1.displacement)*1000);
    uart_sent_data[8] = (uint8_t)sensor_1.speed;
    uart_sent_data[9] = (uint8_t)((sensor_1.speed - (uint16_t)sensor_1.speed)*100);
    uart_sent_data[10] = (uint8_t)sensor_1.magnet_error; 
    uart_sent_data[11] = sensor_1.i2c_data_received[1];
    uart_sent_data[12] = sensor_1.i2c_data_received[2];
    uart_sent_data[13] = (uint8_t)((pwm_duty*100)/6000);
    uart_sent_data[14] = ((((float)pwm_duty*100.0)/6000.0) - (uint8_t)((pwm_duty*100)/6000))*100;*/
}
void AS5048A_UpdateSerialData (void)
{
    //Position of the motor 1
    uart_sent_data[0] = (int32_t)(sensor_1.position*100) >>24;
    uart_sent_data[1] = (int32_t)(sensor_1.position*100) >>16;
    uart_sent_data[2] = (int32_t)(sensor_1.position*100) >>8;
    uart_sent_data[3] = (int32_t)(sensor_1.position*100);
    
    //Total position angle
    uart_sent_data[4] = (int32_t)(sensor_1.angle*100) >>24;
    uart_sent_data[5] = (int32_t)(sensor_1.angle*100) >>16;
    uart_sent_data[6] = (int32_t)(sensor_1.angle*100) >>8;
    uart_sent_data[7] = (int32_t)(sensor_1.angle*100);
    
    uart_sent_data[8]  = (int32_t)(periods[idx]*100) >>24;
    uart_sent_data[9]  = (int32_t)(periods[idx]*100) >>16;
    uart_sent_data[10] = (int32_t)(periods[idx]*100) >>8;
    uart_sent_data[11] = (int32_t)(periods[idx]*100);
    
    uart_sent_data[12]  = (int32_t)(motor_control_1.pwm_output*100) >>24;
    uart_sent_data[13]  = (int32_t)(motor_control_1.pwm_output*100) >>16;
    uart_sent_data[14] = (int32_t)(motor_control_1.pwm_output*100) >>8;
    uart_sent_data[15] = (int32_t)(motor_control_1.pwm_output*100);
    
    uart_sent_data[16]  = (int32_t)(motor_control_1.ref*100) >>24;
    uart_sent_data[17]  = (int32_t)(motor_control_1.ref*100) >>16;
    uart_sent_data[18] = (int32_t)(motor_control_1.ref*100) >>8;
    uart_sent_data[19] = (int32_t)(motor_control_1.ref*100);
    UART2_Write(&uart_sent_data[0],20);
    
    //UART2_Write(&uart_sent_data[0],4);
}

bool getParity(uint16_t data)
{
    bool parity = 0;
    while (data)
    {
        parity = !parity;
        data      = data & (data - 1);
    }       
    return parity;
}