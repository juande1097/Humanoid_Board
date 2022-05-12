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


as5600_sensor sensor_1;
as5600_sensor sensor_2;
as5600_sensor sensor_3;
as5048a_sensor sensor_4;
as5048a_sensor sensor_5;
as5048a_sensor sensor_6;

STA_data motor_control_1;
STA_data motor_control_2;
STA_data motor_control_3;
STA_data motor_control_4;
STA_data motor_control_5;
STA_data motor_control_6;


uint8_t uart_sent_data[20] = {0};
uint8_t i2c_data2[5]={0};

uint16_t pwm_duty = 3000; //max 6000 %45 2700, 
uint16_t counter = 0;
uint8_t subiendo = 0; //0 subiendo
uint16_t time_change = 200; //cada 1s
uint16_t duty_step = 60; // 1%

/**********Peripheral call backs **********/
void I2C1_callback(uintptr_t context)
{
    AS5600_UpdateData(&sensor_1);
    Control_SuperTwisting(&motor_control_1);
    
    LED4_Toggle();
}

void I2C2_callback(uintptr_t context)
{
    AS5600_UpdateData(&sensor_3);
    Control_SuperTwisting(&motor_control_3);
    //Control_SendData();
}
void I2C4_callback(uintptr_t context)
{
    AS5600_UpdateData(&sensor_2);
    Control_SuperTwisting(&motor_control_2);
    //Control_SendData(motor_control_2);
    AS5048A_ReadStatusPosition(&sensor_4,1);
    AS5048A_ReadStatusPosition(&sensor_5,2);
    AS5048A_ReadStatusPosition(&sensor_6,3);
    
}
void SPI1_callback(uintptr_t context)
{
    AS5048_CS_1_Set();
    AS5048A_UpdateData(&sensor_4);
    Control_SuperTwisting(&motor_control_4);
}
void SPI3_callback(uintptr_t context)
{
    AS5048_CS_2_Set();
    AS5048A_UpdateData(&sensor_5);
    Control_SuperTwisting(&motor_control_5);
}
void SPI4_callback(uintptr_t context)
{
    AS5048_CS_3_Set();
    AS5048A_UpdateData(&sensor_6);
    Control_SuperTwisting(&motor_control_6);
}
void Timer1_callback(uint32_t status, uintptr_t context) //10ms
{
    //LED5_Toggle();
    AS5600_ReadStatusPosition(&sensor_1,1);
    AS5600_ReadStatusPosition(&sensor_2,4);
    AS5600_ReadStatusPosition(&sensor_3,2);
//    AS5048A_ReadStatusPosition(&sensor_4,1);
//    AS5048A_ReadStatusPosition(&sensor_5,2);
//    AS5048A_ReadStatusPosition(&sensor_4,3);
    
    
    
    counter++;
    if(counter >= 100)
    {
        counter =0;
        Control_SendData();
        
        //AS5048A_UpdateSerialData();
        
    }
  
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
    sensor_1.magnet_error =0;
    sensor_1.variable_readed = NOTING_READED;
    
    sensor_2.position = 0.0;
    sensor_2.old_position = 0.0;
    sensor_2.turns = 0;
    sensor_2.displacement = 0.0;
    sensor_2.speed = 0.0;
    sensor_2.direction = 0;
    sensor_2.magnet_error =0;
    sensor_2.variable_readed = NOTING_READED;
    
    sensor_3.position = 0.0;
    sensor_3.old_position = 0.0;
    sensor_3.turns = 0;
    sensor_3.displacement = 0.0;
    sensor_3.speed = 0.0;
    sensor_3.direction = 0;
    sensor_3.magnet_error =0;
    sensor_3.variable_readed = NOTING_READED;
    
    sensor_4.position = 0.0;
    sensor_4.old_position = 0.0;
    sensor_4.turns = 0;
    sensor_4.displacement = 0.0;
    sensor_4.speed = 0.0;
    sensor_4.direction = 0;
    sensor_4.flag_error =0;
    sensor_4.variable_readed = NOTING_READED;
    
    
    Control_initialize(163,&motor_control_1,&sensor_1,1, 500, 0.9, 2);
    Control_initialize(150,&motor_control_2,&sensor_2,2, 400, 0.7, 2); //130-160
    Control_initialize(140,&motor_control_3,&sensor_3,3, 400, 0.7, 2);
    Control_initialize_As5048(160,&motor_control_4,&sensor_4,4, 400, 0.7, 2);
    Control_initialize_As5048(210,&motor_control_5,&sensor_5,5, 400, 0.7, 2);
    Control_initialize_As5048(284,&motor_control_6,&sensor_6,6, 400, 0.7, 2);
    
    I2C1_CallbackRegister(&I2C1_callback,0);  
    I2C2_CallbackRegister(&I2C2_callback,0); 
    I2C4_CallbackRegister(&I2C4_callback,0);
    TMR1_CallbackRegister(&Timer1_callback,0);
    SPI1_CallbackRegister(&SPI1_callback,0);
    SPI3_CallbackRegister(&SPI3_callback,0);
    SPI4_CallbackRegister(&SPI4_callback,0);
    
    AS5600_ReadPosition(&sensor_1);
    //AS5600_ReadPosition(&sensor_2);
    //AS5600_ReadPosition(&sensor_3);
    

    
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
                        
            sensor->variable_readed = NOTING_READED;
            
        
        break;
        
        case CLEARFLAG_POSITION:
            sensor->old_position =  sensor->position;
            sensor->position = (float)(((sensor->spi_data_received[3]&0x3FFF)*TURN_DEGREES)) / (AS5048a_RESOLUTION);

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
void AS5048A_ReadStatusPosition(as5048a_sensor *sensor, uint8_t channel)
{
    uint16_t spi_data_write[4] = {0};
    spi_data_write[0]=AS5048A_SPI_CMD_READ | AS5048A_CLEA_RERROR_REG;
    spi_data_write[2]=AS5048A_SPI_CMD_READ | AS5048A_ANGLE_REG;
    spi_data_write[2] |= getParity(spi_data_write[2]) << 15;
    //spi_data_write[1] = spi_data_write[0]; 
    switch(channel)
    {
        case 1:
            AS5048_CS_1_Clear();
            SPI1_WriteRead(&spi_data_write[0],8,&sensor->spi_data_received[0],8);
            sensor->variable_readed = CLEARFLAG_POSITION;
        break;
        case 2:
            AS5048_CS_2_Clear();
            SPI3_WriteRead(&spi_data_write[0],8,&sensor->spi_data_received[0],8);
            sensor->variable_readed = CLEARFLAG_POSITION;
        break;
        case 3:
            AS5048_CS_3_Clear();
            SPI4_WriteRead(&spi_data_write[0],8,&sensor->spi_data_received[0],8);
            sensor->variable_readed = CLEARFLAG_POSITION;
        break;
    }
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
    uart_sent_data[0] = (int32_t)(sensor_4.position*100) >>24;
    uart_sent_data[1] = (int32_t)(sensor_4.position*100) >>16;
    uart_sent_data[2] = (int32_t)(sensor_4.position*100) >>8;
    uart_sent_data[3] = (int32_t)(sensor_4.position*100);
    
    UART2_Write(&uart_sent_data[0],4);
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