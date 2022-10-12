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

uint8_t humanoid_leg = 1; //0 Right, 1 Left
uint8_t humanoid_trajectory = 0; //0 Squats, 1 Gait
uint16_t humanoid_traj_size[2] = {2000,2239};
uint8_t zpos_send = 0;
uint8_t counter_z = 0;

uint16_t counter = 0;
uint16_t counter_trajectory_period = 0;
uint16_t trajectory_period = 5;
uint32_t counter_trajectory_next = 0;
uint8_t enable_trajectory = 0;
extern float squat_trajectory[2000][6];
extern float walking_trajectory_L[2240][6];
extern float walking_trajectory_R[2240][6];

/**********Peripheral call backs **********/
void I2C1_callback(uintptr_t context)
{
    AS5600_UpdateData(&sensor_1);
    Control_PID(&motor_control_1);
    
    LED4_Toggle();
}

void I2C2_callback(uintptr_t context)
{
    AS5600_UpdateData(&sensor_3);
    Control_PID(&motor_control_3);
}
void I2C4_callback(uintptr_t context)
{
    AS5600_UpdateData(&sensor_2);
    Control_PID(&motor_control_2);
    
        AS5048A_ReadStatusPosition(&sensor_4,1);
        AS5048A_ReadStatusPosition(&sensor_5,2);
        AS5048A_ReadStatusPosition(&sensor_6,3);
    
    
    
}
void SPI1_callback(uintptr_t context)
{
    AS5048_CS_1_Set();
    AS5048A_UpdateData(&sensor_4);
    Control_PID(&motor_control_4);
}
void SPI3_callback(uintptr_t context)
{
    AS5048_CS_2_Set();
    AS5048A_UpdateData(&sensor_5);
    Control_PID(&motor_control_5);
}
void SPI4_callback(uintptr_t context)
{
    AS5048_CS_3_Set();
    AS5048A_UpdateData(&sensor_6);
    Control_PID(&motor_control_6);
}
void Timer1_callback(uint32_t status, uintptr_t context) //10ms
{
        AS5600_ReadPosition(&sensor_1);
        AS5600_ReadPosition(&sensor_2);
        AS5600_ReadPosition(&sensor_3);
    
    counter++;
    if(counter >= 10) 
    {
        counter =0;
        Control_SendData();    
    }
    if (enable_trajectory == 1)
    {
        counter_trajectory_period++;
    if(counter_trajectory_period >= trajectory_period)
    {
        counter_trajectory_period =0;
        if  (humanoid_leg == 0) //Derecha
        {
            if (humanoid_trajectory==0)
            {
                motor_control_1.ref = 180 - squat_trajectory[counter_trajectory_next][0];
                motor_control_2.ref = 180 - squat_trajectory[counter_trajectory_next][1];
                motor_control_3.ref = 180 - squat_trajectory[counter_trajectory_next][2];
                motor_control_4.ref = 180 - squat_trajectory[counter_trajectory_next][3];
                motor_control_5.ref = 180 - squat_trajectory[counter_trajectory_next][4];
                motor_control_6.ref = 180 - squat_trajectory[counter_trajectory_next][5];
                
            }
            else
            {
                motor_control_1.ref = 180 - walking_trajectory_R[counter_trajectory_next][0];
                motor_control_2.ref = 180 - walking_trajectory_R[counter_trajectory_next][1];
                motor_control_3.ref = 180 - walking_trajectory_R[counter_trajectory_next][2];
                motor_control_4.ref = 180 - walking_trajectory_R[counter_trajectory_next][3];
                motor_control_5.ref = 180 + walking_trajectory_R[counter_trajectory_next][4];
                motor_control_6.ref = 180 - walking_trajectory_R[counter_trajectory_next][5];
            }
            
        }
        else
        {
            if (humanoid_trajectory==0)
            {
                motor_control_1.ref = 180 + squat_trajectory[counter_trajectory_next][0];
                motor_control_2.ref = 180 + squat_trajectory[counter_trajectory_next][1];
                motor_control_3.ref = 180 + squat_trajectory[counter_trajectory_next][2];
                motor_control_4.ref = 180 + squat_trajectory[counter_trajectory_next][3];
                motor_control_5.ref = 180 + squat_trajectory[counter_trajectory_next][4];
                motor_control_6.ref = 180 + squat_trajectory[counter_trajectory_next][5];
            }
            else
            {
                motor_control_1.ref = 180 + walking_trajectory_L[counter_trajectory_next][0];
                motor_control_2.ref = 180 + walking_trajectory_L[counter_trajectory_next][1];
                motor_control_3.ref = 180 + walking_trajectory_L[counter_trajectory_next][2];
                motor_control_4.ref = 180 + walking_trajectory_L[counter_trajectory_next][3];
                motor_control_5.ref = 180 + walking_trajectory_L[counter_trajectory_next][4];
                motor_control_6.ref = 180 + walking_trajectory_L[counter_trajectory_next][5];
            }
            
        }
        
        counter_trajectory_next++;
        if (counter_trajectory_next > humanoid_traj_size[humanoid_trajectory]) //change for size
        {
            counter_trajectory_next = 0;
        }
    }
        
    }
    
    
    
  
}


/**********Module Specific Functions**********/
void AS5600_Initialize(void)        ////Initializes the AD4111
{
    if  (humanoid_leg == 0) //Derecha the zero angle is configured here
    {
        AS5600_SensorInit(&sensor_1,1,118); //117 //118
        AS5600_SensorInit(&sensor_2,2,279); //283 //283
        AS5600_SensorInit(&sensor_3,3,205.5); //206  //206
        AS5048A_SensorInit(&sensor_4,4,178.5); //178  //179
        AS5048A_SensorInit(&sensor_5,5,246); //245  //244
        AS5048A_SensorInit(&sensor_6,6,260); //265 //257
    }
    else
    {
        AS5600_SensorInit(&sensor_1,1,348); //347 //348
        AS5600_SensorInit(&sensor_2,2,78);  //68 //68
        AS5600_SensorInit(&sensor_3,3,303); //299 //302
        AS5048A_SensorInit(&sensor_4,4,331); //330 //331
        AS5048A_SensorInit(&sensor_5,5,29);  //30 //27
        AS5048A_SensorInit(&sensor_6,6,94); //102 //94
    }
        
    
    
    Control_initialize(180,&motor_control_1,&sensor_1,1, 20, 0.08, 0.01);        //163 //285 //100 0.7 2 //300, 2, 2); 200/2/2   PID
    Control_initialize(180,&motor_control_2,&sensor_2,2, 20, 0.08, 0.01);        //150 //285
    Control_initialize(180,&motor_control_3,&sensor_3,3, 20, 0.08, 0.01);        //140 //11
    Control_initialize_As5048(180,&motor_control_4,&sensor_4,4, 20, 0.08, 0.01); //160 //350
    Control_initialize_As5048(180,&motor_control_5,&sensor_5,5, 20, 0.08, 0.01); //210 //15
    Control_initialize_As5048(180,&motor_control_6,&sensor_6,6, 20, 0.08, 0.01); //284 //50
    
    I2C1_CallbackRegister(&I2C1_callback,0);  
    I2C2_CallbackRegister(&I2C2_callback,0); 
    I2C4_CallbackRegister(&I2C4_callback,0);
    TMR1_CallbackRegister(&Timer1_callback,0);
    SPI1_CallbackRegister(&SPI1_callback,0);
    SPI3_CallbackRegister(&SPI3_callback,0);
    SPI4_CallbackRegister(&SPI4_callback,0);
    

        AS5600_Write_ZPOS(&sensor_1);
        AS5600_Write_ZPOS(&sensor_2);
        AS5600_Write_ZPOS(&sensor_3);
        AS5048A_WriteZPOS(&sensor_4);
        AS5048A_WriteZPOS(&sensor_5);
        AS5048A_WriteZPOS(&sensor_6);
        
        
    

}
void AS5600_SensorInit(as5600_sensor *sensor, uint8_t sensor_num, float zero_pos)
{
    sensor->sensor_number = sensor_num;
    sensor->zero_position = zero_pos;
    sensor->position = 0.0;
    sensor->old_position = 0.0;
    sensor->turns = 0;
    sensor->displacement = 0.0;
    sensor->speed = 0.0;
    sensor->direction = 0;
    sensor->magnet_error =0; 
    sensor->variable_readed = NOTING_READED;
}
void AS5048A_SensorInit(as5048a_sensor *sensor, uint8_t sensor_num, float zero_pos)
{
    sensor->sensor_number = sensor_num;
    sensor->zero_position = zero_pos;
    sensor->position = 0.0;
    sensor->old_position = 0.0;
    sensor->turns = 0;
    sensor->displacement = 0.0;
    sensor->speed = 0.0;
    sensor->direction = 0;
    sensor->variable_readed = NOTING_READED;
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
        
        case ZPOS:           
            TMR1_Start();
            sensor->variable_readed = NOTING_READED;
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
            I2C4_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 3);
            break;    
        case 3:
            //I2C3_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 3);
            break;   
        case 4:
            I2C2_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 3);
            break;   
    }
    sensor->variable_readed = STATUS_POSITION;
}

void AS5600_ReadPosition(as5600_sensor *sensor) //Read position variable of the as5600_sensor 
{
    uint8_t start_address = AS5600_ANGLE_REG;
    switch(sensor->sensor_number)
    {
        case 1:
            I2C1_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 2);
        break;
        case 2:
            I2C4_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 2);
        break;
        case 3:
            I2C2_WriteRead(AS5600_SLAVE_ADDRESS,&start_address,1, &sensor->i2c_data_received[0], 2);
         break;
         
    }
    
    
    sensor->variable_readed = POSITION;
    
}
void AS5600_Write_ZPOS(as5600_sensor *sensor) //Write the start position of the sensor
{

    uint8_t start_address = AS5600_ZPOS_REG;
    sensor->i2c_data_send[0] = start_address;
    sensor->i2c_data_send[1] = (uint16_t)(sensor->zero_position*AS5600_RESOLUTION/360) >> 8;
    sensor->i2c_data_send[2] = (uint16_t)(sensor->zero_position*AS5600_RESOLUTION/360) & 0xFF;    
    
    switch(sensor->sensor_number)
    {
        case 1:
            I2C1_Write(AS5600_SLAVE_ADDRESS,&sensor->i2c_data_send[0],3);
            break;
        case 2:
            I2C4_Write(AS5600_SLAVE_ADDRESS,&sensor->i2c_data_send[0],3);
            break;    
        case 3:
            I2C2_Write(AS5600_SLAVE_ADDRESS,&sensor->i2c_data_send[0],3);
            break;    
    }

    sensor->variable_readed = ZPOS;
    
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
        
        case ZPOS:
        
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
void AS5048A_WriteZPOS(as5048a_sensor *sensor)
{
    sensor->spi_data_send[0] = AS5048A_SPI_CMD_READ | AS5048A_DIAGNOSTICS_REG;
    sensor->spi_data_send[0] |= getParity(sensor->spi_data_send[0]) << 15;
    sensor->spi_data_send[1] = 0;
    sensor->spi_data_send[2]=AS5048A_SPI_CMD_WRITE | AS5048A_ZERO_POS_HI_REG;
    sensor->spi_data_send[2] |= getParity(sensor->spi_data_send[2]) << 15;
    sensor->spi_data_send[3] = (uint16_t)(sensor->zero_position*AS5048a_RESOLUTION/360) >> 6;
    sensor->spi_data_send[3] |= getParity(sensor->spi_data_send[3]) << 15;
    sensor->spi_data_send[4]=AS5048A_SPI_CMD_WRITE | AS5048A_ZERO_POS_LO_REG;
    sensor->spi_data_send[4] |= getParity(sensor->spi_data_send[4]) << 15;
    sensor->spi_data_send[5] = (uint16_t)(sensor->zero_position*AS5048a_RESOLUTION/360) & 0x3F;
    sensor->spi_data_send[5] |= getParity(sensor->spi_data_send[5]) << 15;
    sensor->spi_data_send[6] = 0;
    
    /*sensor->spi_data_send[0]=AS5048A_SPI_CMD_READ | AS5048A_CLEA_RERROR_REG;
    sensor->spi_data_send[0]= 0;
    sensor->spi_data_send[2]=AS5048A_SPI_CMD_WRITE | AS5048A_ZERO_POS_HI_REG;
    sensor->spi_data_send[2] |= getParity(sensor->spi_data_send[0]) << 15;
    sensor->spi_data_send[3] = 0;
    sensor->spi_data_send[4]=AS5048A_SPI_CMD_WRITE | AS5048A_ZERO_POS_HI_REG;
    sensor->spi_data_send[4] |= getParity(sensor->spi_data_send[2]) << 15;
    sensor->spi_data_send[5] = (uint16_t)(sensor->zero_position*AS5048a_RESOLUTION/360) >> 8;
    sensor->spi_data_send[6] = AS5048A_SPI_CMD_WRITE | AS5048A_ZERO_POS_LO_REG;
    sensor->spi_data_send[6] |= getParity(sensor->spi_data_send[4]) << 15;
    sensor->spi_data_send[7] = (sensor->zero_position*AS5048a_RESOLUTION/360) & 0xFF;   
    sensor->spi_data_send[8] = 0;*/
    switch(sensor->sensor_number)
    {
        case 4:
            AS5048_CS_1_Clear();
            SPI1_WriteRead(&sensor->spi_data_send[0],14,&sensor->spi_data_received[0],14);
            sensor->variable_readed = ZPOS;
        break;
        case 5:
            AS5048_CS_2_Clear();
            SPI3_WriteRead(&sensor->spi_data_send[0],10,&sensor->spi_data_received[0],10);
            sensor->variable_readed = ZPOS;
        break;
        case 6:
            AS5048_CS_3_Clear();
            SPI4_WriteRead(&sensor->spi_data_send[0],10,&sensor->spi_data_received[0],10);
            sensor->variable_readed = ZPOS;
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
//    uart_sent_data[0] = (int32_t)(sensor_4.position*100) >>24;
//    uart_sent_data[1] = (int32_t)(sensor_4.position*100) >>16;
//    uart_sent_data[2] = (int32_t)(sensor_4.position*100) >>8;
//    uart_sent_data[3] = (int32_t)(sensor_4.position*100);
//    
//    UART2_Write(&uart_sent_data[0],4);
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