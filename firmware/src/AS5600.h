/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef AS5600_H    /* Guard against multiple inclusion */
#define AS5600_H

#include <stdbool.h>
#include <stdint.h>
#include "config/default/peripheral/i2c/master/plib_i2c1_master.h"
#include "config/default/peripheral/i2c/master/plib_i2c2_master.h"
#include "config/default/peripheral/i2c/master/plib_i2c3_master.h"
#include "config/default/peripheral/i2c/master/plib_i2c4_master.h"
#include "config/default/peripheral/i2c/master/plib_i2c_master_common.h"
#include "config/default/peripheral/tmr1/plib_tmr1.h"
#include "config/default/peripheral/tmr1/plib_tmr1_common.h"
#include "config/default/peripheral/gpio/plib_gpio.h"
#include "config/default/peripheral/uart/plib_uart2.h"
#include "config/default/peripheral/uart/plib_uart_common.h"
#include "config/default/peripheral/gpio/plib_gpio.h"
#include "config/default/peripheral/mcpwm/plib_mcpwm.h"
#include "config/default/peripheral/mcpwm/plib_mcpwm_common.h"
#include "config/default/peripheral/spi/spi_master/plib_spi1_master.h"
#include "config/default/peripheral/spi/spi_master/plib_spi_master_common.h"
#include "config/default/peripheral/spi/spi_master/plib_spi3_master.h"
#include "config/default/peripheral/spi/spi_master/plib_spi4_master.h"
#include "config/default/peripheral/coretimer/plib_coretimer.h"
/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
    #define AS5600_SLAVE_ADDRESS 0b0110110 
    #define AS5600_RESOLUTION  4096
    #define AS5048a_RESOLUTION 16384

    
    //AS5600 Register Map
    #define AS5600_ZMCO_REG        0x00
    #define AS5600_ZPOS_REG        0x01
    #define AS5600_MPOS_REG        0x03
    #define AS5600_MANG_REG        0x05
    #define AS5600_CONF_REG        0x07
    #define AS5600_RAW_ANGLE_REG   0x0C
    #define AS5600_ANGLE_REG       0x0E
    #define AS5600_STATUS_REG      0x0B
    #define AS5600_AGC_REG         0x1A
    #define AS5600_MAGNITUDE_REG   0x1B
    #define AS5600_BURN_REG        0xFF
    
    ////AS5048A Register Map
    #define AS5048A_NOP_REG           0x0000
    #define AS5048A_CLEA_RERROR_REG   0x0001
    #define AS5048A_PROG_CONTR_REG    0x0003
    #define AS5048A_ZERO_POS_HI_REG   0x0016
    #define AS5048A_ZERO_POS_LO_REG   0x0017
    #define AS5048A_DIAGNOSTICS_REG   0x3FFD
    #define AS5048A_MAGNITUDE_REG     0x3FFE
    #define AS5048A_ANGLE_REG         0x3FFF
    #define AS5048A_SPI_CMD_READ      0x4000
    #define AS5048A_SPI_CMD_WRITE     0x0000


    #define SPEED_CONSTANT         1200
    #define TURN_DEGREES           360

    
    

//ENUMs
enum sensor_variable_readed  {NOTING_READED, POSITION, STATUS_POSITION, CONFIG_OUTPUT_STATUS,CLEARFLAG_POSITION, ZPOS, MPOS};

    
    //Structures    
    typedef struct
    {
        uint8_t                            sensor_number;          //Position number of the sensor
        float                              position;               //position of the motor in degrees
        float                              old_position;           //old position of the motor in degrees
        int16_t                            turns;                  //Motor turns in integers
        float                              displacement;           //displacement in revolutions 
        float                              speed;                  //in rpm
        uint16_t                           direction;              // 0 clockwise, 1 counterclockwise
        uint16_t                           magnet_error;          //1 error , 0 good
        uint8_t                            i2c_data_received[20];  //variable were the i2c data readed is storege
        uint8_t                            i2c_data_send[20];
        float                              zero_position;
        enum sensor_variable_readed        variable_readed; 
    }as5600_sensor;
    
    typedef struct
    {
        uint8_t                            sensor_number;          //Position number of the sensor
        float                              position;               //position of the motor in degrees
        float                              old_position;           //old position of the motor in degrees
        int16_t                            turns;                  //Motor turns in integers
        float                              displacement;           //displacement in revolutions 
        float                              speed;                  //in rpm
        uint16_t                           direction;              // 0 clockwise, 1 counterclockwise
        uint16_t                           flag_error;             //
        uint16_t                           spi_data_received[20];  //variable were the i2c data readed is storege
        uint16_t                           spi_data_send[20];
        float                           zero_position;
        enum sensor_variable_readed        variable_readed; 
    }as5048a_sensor;

    /**********Module Specific Functions**********/
    void AS5600_Initialize(void);                   //Initializes the AD4111
    void AS5600_SensorInit(as5600_sensor *sensor, uint8_t sensor_num, float zero_pos);
    void AS5600_UpdateData(as5600_sensor *sensor);                   //Update all the variables of the as5600_sensor struct
    void AS5600_ReadStatusPosition(as5600_sensor *sensor, uint8_t channel) ;           //Read position and status variable of the as5600_sensor 
    void AS5600_ReadPosition(as5600_sensor *sensor);                 //Read position variable of the as5600_sensor 
    void AS5600_Write_ZPOS(as5600_sensor *sensor);
    void AS5600_UpdateSerialData (void);
    void AS5048A_SensorInit(as5048a_sensor *sensor, uint8_t sensor_num, float zero_pos);
    void AS5048A_WriteZPOS(as5048a_sensor *sensor);
    void AS5048A_UpdateSerialData (void);
    void AS5048A_ReadStatusPosition(as5048a_sensor *sensor, uint8_t channel);
    void AS5048A_ReadPosition(as5048a_sensor *sensor);
    void AS5048A_UpdateData(as5048a_sensor *sensor);
    bool getParity(uint16_t data);

    /**********Peripheral call backs**********/
    void I2C1_callback(uintptr_t context);
    void I2C2_callback(uintptr_t context);
    void I2C3_callback(uintptr_t context);
    void I2C4_callback(uintptr_t context);
    void Timer1_callback(uint32_t status, uintptr_t context);
    


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */

/* *****************************************************************************
 End of File
 */
