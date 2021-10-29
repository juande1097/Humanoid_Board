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
#include "config/default/peripheral/i2c/master/plib_i2c_master_common.h"
#include "config/default/peripheral/tmr1/plib_tmr1.h"
#include "config/default/peripheral/tmr1/plib_tmr1_common.h"
#include "config/default/peripheral/gpio/plib_gpio.h"
#include "config/default/peripheral/uart/plib_uart2.h"
#include "config/default/peripheral/uart/plib_uart_common.h"
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
    #define AS5600_BITS_RESOLUTION 12

    
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

    #define SPEED_CONSTANT         1200

    
    

//ENUMs
enum as5600_variable_readed {NOTING_READED, POSITION, STATUS_POSITION, CONFIG_OUTPUT_STATUS};
    
    //Structures    
    typedef struct
    {
        float                              position;               //position of the motor in degrees
        float                              old_position;           //past position of the motor in degrees
        int16_t                            turns;                  //Motor turns in integers
        float                              displacement;           //displacement in revolutions 
        float                              speed;                  //in rpm
        uint16_t                           direction;              // 0 clockwise, 1 counterclockwise
        uint16_t                           magnet_error;          //1 error , 0 good
        uint8_t                            i2c_data_received[20];  //variable were the i2c data readed is storege
        enum as5600_variable_readed        variable_readed; 
    }as5600_sensor;

    /**********Module Specific Functions**********/
    void AS5600_Initialize(void);                   //Initializes the AD4111
    void AS5600_UpdateData(void);                   //Update all the variables of the as5600_sensor struct
    void AS5600_ReadStatusPosition(void);           //Read position and status variable of the as5600_sensor 
    void AS5600_ReadPosition(void);                 //Read position variable of the as5600_sensor 
    void AS5600_UpdateDirection(uint16_t direction);//Update direction of the motor 

    /**********Peripheral call backs**********/
    void I2C1_callback(uintptr_t context);
    void Timer1_callback(uint32_t status, uintptr_t context);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */

/* *****************************************************************************
 End of File
 */
