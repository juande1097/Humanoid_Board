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

#ifndef CONTROL_H    /* Guard against multiple inclusion */
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "AS5600.h"
#include "config/default/peripheral/mcpwm/plib_mcpwm.h"
#include "config/default/peripheral/mcpwm/plib_mcpwm_common.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


#define DUTY_MAX_PERIOD PTPER

    
    enum control_type {NONE, P, PI, PD, PID};
    
    //Structures    
    typedef struct
    {
        enum control_type        control_type_selected;  //control algorithm to implement
        float                    period;
        float                    duty_cycle;             //duty generated from control
        float*                   position;               //position of the motor in degrees
        float*                   old_position;           //past position of the motor in degrees
        float                    speed;                  //speed in rpm
        float                    acceleration;           //acceleration in rpm2
        uint16_t                 direction;              // 0 clockwise, 1 counterclockwise
        float                    ref;
        

    }control_data;

    
    /**********Control Specific Functions**********/
    void Control_initialize(void);
    void Control_PID(float kp, float ki, float kd);
    void Control_UpdateSpeedAcceleration(void);
    void Control_UpdateDirection(float pwm_duty);
    void Control_SetDutyPeriod(float pwm_duty);

    

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
