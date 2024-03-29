/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************





int main ( void )
{
    /* Initialize all modules */    
    SYS_Initialize ( NULL );
    
    
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_1,0);
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2,0);
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_3,0);
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_4,0);
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_5,0);
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_6,0);
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_7,0);
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_8,0);
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_9,0);
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_11,0);
    MCPWM_ChannelPrimaryDutySet(MCPWM_CH_12,0);
    MCPWM_Start();
    
    

    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

