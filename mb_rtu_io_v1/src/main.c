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

#include "../mb_rtu_io_v1.X/modbus-rtu.h"
#include "../mb_rtu_io_v1.X/ioctl.h"
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    int rc = 0;
    
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    ioctl_init();
    mb_init(9600);
    
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
        
        rc = mb_loop();
        if (rc == 0) {
            // listenning
        }
        else if (rc > 0) {
            printf("MODBUS RTU exchange successful\n");
        }
        else {
            printf("MODBUS RTU exchange error, code = %d\n", rc);
        }
        ioctl_loop();
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

