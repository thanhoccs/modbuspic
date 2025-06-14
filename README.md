MODBUS RTU on PIC
=========

Introduction
------------

MODBUS RTU on PIC is a ISC licensed library to handle Modbus requests on PIC
(slave).
Firmware base on PIC32MZ2048EFM144 or PIC32MZ EMBEDDED CONNECTIVITY WITH FPU (EF) STARTER KIT

Features
--------

Modbus functions are supported:

* read coils (0x01)
* read discrete inputs (0x02)
* read holding registers (0x03)
* read input registers (0x04)
* write single coil (0x05)
* write single register (0x06)
* write multiple coils (0x0F)
* write multiple registers (0x10)

Example
-------

```c
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
```

Contribute
----------

