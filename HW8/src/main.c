/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include <xc.h>
#include <stdio.h>
#include "accel.h"
#include "i2c_display.h"
#include "i2c_master_int.h"
#include "OLED.h"

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

void initialize_setup(void);

int main(void)
{
    /* Initialize all MPLAB Harmony modules, including application(s). */

    // INITIALIZE_SETUP IS DEFINED BY USER AT THE END OF THIS CODE

    initialize_setup();

    SYS_Initialize ( NULL );

      __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that
    // kseg0 is cacheable (0x3) or uncacheable (0x2)
    // see Chapter 2 "CPU for Devices with M4K Core"
    // of the PIC32 reference manual
    // no cache on this chip!
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210582);
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to be able to use TDI, TDO, TCK, TMS as digital
    DDPCONbits.JTAGEN = 0;

    // set up timer 2 for 100Hz
    T2CONbits.TCKPS = 0b101;   // Timer2 Type B prescaler N=1:64  PLL 40M Hz = 25 ns, 100Hz = 10ms
    // ( PR2 + 1 ) * prescaler = 10ms / 25ns = 400000   PR2 + 1 = 400000 / 64 = 6250
    PR2 = 6249;
    TMR2 = 0; // initial TMR2 count is 0
    T2CONbits.ON = 1; // turn on Timer2

    __builtin_enable_interrupts();


    // ADDED CODE BY USER TO INITIALIZE ACCELEROMETER AND SCREEN

    acc_setup();                    // initialize the accelerometor
    display_init();                 // initial the display
    display_clear();                // clear the display status


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

// set the CP0 CONFIG register to indicate that
// kseg0 is cacheable (0x3) or uncacheable (0x2)
// see Chapter 2 "CPU for Devices with M4K Core"
// of the PIC32 reference manual

void initialize_setup (void){

__builtin_disable_interrupts();
    
// no cache on this chip!

// 0 data RAM access wait states
//BMXCONbits.BMXWSDRM = 0x0;

// enable multi vector interrupts
//INTCONbits.MVEC = 0x1;

// disable JTAG to be able to use TDI, TDO, TCK, TMS as digital
//DDPCONbits.JTAGEN = 0;


// set up USER (B13) pin as input TRISB.B13 = 1
ANSELBbits.ANSB13 = 0; //Switches off Analogue and allows I/O functionality
// U1RXRbits.U1RXR = 0b0011; // set U1RX to pin B13 not needed
TRISBbits.TRISB13 = 1; // Digital Input

// set up LED1 (B7) pin as a digital output
//RPB7Rbits.RPB7R = 0b0001; // set B7 to U1TX not needed
PORTBbits.RB7 = 1;
TRISBbits.TRISB7 = 0;   // Digital Output

// set up LED2 (B15) as a digital Output
ANSELBbits.ANSB15 = 0;
TRISBbits.TRISB15 = 0;  // Set B15 or LED 

// set up LED3 (RA4) as a digital Output

TRISBbits.TRISB3 = 0;  // Set B15 or LED


// set up A0 as AN0
    ANSELAbits.ANSA0 = 1;
    AD1CON3bits.ADCS = 3;
    AD1CHSbits.CH0SA = 0;
    AD1CON1bits.ADON = 1;

    __builtin_enable_interrupts();

}