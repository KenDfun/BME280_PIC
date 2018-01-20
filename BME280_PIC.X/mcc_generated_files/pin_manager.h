/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.26.1
        Device            :  PIC16F15344
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
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

*/


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set D5 aliases
#define D5_TRIS               TRISAbits.TRISA1
#define D5_LAT                LATAbits.LATA1
#define D5_PORT               PORTAbits.RA1
#define D5_WPU                WPUAbits.WPUA1
#define D5_OD                ODCONAbits.ODCA1
#define D5_ANS                ANSELAbits.ANSA1
#define D5_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define D5_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define D5_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define D5_GetValue()           PORTAbits.RA1
#define D5_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define D5_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define D5_SetPullup()      do { WPUAbits.WPUA1 = 1; } while(0)
#define D5_ResetPullup()    do { WPUAbits.WPUA1 = 0; } while(0)
#define D5_SetPushPull()    do { ODCONAbits.ODCA1 = 1; } while(0)
#define D5_SetOpenDrain()   do { ODCONAbits.ODCA1 = 0; } while(0)
#define D5_SetAnalogMode()  do { ANSELAbits.ANSA1 = 1; } while(0)
#define D5_SetDigitalMode() do { ANSELAbits.ANSA1 = 0; } while(0)

// get/set D6 aliases
#define D6_TRIS               TRISAbits.TRISA2
#define D6_LAT                LATAbits.LATA2
#define D6_PORT               PORTAbits.RA2
#define D6_WPU                WPUAbits.WPUA2
#define D6_OD                ODCONAbits.ODCA2
#define D6_ANS                ANSELAbits.ANSA2
#define D6_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define D6_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define D6_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define D6_GetValue()           PORTAbits.RA2
#define D6_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define D6_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define D6_SetPullup()      do { WPUAbits.WPUA2 = 1; } while(0)
#define D6_ResetPullup()    do { WPUAbits.WPUA2 = 0; } while(0)
#define D6_SetPushPull()    do { ODCONAbits.ODCA2 = 1; } while(0)
#define D6_SetOpenDrain()   do { ODCONAbits.ODCA2 = 0; } while(0)
#define D6_SetAnalogMode()  do { ANSELAbits.ANSA2 = 1; } while(0)
#define D6_SetDigitalMode() do { ANSELAbits.ANSA2 = 0; } while(0)

// get/set D4 aliases
#define D4_TRIS               TRISAbits.TRISA5
#define D4_LAT                LATAbits.LATA5
#define D4_PORT               PORTAbits.RA5
#define D4_WPU                WPUAbits.WPUA5
#define D4_OD                ODCONAbits.ODCA5
#define D4_ANS                ANSELAbits.ANSA5
#define D4_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define D4_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define D4_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define D4_GetValue()           PORTAbits.RA5
#define D4_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define D4_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define D4_SetPullup()      do { WPUAbits.WPUA5 = 1; } while(0)
#define D4_ResetPullup()    do { WPUAbits.WPUA5 = 0; } while(0)
#define D4_SetPushPull()    do { ODCONAbits.ODCA5 = 1; } while(0)
#define D4_SetOpenDrain()   do { ODCONAbits.ODCA5 = 0; } while(0)
#define D4_SetAnalogMode()  do { ANSELAbits.ANSA5 = 1; } while(0)
#define D4_SetDigitalMode() do { ANSELAbits.ANSA5 = 0; } while(0)

// get/set SDA1 aliases
#define SDA1_TRIS               TRISBbits.TRISB4
#define SDA1_LAT                LATBbits.LATB4
#define SDA1_PORT               PORTBbits.RB4
#define SDA1_WPU                WPUBbits.WPUB4
#define SDA1_OD                ODCONBbits.ODCB4
#define SDA1_ANS                ANSELBbits.ANSB4
#define SDA1_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define SDA1_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define SDA1_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define SDA1_GetValue()           PORTBbits.RB4
#define SDA1_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define SDA1_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define SDA1_SetPullup()      do { WPUBbits.WPUB4 = 1; } while(0)
#define SDA1_ResetPullup()    do { WPUBbits.WPUB4 = 0; } while(0)
#define SDA1_SetPushPull()    do { ODCONBbits.ODCB4 = 1; } while(0)
#define SDA1_SetOpenDrain()   do { ODCONBbits.ODCB4 = 0; } while(0)
#define SDA1_SetAnalogMode()  do { ANSELBbits.ANSB4 = 1; } while(0)
#define SDA1_SetDigitalMode() do { ANSELBbits.ANSB4 = 0; } while(0)

// get/set RB5 procedures
#define RB5_SetHigh()    do { LATBbits.LATB5 = 1; } while(0)
#define RB5_SetLow()   do { LATBbits.LATB5 = 0; } while(0)
#define RB5_Toggle()   do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define RB5_GetValue()         PORTBbits.RB5
#define RB5_SetDigitalInput()   do { TRISBbits.TRISB5 = 1; } while(0)
#define RB5_SetDigitalOutput()  do { TRISBbits.TRISB5 = 0; } while(0)

// get/set SCL1 aliases
#define SCL1_TRIS               TRISBbits.TRISB6
#define SCL1_LAT                LATBbits.LATB6
#define SCL1_PORT               PORTBbits.RB6
#define SCL1_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define SCL1_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define SCL1_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define SCL1_GetValue()           PORTBbits.RB6
#define SCL1_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define SCL1_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)

// get/set RB7 procedures
#define RB7_SetHigh()    do { LATBbits.LATB7 = 1; } while(0)
#define RB7_SetLow()   do { LATBbits.LATB7 = 0; } while(0)
#define RB7_Toggle()   do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define RB7_GetValue()         PORTBbits.RB7
#define RB7_SetDigitalInput()   do { TRISBbits.TRISB7 = 1; } while(0)
#define RB7_SetDigitalOutput()  do { TRISBbits.TRISB7 = 0; } while(0)

// get/set POT1 aliases
#define POT1_TRIS               TRISCbits.TRISC0
#define POT1_LAT                LATCbits.LATC0
#define POT1_PORT               PORTCbits.RC0
#define POT1_WPU                WPUCbits.WPUC0
#define POT1_OD                ODCONCbits.ODCC0
#define POT1_ANS                ANSELCbits.ANSC0
#define POT1_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define POT1_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define POT1_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define POT1_GetValue()           PORTCbits.RC0
#define POT1_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define POT1_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define POT1_SetPullup()      do { WPUCbits.WPUC0 = 1; } while(0)
#define POT1_ResetPullup()    do { WPUCbits.WPUC0 = 0; } while(0)
#define POT1_SetPushPull()    do { ODCONCbits.ODCC0 = 1; } while(0)
#define POT1_SetOpenDrain()   do { ODCONCbits.ODCC0 = 0; } while(0)
#define POT1_SetAnalogMode()  do { ANSELCbits.ANSC0 = 1; } while(0)
#define POT1_SetDigitalMode() do { ANSELCbits.ANSC0 = 0; } while(0)

// get/set Touch aliases
#define Touch_TRIS               TRISCbits.TRISC1
#define Touch_LAT                LATCbits.LATC1
#define Touch_PORT               PORTCbits.RC1
#define Touch_WPU                WPUCbits.WPUC1
#define Touch_OD                ODCONCbits.ODCC1
#define Touch_ANS                ANSELCbits.ANSC1
#define Touch_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define Touch_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define Touch_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define Touch_GetValue()           PORTCbits.RC1
#define Touch_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define Touch_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define Touch_SetPullup()      do { WPUCbits.WPUC1 = 1; } while(0)
#define Touch_ResetPullup()    do { WPUCbits.WPUC1 = 0; } while(0)
#define Touch_SetPushPull()    do { ODCONCbits.ODCC1 = 1; } while(0)
#define Touch_SetOpenDrain()   do { ODCONCbits.ODCC1 = 0; } while(0)
#define Touch_SetAnalogMode()  do { ANSELCbits.ANSC1 = 1; } while(0)
#define Touch_SetDigitalMode() do { ANSELCbits.ANSC1 = 0; } while(0)

// get/set TACT_S1 aliases
#define TACT_S1_TRIS               TRISCbits.TRISC4
#define TACT_S1_LAT                LATCbits.LATC4
#define TACT_S1_PORT               PORTCbits.RC4
#define TACT_S1_WPU                WPUCbits.WPUC4
#define TACT_S1_OD                ODCONCbits.ODCC4
#define TACT_S1_ANS                ANSELCbits.ANSC4
#define TACT_S1_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define TACT_S1_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define TACT_S1_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define TACT_S1_GetValue()           PORTCbits.RC4
#define TACT_S1_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define TACT_S1_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define TACT_S1_SetPullup()      do { WPUCbits.WPUC4 = 1; } while(0)
#define TACT_S1_ResetPullup()    do { WPUCbits.WPUC4 = 0; } while(0)
#define TACT_S1_SetPushPull()    do { ODCONCbits.ODCC4 = 1; } while(0)
#define TACT_S1_SetOpenDrain()   do { ODCONCbits.ODCC4 = 0; } while(0)
#define TACT_S1_SetAnalogMode()  do { ANSELCbits.ANSC4 = 1; } while(0)
#define TACT_S1_SetDigitalMode() do { ANSELCbits.ANSC4 = 0; } while(0)

// get/set D7 aliases
#define D7_TRIS               TRISCbits.TRISC5
#define D7_LAT                LATCbits.LATC5
#define D7_PORT               PORTCbits.RC5
#define D7_WPU                WPUCbits.WPUC5
#define D7_OD                ODCONCbits.ODCC5
#define D7_ANS                ANSELCbits.ANSC5
#define D7_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define D7_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define D7_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define D7_GetValue()           PORTCbits.RC5
#define D7_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define D7_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define D7_SetPullup()      do { WPUCbits.WPUC5 = 1; } while(0)
#define D7_ResetPullup()    do { WPUCbits.WPUC5 = 0; } while(0)
#define D7_SetPushPull()    do { ODCONCbits.ODCC5 = 1; } while(0)
#define D7_SetOpenDrain()   do { ODCONCbits.ODCC5 = 0; } while(0)
#define D7_SetAnalogMode()  do { ANSELCbits.ANSC5 = 1; } while(0)
#define D7_SetDigitalMode() do { ANSELCbits.ANSC5 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/