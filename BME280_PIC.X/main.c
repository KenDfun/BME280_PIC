/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65
        Device            :  PIC16F15344
        Driver Version    :  2.00
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/
#include <stdint.h>
#include "mcc_generated_files/mcc.h"

uint8_t BME280_Read(uint8_t reg, uint8_t *pData);

#define BME280_ADDRESS  0x76   // slave device address
#define BME280_ADDR_ID  0xD0

/*
                         Main application
 */
void main(void)
{
    uint8_t data;
    
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    
//    D4_LAT=0b1;
//    D5_LAT=0b1;
    D6_LAT=0b1;
//    D7_LAT=0b1;

    printf("BME280 PIC start!\r\n");
    
    BME280_Read(0xD0,&data);
    printf("BME280 ID:0x%02x\r\n",data);

    
    
    while (1)
    {
        // Add your application code
        __delay_ms(500);
        D4_Toggle();
        
    }
}

uint8_t BME280_Read(uint8_t reg, uint8_t *pData)
{
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;
    static I2C1_TRANSACTION_REQUEST_BLOCK trb[2];
 
    I2C1_MasterWriteTRBBuild(&trb[0], &reg, 1, BME280_ADDRESS);
    I2C1_MasterReadTRBBuild(&trb[1], pData, 1, BME280_ADDRESS);                
    I2C1_MasterTRBInsert(2, &trb[0], &status);
 
    while(status == I2C1_MESSAGE_PENDING);      // blocking
 
    return (status == I2C1_MESSAGE_COMPLETE); 
}

/**
 End of File
*/