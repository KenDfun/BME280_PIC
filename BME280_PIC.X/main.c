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
#include "BME280_driver/bme280.h"

uint8_t BME280_Read(uint8_t reg, uint8_t *pData);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

#define BME280_ADDRESS  0x76   // slave device address
#define BME280_ADDR_ID  0xD0

/*
                         Main application
 */
void main(void)
{
    static uint8_t data[27];
    int i;
    
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
    __delay_ms(3000);
    printf("BME280 PIC start!\r\n");
    
//    BME280_Read(0xD0,&data);
    user_i2c_read(BME280_ADDRESS,BME280_ADDR_ID,data,1);
    printf("BME280 ID:0x%02x\r\n",data[0]);
    D7_LAT=0b1;    
    user_i2c_read(BME280_ADDRESS,0x88,data,26);
    printf("Calib T: ");
    for(i=0;i<26;i++){
        printf("0x%02x ",data[i]);
    }
    printf("\r\n");
    
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

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
    __delay_ms(period);
    
    return;
}
 

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;
    static I2C1_TRANSACTION_REQUEST_BLOCK trb[2];
    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */
 
    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
   
    
    if(len>26){
        printf("I2C len over\r\n");
        return -1;
    }
 
    I2C1_MasterWriteTRBBuild(&trb[0], &reg_addr, 1, dev_id);
//    for(i=0;i<len;i++){
        I2C1_MasterReadTRBBuild(&trb[1], reg_data, len, dev_id);
//    }
    I2C1_MasterTRBInsert(2, &trb[0], &status);
 
    while(status == I2C1_MESSAGE_PENDING);      // blocking
    
    rslt = status;

    return rslt;
}
 
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
 
    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */
 
    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
 
    return rslt;
}
/**
 End of File
*/