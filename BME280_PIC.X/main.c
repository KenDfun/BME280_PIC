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
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "mcc_generated_files/mcc.h"
#include "bme280_mac.h"

//uint8_t BME280_Read(uint8_t reg, uint8_t *pData);
I2C1_MESSAGE_STATUS user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
I2C1_MESSAGE_STATUS user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
//void user_delay_ms(uint32_t period);


void get_ID(void);
void soft_reset(void);
void get_calib_data(BME280_CALIB_DATA *cal_data);
void set_sensor_setting(BME280_SETTINGS *settings);
void set_sensor_mode(uint8_t sensor_mode);
void get_sensor_data(BME280_DATA *sensor_data,BME280_UNCOMP_DATA *sensor_uncomp_data);
void sensor_sleep_check(void);

#define BME280_ADDRESS  BME280_I2C_ADDR_PRIM   // slave device address

static uint8_t WorkRegData[BME280_TEMP_PRESS_CALIB_DATA_LEN];
static BME280_CALIB_DATA CalibData;
static BME280_SETTINGS Settings;
static BME280_DATA SensorData;
static BME280_UNCOMP_DATA SensorUncompData;

/*
 Main application
 */
void main(void)
{
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
    __delay_ms(3000); // for debugger reset
    
//    printf("\r\n\r\n\r\nBME280 PIC start!\r\n");
    
    get_ID();
    soft_reset();
    get_calib_data(&CalibData);
    
    /* Recommended mode of operation: Indoor navigation */
    Settings.osr_h = BME280_OVERSAMPLING_1X;
    Settings.osr_p = BME280_OVERSAMPLING_16X;
    Settings.osr_t = BME280_OVERSAMPLING_16X;
//    Settings.filter = BME280_FILTER_COEFF_16;
    Settings.filter = BME280_FILTER_COEFF_OFF;
//    Settings.standby_time = BME280_STANDBY_TIME_62_5_MS;
    Settings.standby_time = BME280_STANDBY_TIME_1_MS;
    set_sensor_setting(&Settings);
    set_sensor_mode(BME280_NORMAL_MODE);
    
    D7_LAT=0b1;    
    
    while (1)
    {
        // Add your application code
//        set_sensor_mode(BME280_FORCED_MODE);
        __delay_ms(40);
        get_sensor_data(&SensorData,&SensorUncompData);
        printf("%ld, %ld, %ld             \r",SensorData.temperature, SensorData.pressure, SensorData.humidity);
        D4_Toggle();
        __delay_ms(2000);        
    }
}

void get_ID(void)
{
    user_i2c_read(BME280_ADDRESS,BME280_CHIP_ID_ADDR,&WorkRegData,1);
//    printf("BME280 ID:0x%02x\r\n",WorkRegData[0]);    
}

void soft_reset(void)
{
    uint8_t soft_rst_cmd = 0xB6;
    
    user_i2c_write(BME280_ADDRESS,BME280_RESET_ADDR,&soft_rst_cmd,1);
    __delay_ms(2);
    
//    printf("Soft Reset!\r\n");
}

static void parse_temp_press_calib_data(BME280_CALIB_DATA *calib_data, const uint8_t *reg_data);
static void parse_humidity_calib_data(BME280_CALIB_DATA *calib_data, const uint8_t *reg_data);

void get_calib_data(BME280_CALIB_DATA *cal_data)
{
//    printf("Get Temp Press Calib data\r\n");
    user_i2c_read(BME280_ADDRESS,BME280_TEMP_PRESS_CALIB_DATA_ADDR,&WorkRegData,BME280_TEMP_PRESS_CALIB_DATA_LEN);
    parse_temp_press_calib_data(cal_data,WorkRegData);
    
//    printf("Get Humidity Calib data\r\n");
    user_i2c_read(BME280_ADDRESS,BME280_HUMIDITY_CALIB_DATA_ADDR,&WorkRegData,BME280_HUMIDITY_CALIB_DATA_LEN);
    parse_humidity_calib_data(cal_data,WorkRegData);
}

void set_osr_settings(BME280_SETTINGS *settings);
void set_filter_standby_settings(BME280_SETTINGS *settings);

void set_sensor_setting(BME280_SETTINGS *settings)
{
//    printf("Sensor Setting\r\n");
    
    /* if mode is not sleep mode then soft rest. */
    sensor_sleep_check();
    
    set_osr_settings(settings);
    set_filter_standby_settings(settings);
}

void set_osr_settings(BME280_SETTINGS *settings)
{   
	/* Write the humidity control value in the register */
//    printf("Humidity control value write: 0x%02d\r\n",settings->osr_h);
    WorkRegData[0] = settings->osr_h & BME280_CTRL_HUM_MSK;
    user_i2c_write(BME280_ADDRESS,BME280_CTRL_HUM_ADDR, &WorkRegData, 1);
    
#if 0    
	/* Humidity related changes will be only effective after a
     write operation to ctrl_meas register */
    user_i2c_read(BME280_ADDRESS,BME280_CTRL_MEAS_ADDR, &WorkRegData, 1);
    user_i2c_write(BME280_ADDRESS,BME280_CTRL_MEAS_ADDR, &WorkRegData, 1);
#endif
    
//    printf("MEAS control value write: PRESS 0x%0d, TEMP 0x%02d\r\n",settings->osr_p,settings->osr_t);
    user_i2c_read(BME280_ADDRESS,BME280_CTRL_MEAS_ADDR, &WorkRegData, 1);
//    BME280_SET_BITS(WorkRegData[0], BME280_CTRL_PRESS, settings->osr_p);
    WorkRegData[0] = (WorkRegData[0] & (~BME280_CTRL_PRESS_MSK)) | ((settings->osr_p << BME280_CTRL_PRESS_POS) & BME280_CTRL_PRESS_MSK);
//    BME280_SET_BITS(WorkRegData[0], BME280_CTRL_TEMP, settings->osr_t);
    WorkRegData[0] = (WorkRegData[0] & (~BME280_CTRL_TEMP_MSK)) | ((settings->osr_t << BME280_CTRL_TEMP_POS) & BME280_CTRL_TEMP_MSK);
    user_i2c_write(BME280_ADDRESS,BME280_CTRL_MEAS_ADDR, &WorkRegData, 1);
}

void set_filter_standby_settings(BME280_SETTINGS *settings)
{
//    printf("Config control value write: FILTER 0x%02d, STANBY 0x%02d\r\n",settings->filter,settings->standby_time);
    user_i2c_read(BME280_ADDRESS,BME280_CONFIG_ADDR, &WorkRegData, 1);
//    BME280_SET_BITS(WorkRegData[0], BME280_FILTER, settings->filter);
    WorkRegData[0] = (WorkRegData[0] & (~BME280_FILTER_MSK)) | ((settings->filter << BME280_FILTER_POS) & BME280_FILTER_MSK);
//    BME280_SET_BITS(WorkRegData[0], BME280_STANDBY, settings->standby_time);
    WorkRegData[0] = (WorkRegData[0] & (~BME280_STANDBY_MSK)) | ((settings->standby_time << BME280_STANDBY_POS) & BME280_STANDBY_MSK);
    user_i2c_write(BME280_ADDRESS,BME280_CONFIG_ADDR, &WorkRegData, 1);
}

void set_sensor_mode(uint8_t sensor_mode)
{
//    printf("Set sensor mode: 0x%02x\r\n",sensor_mode);
    
    /* if mode is not sleep mode then soft rest. */
    sensor_sleep_check();
    
    user_i2c_read(BME280_ADDRESS,BME280_PWR_CTRL_ADDR, &WorkRegData, 1);    
    WorkRegData[0] = (WorkRegData[0]  & (~BME280_SENSOR_MODE_MSK)) | (sensor_mode & BME280_SENSOR_MODE_MSK);
    user_i2c_write(BME280_ADDRESS,BME280_PWR_CTRL_ADDR, &WorkRegData, 1);
}


static void parse_temp_press_calib_data(BME280_CALIB_DATA *calib_data, const uint8_t *reg_data)
{
	calib_data->dig_T1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data->dig_T2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
	calib_data->dig_T3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
	calib_data->dig_P1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
	calib_data->dig_P2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
	calib_data->dig_P3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
	calib_data->dig_P4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
	calib_data->dig_P5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
	calib_data->dig_P6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
	calib_data->dig_P7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
	calib_data->dig_P8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
	calib_data->dig_P9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
	calib_data->dig_H1 = reg_data[25];
}

static void parse_humidity_calib_data(BME280_CALIB_DATA *calib_data, const uint8_t *reg_data)
{
	int16_t dig_H4_lsb;
	int16_t dig_H4_msb;
	int16_t dig_H5_lsb;
	int16_t dig_H5_msb;
    
	calib_data->dig_H2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data->dig_H3 = reg_data[2];
    
	dig_H4_msb = (int16_t)(int8_t)reg_data[3] * 16;
	dig_H4_lsb = (int16_t)(reg_data[4] & 0x0F);
	calib_data->dig_H4 = dig_H4_msb | dig_H4_lsb;
    
	dig_H5_msb = (int16_t)(int8_t)reg_data[5] * 16;
	dig_H5_lsb = (int16_t)(reg_data[4] >> 4);
	calib_data->dig_H5 = dig_H5_msb | dig_H5_lsb;
	calib_data->dig_H6 = (int8_t)reg_data[6];
}

void parse_sensor_data(const uint8_t *reg_data, BME280_UNCOMP_DATA *uncomp_data)
{
	/* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	/* Store the parsed register values for pressure data */
	data_msb = (uint32_t)reg_data[0] << 12;
	data_lsb = (uint32_t)reg_data[1] << 4;
	data_xlsb = (uint32_t)reg_data[2] >> 4;
	uncomp_data->pressure = data_msb | data_lsb | data_xlsb;
//    printf("Pressure uncomp: 0x%08lx\r\n",uncomp_data->pressure);

	/* Store the parsed register values for temperature data */
	data_msb = (uint32_t)reg_data[3] << 12;
	data_lsb = (uint32_t)reg_data[4] << 4;
	data_xlsb = (uint32_t)reg_data[5] >> 4;
	uncomp_data->temperature = data_msb | data_lsb | data_xlsb;
//    printf("Temperature uncomp: 0x%08lx\r\n",uncomp_data->temperature);
	
    /* Store the parsed register values for temperature data */
	data_lsb = (uint32_t)reg_data[6] << 8;
	data_msb = (uint32_t)reg_data[7];
	uncomp_data->humidity = data_msb | data_lsb;
//    printf("Humidity uncomp: 0x%08lx\r\n",uncomp_data->humidity);
}

static int32_t compensate_temperature(const BME280_UNCOMP_DATA *uncomp_data,	BME280_CALIB_DATA *calib_data);
static uint32_t compensate_pressure(const BME280_UNCOMP_DATA *uncomp_data, const BME280_CALIB_DATA *calib_data);
static uint32_t compensate_humidity(const BME280_UNCOMP_DATA *uncomp_data,	const BME280_CALIB_DATA *calib_data);

void compensate_data(const BME280_UNCOMP_DATA *uncomp_data, BME280_DATA *comp_data, BME280_CALIB_DATA *calib_data)
{
	if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL)) {
		/* Initialize to zero */
		comp_data->temperature = 0;
		comp_data->pressure = 0;
		comp_data->humidity = 0;
        /* Compensate the temperature data */
        comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
        /* Compensate the pressure data */
        comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
        /* Compensate the humidity data */
        comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
	}
    else {
//		printf("Have Bug in this program!!!!\r\n");
	}
}

static int32_t compensate_temperature(const BME280_UNCOMP_DATA *uncomp_data,	BME280_CALIB_DATA *calib_data)
{
	int32_t var1;
	int32_t var2;
	int32_t temperature;

    var1 = (((uncomp_data->temperature >> 3) - ((int32_t)calib_data->dig_T1 << 1)) * ((int32_t)calib_data->dig_T2 )) >> 11;
    var2 = (((   ((uncomp_data->temperature >> 4) - ((int32_t)calib_data->dig_T1)) * ((uncomp_data->temperature >> 4)-((int32_t)calib_data->dig_T1))  )>>12) * ((int32_t)calib_data->dig_T3)) >> 14 ;
    calib_data->t_fine = var1 + var2;
    temperature = (calib_data->t_fine * 5 + 128) >> 8;
    return temperature;
}

static uint32_t compensate_pressure(const BME280_UNCOMP_DATA *uncomp_data, const BME280_CALIB_DATA *calib_data)
{
    int32_t var1;
	int32_t var2;
	uint32_t pressure;
    
    var1 = (((int32_t)calib_data->t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_data->dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib_data->dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)calib_data->dig_P4) << 16);
    var1 = (((calib_data->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)calib_data->dig_P2) * var1) >> 1)) >> 18;
    var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_P1)) >> 15;
    
    if(var1 == 0){
        return 0;
    }
    
    pressure = ((uint32_t)((((uint32_t)1048576) - uncomp_data->pressure) - (uint32_t)(var2 >> 12))) * 3125;
    if (pressure < 0x80000000){
        pressure = (pressure << 1) / ((uint32_t)var1);
    }
    else{
			pressure = (pressure / (uint32_t)var1) << 1;      
    }
    
    var1 = (((int32_t)calib_data->dig_P9) * ((int32_t)(((pressure >> 3) * (pressure >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(pressure >> 2)) * ((int32_t)calib_data->dig_P8)) >> 13;
    pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_P7) >> 4));    

    return pressure;
}
static uint32_t compensate_humidity(const BME280_UNCOMP_DATA *uncomp_data,	const BME280_CALIB_DATA *calib_data)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	uint32_t humidity;


	var1 = calib_data->t_fine - ((int32_t)76800);
	var2 = (int32_t)(uncomp_data->humidity << 14);
	var3 = (int32_t)(((int32_t)calib_data->dig_H4) << 20);
	var4 = ((int32_t)calib_data->dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) >> 15;
	var2 = (var1 * ((int32_t)calib_data->dig_H6)) >> 10;
	var3 = (var1 * ((int32_t)calib_data->dig_H3)) >> 11;
	var4 = ((var2 * (var3 + (int32_t)32768)) >> 10) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)calib_data->dig_H2)) + 8192) >> 14;
	var3 = var5 * var2;
	var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
	var5 = var3 - ((var4 * ((int32_t)calib_data->dig_H1)) >> 4);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	humidity = (uint32_t)(var5 >> 12);

	return humidity;
}
#if 0
static int32_t compensate_temperature(const BME280_UNCOMP_DATA *uncomp_data,	BME280_CALIB_DATA *calib_data)
{
	int32_t var1;
	int32_t var2;
	int32_t temperature;
	int32_t temperature_min = -4000;
	int32_t temperature_max = 8500;

//	var1 = (int32_t)((uncomp_data->temperature / 8) - ((int32_t)calib_data->dig_T1 * 2));
	var1 = (var1 * ((int32_t)calib_data->dig_T2)) / 2048;
#if 0
	var2 = (int32_t)((uncomp_data->temperature / 16) - ((int32_t)calib_data->dig_T1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_T3)) / 16384;
	calib_data->t_fine = var1 + var2;
	temperature = (calib_data->t_fine * 5 + 128) / 256;
#endif
    temperature = var1;
    
	if (temperature < temperature_min)
		temperature = temperature_min;
	else if (temperature > temperature_max)
		temperature = temperature_max;

	return temperature;
}
#endif
#if 0
static uint32_t compensate_pressure(const BME280_UNCOMP_DATA *uncomp_data, const BME280_CALIB_DATA *calib_data)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	uint32_t var5;
	uint32_t pressure;
	uint32_t pressure_min = 30000;
	uint32_t pressure_max = 110000;

	var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_P6);
	var2 = var2 + ((var1 * ((int32_t)calib_data->dig_P5)) * 2);
	var2 = (var2 / 4) + (((int32_t)calib_data->dig_P4) * 65536);
	var3 = (calib_data->dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int32_t)calib_data->dig_P2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_P1)) / 32768;
	 /* avoid exception caused by division by zero */
	if (var1) {
		var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pressure;
		pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
		if (pressure < 0x80000000)
			pressure = (pressure << 1) / ((uint32_t)var1);
		else
			pressure = (pressure / (uint32_t)var1) * 2;

		var1 = (((int32_t)calib_data->dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
		var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_P8)) / 8192;
		pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_P7) / 16));

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else {
		pressure = pressure_min;
	}

	return pressure;
}
static uint32_t compensate_humidity(const BME280_UNCOMP_DATA *uncomp_data,	const BME280_CALIB_DATA *calib_data)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	uint32_t humidity;
	uint32_t humidity_max = 100000;

	var1 = calib_data->t_fine - ((int32_t)76800);
	var2 = (int32_t)(uncomp_data->humidity * 16384);
	var3 = (int32_t)(((int32_t)calib_data->dig_H4) * 1048576);
	var4 = ((int32_t)calib_data->dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)calib_data->dig_H6)) / 1024;
	var3 = (var1 * ((int32_t)calib_data->dig_H3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)calib_data->dig_H2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)calib_data->dig_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	humidity = (uint32_t)(var5 / 4096);

	if (humidity > humidity_max)
		humidity = humidity_max;

	return humidity;
}
#endif
#if 0
void parse_device_settings(uint8_t *reg_data, BME280_SETTINGS *settings)
{
	settings->osr_h = BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM);
	settings->osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS);
	settings->osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP);
	settings->filter = BME280_GET_BITS(reg_data[3], BME280_FILTER);
	settings->standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY);    
}
#endif

void parse_sensor_data(const uint8_t *reg_data, BME280_UNCOMP_DATA *uncomp_data);
void compensate_data(const BME280_UNCOMP_DATA *uncomp_data, BME280_DATA *comp_data, BME280_CALIB_DATA *calib_data);

void get_sensor_data(BME280_DATA *sensor_data,BME280_UNCOMP_DATA *sensor_uncomp_data)
{
//    printf("get sensor data\r\n");
    user_i2c_read(BME280_ADDRESS,BME280_DATA_ADDR, &WorkRegData, BME280_P_T_H_DATA_LEN);    
    parse_sensor_data(&WorkRegData,sensor_uncomp_data);
    compensate_data(sensor_uncomp_data, sensor_data, &CalibData);
}

void get_sensor_mode(uint8_t *psensor_mode)
{
    user_i2c_read(BME280_ADDRESS,BME280_CTRL_MEAS_ADDR, &WorkRegData, 1);
    *psensor_mode = WorkRegData[0]  & BME280_SENSOR_MODE_MSK;
//    printf("Get Sensor Mode: 0x%02x\r\n",*psensor_mode);
}

void get_sensor_mode(uint8_t *psensor_mode);
/* if mode is not sleep mode then soft rest. */
void sensor_sleep_check(void)
{
    uint8_t mode;
    
//    printf("Sleep Check\r\n");
    get_sensor_mode(&mode);
    if(mode !=BME280_SLEEP_MODE){
        soft_reset();
    }    
}





static I2C1_TRANSACTION_REQUEST_BLOCK Trb[2];

I2C1_MESSAGE_STATUS user_i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;
    
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
    
  
#ifdef I2C_DEBUG
    if(len>26){
        printf("I2C len over\r\n");
        return -1;
    }
#endif
    
    I2C1_MasterWriteTRBBuild(&Trb[0], &reg_addr, 1, i2c_addr); /* write to read for subaddr */
    I2C1_MasterReadTRBBuild(&Trb[1], reg_data, len, i2c_addr); /* read a number of len of data */
    I2C1_MasterTRBInsert(2, &Trb[0], &status); /* transaction start */
    
    while(status == I2C1_MESSAGE_PENDING);      // blocking
  
#ifdef I2C_DEBUG
    {
    int i;
    printf("I2C read : ADDR 0x%02x, SUBADDR 0x%02x\r\n",i2c_addr,reg_addr);
    for(i=0;i<len;i++){
        printf("0x%02x ",reg_data[i]);
//        if((i!=0)&&((i%7)==0)){
        if(((i+1)%8)==0){
            printf("\r\n");
        }
    }
    if((i%8)!=0){    
        printf("\r\n"); 
    }
    printf("Result: 0x%02x\r\n\r\n",status);
    }
#endif
    
    
    return status;
}



I2C1_MESSAGE_STATUS user_i2c_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;
    static uint8_t workreg[BME280_TEMP_PRESS_CALIB_DATA_LEN+1];
    
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
    
    workreg[0]=reg_addr;
    memcpy(&workreg[1],reg_data,len);
  
    I2C1_MasterWriteTRBBuild(&Trb[0], &workreg[0], len+1, i2c_addr); /* write a number of len of data */
    I2C1_MasterTRBInsert(1, &Trb[0], &status); /* transaction start */
    
    while(status == I2C1_MESSAGE_PENDING);      // blocking
    
#ifdef I2C_DEBUG    
    {
    int i;
    printf("I2C write : ADDR 0x%02x, SUBADDR 0x%02x\r\n",i2c_addr,reg_addr);
    for(i=0;i<len;i++){
        printf("0x%02x ",reg_data[i]);
        if((i!=0)&&(i%7)==0){
            printf("\r\n");
        }
    }
    if((i%7)!=0){    
        printf("\r\n"); 
    }
    printf("Result: 0x%02x\r\n\r\n",status);
    }
#endif
    
    return status;
}

#if 0
void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
    __delay_ms(period);
    
    return;
}
#endif

#if 0

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



void put_device_to_sleep(void)
{
    
    /* setup Device Settings */
    user_i2c_read(BME280_ADDRESS,BME280_CTRL_HUM_ADDR,&WorkRegData,4);
    parse_device_settings(&WorkRegData, &tempSettings);
    
    soft_reset();
    
    /* reset device settings */
    set_osr_settings(BME280_ALL_SETTINGS_SEL, &Settings);   
    set_filter_standby_settings(BME280_ALL_SETTINGS_SEL, &Settings);
}
#endif

/**
 End of File
 */