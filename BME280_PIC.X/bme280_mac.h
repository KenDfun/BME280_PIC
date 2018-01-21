/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef BNE280_MAC_H
#define	BNE280_MAC_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

/********************************************************/
/*! @name		Common macros		        */
/********************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)       (int8_t)(x)
#define UINT8_C(x)      (uint8_t)(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)      (int16_t)(x)
#define UINT16_C(x)     (uint16_t)(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)      (int32_t)(x)
#define UINT32_C(x)     (uint32_t)(x)
#endif

#if 0
#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)      (int64_t)(x)
#define UINT64_C(x)     (uint64_t)(x)
#endif
#endif
/**@}*/


/********************************************************/

#ifndef BME280_FLOAT_ENABLE
/* #define BME280_FLOAT_ENABLE */
#endif

#ifndef BME280_FLOAT_ENABLE
#ifndef BME280_64BIT_ENABLE
//#define BME280_64BIT_ENABLE
#endif
#endif

#ifndef TRUE
#define TRUE                UINT8_C(1)
#endif
#ifndef FALSE
#define FALSE               UINT8_C(0)
#endif

/**\name I2C addresses */
#define BME280_I2C_ADDR_PRIM	UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC		UINT8_C(0x77)

/**\name BME280 chip identifier */
#define BME280_CHIP_ID  UINT8_C(0x60)

/**\name Register Address */
#define BME280_CHIP_ID_ADDR					UINT8_C(0xD0)
#define BME280_RESET_ADDR					UINT8_C(0xE0)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR	UINT8_C(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR		UINT8_C(0xE1)
#define BME280_PWR_CTRL_ADDR				UINT8_C(0xF4)
#define BME280_CTRL_HUM_ADDR				UINT8_C(0xF2)
#define BME280_CTRL_MEAS_ADDR				UINT8_C(0xF4)
#define BME280_CONFIG_ADDR					UINT8_C(0xF5)
#define BME280_DATA_ADDR					UINT8_C(0xF7)

/**\name API success code */
#define BME280_OK					INT8_C(0)

/**\name API error codes */
#define BME280_E_NULL_PTR			INT8_C(-1)
#define BME280_E_DEV_NOT_FOUND		INT8_C(-2)
#define BME280_E_INVALID_LEN		INT8_C(-3)
#define BME280_E_COMM_FAIL			INT8_C(-4)
#define BME280_E_SLEEP_MODE_FAIL	INT8_C(-5)

/**\name API warning codes */
#define BME280_W_INVALID_OSR_MACRO      INT8_C(1)

/**\name Macros related to size */
#define BME280_TEMP_PRESS_CALIB_DATA_LEN	UINT8_C(26)
#define BME280_HUMIDITY_CALIB_DATA_LEN		UINT8_C(7)
#define BME280_P_T_H_DATA_LEN				UINT8_C(8)

/**\name Sensor power modes */
#define	BME280_SLEEP_MODE		UINT8_C(0x00)
#define	BME280_FORCED_MODE		UINT8_C(0x01)
#define	BME280_NORMAL_MODE		UINT8_C(0x03)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)

#if 0
#define BME280_SET_BITS(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				((data << bitname##_POS) & bitname##_MSK))
#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				(data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
							(bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))
#endif

/**\name Macros for bit masking */
#define BME280_SENSOR_MODE_MSK	UINT8_C(0x03)
#define BME280_SENSOR_MODE_POS	UINT8_C(0x00)

#define BME280_CTRL_HUM_MSK		UINT8_C(0x07)
#define BME280_CTRL_HUM_POS		UINT8_C(0x00)

#define BME280_CTRL_PRESS_MSK	UINT8_C(0x1C)
#define BME280_CTRL_PRESS_POS	UINT8_C(0x02)

#define BME280_CTRL_TEMP_MSK	UINT8_C(0xE0)
#define BME280_CTRL_TEMP_POS	UINT8_C(0x05)

#define BME280_FILTER_MSK		UINT8_C(0x1C)
#define BME280_FILTER_POS		UINT8_C(0x02)

#define BME280_STANDBY_MSK		UINT8_C(0xE0)
#define BME280_STANDBY_POS		UINT8_C(0x05)

/**\name Sensor component selection macros
   These values are internal for API implementation. Don't relate this to
   data sheet.*/
#define BME280_PRESS		UINT8_C(1)
#define BME280_TEMP			UINT8_C(1 << 1)
#define BME280_HUM			UINT8_C(1 << 2)
#define BME280_ALL			UINT8_C(0x07)

/**\name Settings selection macros */
#define BME280_OSR_PRESS_SEL		UINT8_C(1)
#define BME280_OSR_TEMP_SEL			UINT8_C(1 << 1)
#define BME280_OSR_HUM_SEL			UINT8_C(1 << 2)
#define BME280_FILTER_SEL			UINT8_C(1 << 3)
#define BME280_STANDBY_SEL			UINT8_C(1 << 4)
#define BME280_ALL_SETTINGS_SEL		UINT8_C(0x1F)

/**\name Oversampling macros */
#define BME280_NO_OVERSAMPLING		UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X		UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X		UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X		UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X		UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X		UINT8_C(0x05)

/**\name Standby duration selection macros */
#define BME280_STANDBY_TIME_1_MS              (0x00)
#define BME280_STANDBY_TIME_62_5_MS           (0x01)
#define BME280_STANDBY_TIME_125_MS			  (0x02)
#define BME280_STANDBY_TIME_250_MS            (0x03)
#define BME280_STANDBY_TIME_500_MS            (0x04)
#define BME280_STANDBY_TIME_1000_MS           (0x05)
#define BME280_STANDBY_TIME_10_MS             (0x06)
#define BME280_STANDBY_TIME_20_MS             (0x07)

/**\name Filter coefficient selection macros */
#define BME280_FILTER_COEFF_OFF               (0x00)
#define BME280_FILTER_COEFF_2                 (0x01)
#define BME280_FILTER_COEFF_4                 (0x02)
#define BME280_FILTER_COEFF_8                 (0x03)
#define BME280_FILTER_COEFF_16                (0x04)


typedef struct{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t  dig_H1;
	int16_t dig_H2;
	uint8_t  dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t  dig_H6;
	int32_t t_fine;
} BME280_CALIB_DATA;

typedef struct bme280_settings {
	/*! pressure oversampling */
	uint8_t osr_p;
	/*! temperature oversampling */
	uint8_t osr_t;
	/*! humidity oversampling */
	uint8_t osr_h;
	/*! filter coefficient */
	uint8_t filter;
	/*! standby time */
	uint8_t standby_time;
} BME280_SETTINGS;

typedef struct bme280_data {
	/*! Compensated pressure */
	uint32_t pressure;
	/*! Compensated temperature */
	int32_t temperature;
	/*! Compensated humidity */
	uint32_t humidity;
} BME280_DATA;

typedef struct bme280_uncomp_data {
	/*! un-compensated pressure */
	uint32_t pressure;
	/*! un-compensated temperature */
	uint32_t temperature;
	/*! un-compensated humidity */
	uint32_t humidity;
} BME280_UNCOMP_DATA;

#endif	/* BNE280_MAC_H */

