/**
  ******************************************************************************
  * @file           : abp2_i2c.h
  * @brief          : This file contains the headers of honeywell pressure
  * 				sensor ABP2LANT150PG2A3XX
  * @autor          : igorssantos-bit
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#ifndef INC_ABP2_I2C_H_
#define INC_ABP2_I2C_H_

/* Includes -------------------------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Defines --------------------------------------------------------------------------------------*/

/* Default communication value timeout */
#define DEFAULT_I2C_TIMEOUT			100

/* Measurement error value */
#define ABP2_MEASURE_FAILED			0xFFFFFFFF

/* I2C communication address
 * Slave Pressure sensors address can be found based on the part number: ABP2________XA3__.
 * Where X define the following adresses in 7-bit format:
 * S  - spi (this is not the right library for spi opperation)
 * 0  - i2c slave address 0x08
 * 1  - i2c slave address 0x18
 * 2  - i2c slave address 0x28 (used in the tests)
 * 3  - i2c slave address 0x38
 * 5  - i2c slave address 0x58
 * 6  - i2c slave address 0x68
 * 7  - i2c slave address 0x78
 * */
#define ABP2_READ_PRESS_ADDR		((0x28 << 1) | 0x01)
#define ABP2_WRITE_PRESS_ADDR		(0x28 << 1)

/* Read Command */
#define ABP2_READ_COMMAND			0xAA0000

/* Datasheet specifications for ABP2-L-AN-T-150PG-2-A-3-XX sensor:
 * ABP2 = Amplified Basic
 * L = Leadless SMT
 * AN = Single axial barbed port (layout)
 * T = Liquid media, food grade gel, no diagnostics
 * 150PG = 0 psi to 150 psi
 * 2 = Address 0x28
 * A = 10% to 90% of 2^24 counts
 * 3 = 3.3 Vdd voltage supply
 * xx = Feature
 * */
#define OUTPUT_MIN 					0x19999A    // 10%
#define OUTPUT_MAX 					0xE66666    // 90% of 2^24 - 1
#define PRESSURE_MIN 				0
#define PRESSURE_MAX_PSI 			150.0
#define PRESSURE_MAX_PASCAL 		1034213.6 	// 150 psi in pascal
#define TEMPERATURE_MAX 			150
#define TEMPERATURE_MIN 			-50

/* Functions declarations -----------------------------------------------------------------------*/

/**
 * @brief		Reads pressure measurement and temperature from the sensor via i2c
 * @param		hi2c: I2C bus handler.
 * @retval		Current pressure measured in psi and temperature em C.
 * 				ABP2_MEASURE_FAILED
 */
void abp2_i2c_get_data(I2C_HandleTypeDef *hi2c, float *pressure, float *temperature);

/**
 * @brief		Reads pressure measurement from the sensor via i2c
 * @param		hi2c: I2C bus handler.
 * @retval		Current pressure measured in psi.
 * 				ABP2_MEASURE_FAILED
 */
uint32_t abp2_i2c_read_pressure(I2C_HandleTypeDef *hi2c);

#endif /* INC_ABP2_I2C_H_ */
