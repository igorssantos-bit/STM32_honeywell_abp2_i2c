# STM32_honeywell_abp2_i2c

## Overview
This repo uses [Honeywell ABP2 Series Datasheet](https://br.mouser.com/datasheet/2/187/Honeywell_ABP2_Series_Datasheet_-_Issue_A-1853521.pdf) to read raw pressure and temperature count values using STM32. 

## File Information
* abp2_i2c.h : This header file contains the declarations of the sensor driver APIs.
* abp2_i2c.c : This source file contains the definitions of the sensor driver APIs.

## Usage Guide
The user have to set I2C Address and the constants used for calculate pressure. This information may vary by sensor, and can be found in the datasheet.
```
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
```

### Pressure sensing
Pressure is measured in psi, but can be set to pascal by changing PRESSURE_MAX_PSI to PRESSURE_MAX_PASCAL.
```
uint32_t abp2_i2c_read_pressure(I2C_HandleTypeDef *hi2c){
	HAL_StatusTypeDef i2c_comm;
	uint8_t	abp2_buffer[4];
	uint8_t status = 0;
	uint32_t press_data = 0;
	float pressure;

	abp2_cmd[0] = (uint8_t)(ABP2_READ_COMMAND>>16);
	abp2_cmd[1] = (uint8_t)(ABP2_READ_COMMAND>>8);
	abp2_cmd[2] = (uint8_t)(ABP2_READ_COMMAND);

	i2c_comm = HAL_I2C_Master_Transmit(hi2c, ABP2_WRITE_PRESS_ADDR, abp2_cmd, 3, DEFAULT_I2C_TIMEOUT);
	if(i2c_comm == HAL_OK){
		HAL_Delay(6);
		/* Read result */
		i2c_comm = HAL_I2C_Master_Receive(hi2c, ABP2_READ_PRESS_ADDR, abp2_buffer, 4, DEFAULT_I2C_TIMEOUT);
		if(i2c_comm == HAL_OK){
			// bit 7 -
			// bit 6 - power indicator (1 = powered)
			// bit 5 - busy flag
			// bit 4 -
			// bit 3 -
			// bit 2 - mem integrity / error flag (1 = test failed)
			// bit 1 -
			// bit 0 - math saturation (1 = internal error)
			status = (abp2_buffer[0] & 00000101);

			if(status != 00000101){
				press_data = (abp2_buffer[1] << 16) + (abp2_buffer[2] << 8) + abp2_buffer[3];
				pressure = (1.0 * ((((press_data - OUTPUT_MIN) * (PRESSURE_MAX_PSI - PRESSURE_MIN)) / (OUTPUT_MAX - OUTPUT_MIN)) + PRESSURE_MIN));
				return (uint32_t) (pressure*100);
			}
			else return ABP2_MEASURE_FAILED;
		}
		else return ABP2_MEASURE_FAILED;
	}
	return ABP2_MEASURE_FAILED;
}
```

### Temperature sensing
Temperature is measured in 8 bit, but it can be set to 11 bits by changing HSC_SSC_TEMP_8BIT_RES parameter to HSC_SSC_TEMP_11BIT_RES in HAL_I2C_Master_Receive.
```
void abp2_i2c_get_data(I2C_HandleTypeDef *hi2c, float *pressure, float *temperature){
	HAL_StatusTypeDef i2c_comm;
	uint8_t	abp2_buffer[7];
	uint8_t status = 0;
	uint32_t press_data = 0;
	uint32_t temp_data = 0;

	abp2_cmd[0] = (uint8_t)(ABP2_READ_COMMAND>>16);
	abp2_cmd[1] = (uint8_t)(ABP2_READ_COMMAND>>8);
	abp2_cmd[2] = (uint8_t)(ABP2_READ_COMMAND);

	i2c_comm = HAL_I2C_Master_Transmit(hi2c, ABP2_WRITE_PRESS_ADDR, abp2_cmd, 3, DEFAULT_I2C_TIMEOUT);
	if(i2c_comm == HAL_OK){
		HAL_Delay(6);
		/* Read result */
		i2c_comm = HAL_I2C_Master_Receive(hi2c, ABP2_READ_PRESS_ADDR, abp2_buffer, 7, DEFAULT_I2C_TIMEOUT);
		if(i2c_comm == HAL_OK){
			// bit 7 -
			// bit 6 - power indicator (1 = powered)
			// bit 5 - busy flag
			// bit 4 -
			// bit 3 -
			// bit 2 - mem integrity / error flag (1 = test failed)
			// bit 1 -
			// bit 0 - math saturation (1 = internal error)
			status = (abp2_buffer[0] & 00000101);

			if(status != 00000101){
				// resultant bits for bridge data
				press_data = (abp2_buffer[1] << 16) + (abp2_buffer[2] << 8) + abp2_buffer[3];
				*pressure = (1.0 * ((((press_data - OUTPUT_MIN) * (PRESSURE_MAX_PSI - PRESSURE_MIN)) / (OUTPUT_MAX - OUTPUT_MIN)) + PRESSURE_MIN));
				temp_data = (abp2_buffer[4] << 16) + (abp2_buffer[5] << 8) + abp2_buffer[6];
				*temperature = ((temp_data * (TEMPERATURE_MAX - TEMPERATURE_MIN) / 16777215.0) + TEMPERATURE_MIN) * 1.0;
			}
		}
	}
}
```
