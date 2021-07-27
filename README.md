# STM32_hsc_ssc_i2c

## Overview
This repo uses [TruStability HSC and SSC pressure sensor library for the Arduino](https://github.com/rodan/honeywell_hsc_ssc_i2c) and [Honeywell Implementation 
Info](https://sensing.honeywell.com/honeywell-sensing-force-i2c-comms-tech-note-008291.pdf) to read raw pressure and temperature count values using STM32. 

## File Information
* hsc_ssc_i2c.h : This header file contains the declarations of the sensor driver APIs.
* hsc_ssc_i2c.c : This source file contains the definitions of the sensor driver APIs.

## Usage Guide
The user have to set I2C Address and the constants used for calculate pressure. This information may vary by sensor, and can be found in [Honeywell SSC Pressure Sensor Datasheet](https://br.mouser.com/datasheet/2/187/honeywell_sensing_trustability_ssc_series_standard-1095240.pdf).
```
/* I2C communication address
 * Slave Pressure sensors address can be found based on the part number: _SC_________XA_.
 * Where X define the following adresses in 7-bit format:
 * S  - spi (this is not the right library for spi opperation)
 * 2  - i2c slave address 0x28 (used in the tests)
 * 3  - i2c slave address 0x38
 * 5  - i2c slave address 0x58
 * 6  - i2c slave address 0x68
 * 7  - i2c slave address 0x78
 * */
#define HSC_SSC_PRESS_ADDR			((0x28 << 1) | 0x01) // Only read option available

/* Datasheet specifications for SSC-D-AN-N-150PG-2-A-3 sensor:
 * SSC = Standard Accuracy, Compensated/Amplified
 * D = DIP (Dual Inline Pin)
 * AN = Single axial barbed port (layout)
 * N = Dry gases only, no diagnostics
 * 150PG = 0 psi to 150 psi
 * 2 = Address 0x28
 * A = 10% to 90% of Vsupply (analog), 214 counts (digital)
 * 3 = 3.3 Vdd voltage supply
 * */
#define OUTPUT_MIN 					0x666       // 10%
#define OUTPUT_MAX 					0x3999      // 90% of 2^14 - 1
#define PRESSURE_MIN 				0
#define PRESSURE_MAX_PSI 			150.0
#define PRESSURE_MAX_PASCAL 		1034213.6 	// 150 psi in pascal
```

### Pressure sensing
Pressure is measured in psi, but can be set to pascal by changing PRESSURE_MAX_PSI to PRESSURE_MAX_PASCAL.
```
float hsc_ssc_i2c_read_pressure(I2C_HandleTypeDef *hi2c){
	HAL_StatusTypeDef i2c_comm;
	uint8_t	hsc_ssc_buffer[2];
	uint8_t status = 0;
	uint16_t bridge_data = 0;

	i2c_comm = HAL_I2C_Master_Transmit(hi2c, HSC_SSC_PRESS_ADDR, hsc_ssc_buffer, 1, DEFAULT_I2C_TIMEOUT);
	if(i2c_comm == HAL_OK){
		HAL_Delay(30);
		/* Read result */
		i2c_comm = HAL_I2C_Master_Receive(hi2c, HSC_SSC_PRESS_ADDR, hsc_ssc_buffer, 2, DEFAULT_I2C_TIMEOUT);
		if(i2c_comm == HAL_OK){

			// first 2 bits from first byte (S1, S0):
			// 00 = normal operation, valid data.
			// 01 = device in command mode.
			// 10 = stale data: data that has already been fetched since the last measurement cycle, or data
			//      fetched before the first measurement has been completed.
			// 11 = diagnostic condition
			status = (hsc_ssc_buffer[0] & 0xc0) >> 6;

			if(status == 0){
				// resultant bits for bridge data
				bridge_data = ((hsc_ssc_buffer[0] & 0x3f) << 8) + hsc_ssc_buffer[1];
				return (1.0 * ((((bridge_data - OUTPUT_MIN) * (PRESSURE_MAX_PSI - PRESSURE_MIN)) / (OUTPUT_MAX - OUTPUT_MIN)) + PRESSURE_MIN));
			}
			else return HSC_SSC_MEASURE_FAILED;
		}
		else return HSC_SSC_MEASURE_FAILED;
	}
	return HSC_SSC_MEASURE_FAILED;
}
```

### Temperature sensing
Temperature is measured in 8 bit, but it can be set to 11 bits by changing HSC_SSC_TEMP_8BIT_RES parameter to HSC_SSC_TEMP_11BIT_RES in HAL_I2C_Master_Receive.
```
void hsc_ssc_i2c_get_data(I2C_HandleTypeDef *hi2c, float *pressure, float *temperature){
	HAL_StatusTypeDef i2c_comm;
	uint8_t	hsc_ssc_buffer[4];
	uint8_t status = 0;
	uint16_t bridge_data = 0;
	uint16_t temp_data = 0;

	i2c_comm = HAL_I2C_Master_Transmit(hi2c, HSC_SSC_PRESS_ADDR, hsc_ssc_buffer, 1, DEFAULT_I2C_TIMEOUT);
	if(i2c_comm == HAL_OK){
		HAL_Delay(30);
		/* Read result */
		i2c_comm = HAL_I2C_Master_Receive(hi2c, HSC_SSC_PRESS_ADDR, hsc_ssc_buffer, HSC_SSC_TEMP_8BIT_RES, DEFAULT_I2C_TIMEOUT);
		if(i2c_comm == HAL_OK){

			// first 2 bits from first byte (S1, S0):
			// 00 = normal operation, valid data.
			// 01 = device in command mode.
			// 10 = stale data: data that has already been fetched since the last measurement cycle, or data
			//      fetched before the first measurement has been completed.
			// 11 = diagnostic condition
			status = (hsc_ssc_buffer[0] & 0xc0) >> 6;

			if(status == 0){
				// resultant bits for bridge data
				bridge_data = ((hsc_ssc_buffer[0] & 0x3f) << 8) + hsc_ssc_buffer[1];
				*pressure = (1.0 * ((((bridge_data - OUTPUT_MIN) * (PRESSURE_MAX_PSI - PRESSURE_MIN)) / (OUTPUT_MAX - OUTPUT_MIN)) + PRESSURE_MIN));
				temp_data = ((hsc_ssc_buffer[2] << 8) + (hsc_ssc_buffer[3] & 0xe0)) >> 5;
				*temperature = (temp_data * 0.0977) - 50;
			}
		}
	}
}
```
