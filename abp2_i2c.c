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

/* Includes -------------------------------------------------------------------------------------*/
#include <abp2_i2c.h>
#include "main.h"

/* Variables declarations -----------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;
static uint8_t	abp2_cmd[3];

/* Functions definitions ------------------------------------------------------------------------*/

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
