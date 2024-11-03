/*
 * YostLabs3SpaceEmbeddedAHRS.c
 *
 *  Created on: Nov 3, 2024
 *      Author: chama
 */
#include "YostLabs3SpaceEmbeddedAHRS.h"

/************************ Initialisation Functions **************************/
HAL_StatusTypeDef AHRS_init(void);
HAL_StatusTypeDef AHRS_configure(AHRS_config* AHRS_cfg_ptr);

/************************ Data Acquisition Functions **************************/
HAL_StatusTypeDef AHRS_updateTaredQuaternion(AHRS* ahrs_ptr);
HAL_StatusTypeDef AHRS_updateTaredEulerAngles(AHRS* ahrs_ptr);
HAL_StatusTypeDef AHRS_updateTaredRotMat(AHRS* ahrs_ptr);
HAL_StatusTypeDef AHRS_updateTaredAxisAngle(AHRS* ahrs_ptr);
HAL_StatusTypeDef AHRS_updateTaredTwoVector(AHRS* ahrs_ptr);

HAL_StatusTypeDef AHRS_updateUntaredQuaternion(AHRS* ahrs_ptr);
HAL_StatusTypeDef AHRS_updateUntaredEulerAngles(AHRS* ahrs_ptr);
HAL_StatusTypeDef AHRS_updateUntaredRotMat(AHRS* ahrs_ptr);
HAL_StatusTypeDef AHRS_updateUntaredAxisAngle(AHRS* ahrs_ptr);
HAL_StatusTypeDef AHRS_updateUntaredTwoVector(AHRS* ahrs_ptr);

HAL_StatusTypeDef AHRS_setStreamingStandard(AHRS* ahrs_ptr) {
	HAL_StatusTypeDef result = HAL_OK;

	// Set streaming slots
	char* streamBufferTiming[100];
	char* streamSettings[100];

	uint8_t interval_us = 100*1000;
	uint8_t duration = -1;
	uint8_t start_delay = 0;

	sprintf(streamSettings,
			":%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			SetStreamingSlots,
			GetUntaredQuaternion,
			GetCorrectedGyro,
			GetCorrectedAccelerometer,
			GetCorrectedCompass,
			GetCorrectedAccelerationGlobal,
			GetTemperatureC,
			GetConfidenceFactor,
			NoCommand
	);
	sprintf(streamBufferTiming, ":%d,%d,%d,%d\n", SetStreamingTiming, interval_us, duration, start_delay);

	// Send this buffer to the AHRS
	if (ahrs_ptr->interface == USE_UART) {
		AHRS_writeUART(ahrs_ptr->IO, streamSettings, sizeof(streamSettings));
		AHRS_writeUART(ahrs_ptr->IO, streamBufferTiming, sizeof(streamBufferTiming));
	}

	else if (ahrs_ptr->interface == USE_SPI) {
		AHRS_writeSPI(ahrs_ptr->IO, streamSettings, sizeof(streamSettings));
		AHRS_writeSPI(ahrs_ptr->IO, streamBufferTiming, sizeof(streamBufferTiming));
	}

	return result;
}

HAL_StatusTypeDef AHRS_getStreamedData(AHRS* ahrs_ptr) {
	HAL_StatusTypeDef result = HAL_OK;


	return result;
}
/************************ Low Level Functions **************************/
HAL_StatusTypeDef AHRS_readUART(AHRS_IO* IO_ptr, AHRSCommand cmd) {
	HAL_StatusTypeDef result = HAL_UART_Receive(IO_ptr->huart, IO_ptr->uartRxBuffer, sizeof(IO_ptr->uartRxBuffer), HAL_MAX_DELAY);
	return result;
}

HAL_StatusTypeDef AHRS_writeUART(AHRS_IO* IO_ptr, uint8_t* buffer, uint16_t len) {
	HAL_StatusTypeDef result = HAL_UART_Transmit(IO_ptr->huart, buffer, len, HAL_MAX_DELAY);
	return result;
}

HAL_StatusTypeDef AHRS_readSPI(AHRS_IO* IO_ptr, AHRSCommand cmd) {
	HAL_StatusTypeDef result = HAL_OK;


	return result;
}

HAL_StatusTypeDef AHRS_writeSPI(AHRS_IO* IO_ptr, uint8_t* buffer) {
	HAL_StatusTypeDef result = HAL_OK;


	return result;
}

