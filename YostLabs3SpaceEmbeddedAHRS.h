/*
 * YostLabs3SpaceEmbeddedAHRS.h
 *
 *  Created on: Nov 3, 2024
 *      Author: chama
 */

#ifndef INC_YOSTLABS3SPACEEMBEDDEDAHRS_H_
#define INC_YOSTLABS3SPACEEMBEDDEDAHRS_H_

#define REGISTER_BYTE_LENGTH 4 // 4 bytes per register -> 32bit read

#include "stm32g4xx_hal.h"

/************************ Enum definitions **************************/
typedef enum {
	USE_SPI = 0,
	USE_UART = 0,
} CommInterface;

typedef enum {
	ACCEL_SCALE_2g = 0,
	ACCEL_SCALE_4g = 1,
	ACCEL_SCALE_8g = 2,
} AccelScale;

typedef enum {
	GYRO_SCALE_250 = 0,
	GYRO_SCALE_500 = 1,
	GYRO_SCALE_1000 = 2,
	GYRO_SCALE_2000 = 3,
} GyroScale;

typedef enum {
	MAG_SCALE_088 = 0, // 0.88G
	MAG_SCALE_130 = 1, // 1.30G
	MAG_SCALE_190 = 2, // 1.90G
	MAG_SCALE_250 = 3, // 2.50G
	MAG_SCALE_400 = 4, // 4.00G
	MAG_SCALE_470 = 5, // 4.70G
	MAG_SCALE_560 = 6, // 5.60G
	MAG_SCALE_810 = 7, // 8.10G
} MagScale;

typedef enum {
	EULER_DECOMP_XYZ = 0,
	EULER_DECOMP_YZX = 1,
	EULER_DECOMP_ZXY = 2,
	EULER_DECOMP_ZYX = 3,
	EULER_DECOMP_XZY = 4,
	EULER_DECOMP_YXZ = 5,
} EulerDecompOrder;

typedef enum {
	FILTER_MODE_IMU = 0,
	FILTER_MODE_KF = 1,
	FILTER_MODE_QCOMP = 2,
	FILTER_MODE_QGRAD = 3,
} FilterMode;

// Command IDs in hex from datasheet -> https://yostlabs.com/wp-content/uploads/pdf/3-Space-Sensor-Users-Manual-3.pdf
typedef enum {
    GetTaredQuaternion = 0x00,
    GetTaredEulerAngles,
    GetTaredRotMat,
    GetTaredAxisAngle,
    GetTaredTwoVector,
    GetDiffQuaternion,
    GetUntaredQuaternion,
    GetUntaredEulerAngles,
    GetUntaredRotMat,
    GetUntaredAxisAngle,
    GetUntaredTwoVector,
    GetTaredTwoVectorSensor,
    GetUntaredTwoVectorSensor,

    SetEulerDecompositionOrder = 0x10,
    SetMagnetoresistiveThreshold,
    SetAccelerometerResistanceThreshold,
    OffsetWithCurrentOrientation,
    ResetBaseOffset,
    OffsetWithQuaternion,
    SetBaseOffsetWithCurrent,

    SetPinMode = 0x1D,
    GetPinMode,
    SetInterruptStatus,
    GetAllNormalised,
    GetNormalisedGyro,
    GetNormalisedAccelerometer,
    GetNormalisedCompass,

    GetAllCorrected = 0x25,
    GetCorrectedGyro,
    GetCorrectedAccelerometer,
    GetCorrectedCompass,
    GetCorrectedAccelerationGlobal,

    GetTemperatureC = 0x2B,
    GetTemperatureF,
    GetConfidenceFactor,

    CorrectRawGyro = 0x30,
    CorrectRawAccel,
    CorrectRawCompass,

    GetAllRaw = 0x40,
    GetRawGyroscope,
    GetRawAccelerometer,
    GetRawCompass,

    SetStreamingSlots = 0x50,
    GetStreamingSlots,
    SetStreamingTiming,
    GetStreamingTiming,
    GetStreamingBatch,
    StartStreaming,
    StopStreaming,

    UpdateCurrentTimestamp = 0x5F,
    TareWithCurrentOrientation,
    TareWithQuaternion,
    TareWithRotMat,
    SetStaticAccelerometerTrust,
    SetConfidenceAccelerometerTrust,
    SetStaticCompassTrust,
    SetConfidenceCompassTrust,
    SetDesiredUpdateRate,
    SetMultiReferenceVectors,
    SetReferenceVectorMode,
    SetOversampleRate,
    SetGyroscopeEnabled,
    SetAccelerometerEnabled,
    SetCompassEnabled,
    ResetMultiReferenceVectors,
    SetMultiReferenceTableResolution,
    SetCompassMultiReferenceVector,
    SetCompassMultiReferenceCheckVector,
    SetAccelerometerMultiReferenceVector,
    SetAccelerometerMultiReferenceCheckVector,
    SetAxisDirections,
    SetRunningAveragePercent,
    SetCompassReferenceVector,
    SetAccelerometerReferenceVector,
    ResetFilter,
    SetAccelerometerRange,
    SetMultiReferenceWeightPower,
    SetFilterMode,
    SetRunningAverageMode,
    SetGyroscopeRange,
    SetCompassRange,

    GetTareQuaternion = 0x80,
    GetTareRotMat,
    GetAccelerometerTrust,
    GetCompassTrust,
    GetCurrentUpdateRate,
    GetCompassReferenceVector,
    GetAccelerometerReferenceVector,
    GetReferenceVectorMode,
    GetCompassMultiReferenceVector,
    GetCompassMultiReferenceCheckVector,
    GetAccelerometerMultiReferenceVector,
    GetAccelerometerMultiReferenceCheckVector,
    GetGyroscopeEnabled,
    GetAccelerometerEnabled,
    GetCompassEnabled,
    GetAxisDirection,
    GetOversampleRate,
    GetRunningAveragePercent,
    GetDesiredUpdateRate,
    GetAccelerometerRange,
    GetMultiReferenceWeightPower,
    GetMultiReferenceResolution,
    GetNumberMultiReferenceCells,
    GetFilterMode,
    GetRunningAverageMode,
    GetGyroscopeRange,
    GetCompassRange,
    GetEulerDecompositionOrder,
    GetMagnetoresistiveThreshold,
    GetAccelerometerResistanceThreshold,
    GetOffsetQuaternion,
    SetCompassCalibrationCoefficients,
    SetAccelerometerCalibrationCoefficients,
    GetCompassCalibrationCoefficients,
    GetAccelerometerCalibrationCoefficients,
    GetGyroscopeCalibrationCoefficients,
    BeginGyroscopeAutoCalibration,
    SetGyroscopeCalibrationCoefficients,
    SetCalibrationMode,
    GetCalibrationMode,
    SetOrthoCalibrationPointFromCurrent,
    SetOrthoCalibrationPointFromVector,
    PerformOrthoCalibration,
    ClearOrthoCalibrationData,

    ReadBatteryVoltage = 0xC9,
    ReadBatteryPercentage,
    ReadBatteryStatus,

    SetLEDMode = 0xC4,
    GetLEDMode = 0xC8,


    SetWiredResponseHeader = 0xDD,
    GetWiredResponseHeader,
    GetFirmwareVersion,
    RestoreFactorySettings,
    CommitSettings,
    SoftwareReset,
    SetSleepMode,
    GetSleepMode,
    EnterBootloaderMode,
    GetHardwareVersion,
    SetUARTBaudRate,
    GetUARTBaudRate,
    SetUSBMode,
    GetUSBMode,
    GetSerialNumber,
    SetLEDColor,
    GetLEDColor,
    SetJoystickEnabled,
    SetMouseEnabled,
    GetJoystickEnabled,
    GetMouseEnabled,
    SetControlMode,
    SetControlData,
    GetControlMode,
    GetControlData,
    SetButtonGyroDisableLength,
    GetButtonGyroDisableLength,
    GetButtonState,
    SetMouseRelative,
    GetMouseRelative,
    SetJoystickMousePresent,
    GetJoystickMousePresent,

    NoCommand = 0xFF
} AHRSCommand;

/************************ Struct definitions **************************/

typedef struct AHRS_data {
	float gyroX;
	float gyroY;
	float gyroZ;

	float accelX;
	float accelY;
	float accelZ;

	float magX;
	float magY;
	float magZ;

	float quatOrientation[4]; // x, y, z, w order from the AHRS
	float eulerOrientation[3]; // pitch roll yaw order from the AHRS

} AHRS_data;

// Either use SPI or UART
typedef union AHRS_IO {
	// SPI pins and buffers
	struct {
		SPI_HandleTypeDef* hspi;
		GPIO_TypeDef* SPI_CS_Port;
		uint16_t SPI_CS_Pin;
		uint8_t spiTxBuffer[REGISTER_BYTE_LENGTH];
		uint8_t spiRxBuffer[REGISTER_BYTE_LENGTH];
		GPIO_TypeDef* SPI_Data_Rdy_Port;
		uint16_t SPI_Data_Rdy_Pin;
	};

	// UART pins and buffers
	struct {
		UART_HandleTypeDef* huart;
		uint8_t uartTxBuffer[REGISTER_BYTE_LENGTH];
		uint8_t uartRxBuffer[REGISTER_BYTE_LENGTH];
		GPIO_TypeDef* UART_Data_Rdy_Port;
		uint16_t UART_Data_Rdy_Pin;
	};

} AHRS_IO;

typedef struct AHRS_config {
	// Scales for sensors
	AccelScale accScale;
	GyroScale gyScale;
	MagScale mgScale;

	EulerDecompOrder eulerDecompOrder;
	FilterMode filter;

	// Orientation settings
	uint8_t orientShouldUseGyro;
	uint8_t orientShouldUseAccel;
} AHRS_config;

// Overarching struct to hold everything
typedef struct AHRS {
	AHRS_data* data;
	AHRS_IO* IO;
	AHRS_config* config;

	CommInterface interface;
} AHRS;

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

HAL_StatusTypeDef AHRS_setStreamingStandard(AHRS* ahrs_ptr);
HAL_StatusTypeDef AHRS_getStreamedData(AHRS* ahrs_ptr);
/************************ Low Level Functions **************************/
HAL_StatusTypeDef AHRS_readUART(AHRS_IO* IO_ptr, AHRSCommand cmd);
HAL_StatusTypeDef AHRS_writeUART(AHRS_IO* IO_ptr, uint8_t* buffer, uint16_t len);
HAL_StatusTypeDef AHRS_readSPI(AHRS_IO* IO_ptr, AHRSCommand cmd);
HAL_StatusTypeDef AHRS_writeSPI(AHRS_IO* IO_ptr, uint8_t* buffer, uint16_t len);

#endif /* INC_YOSTLABS3SPACEEMBEDDEDAHRS_H_ */
