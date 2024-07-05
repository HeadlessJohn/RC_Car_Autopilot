#ifndef SRC_VL53L0X_VL53L0X_H_
#define SRC_VL53L0X_VL53L0X_H_

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "vl53l0x_api.h"


VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t  vl53l0x_c; 
VL53L0X_DEV    Dev = &vl53l0x_c;

VL53L0X_RangingMeasurementData_t RangingData2;
VL53L0X_Dev_t  vl53l0x_c2;
VL53L0X_DEV    Dev2 = &vl53l0x_c2;

VL53L0X_RangingMeasurementData_t RangingData3;
VL53L0X_Dev_t  vl53l0x_c3;
VL53L0X_DEV    Dev3 = &vl53l0x_c3;


uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

uint32_t refSpadCount2;
uint8_t isApertureSpads2;
uint8_t VhvSettings2;
uint8_t PhaseCal2;

uint32_t refSpadCount3;
uint8_t isApertureSpads3;
uint8_t VhvSettings3;
uint8_t PhaseCal3;

uint16_t distance_arr_left[5]  = {0,};
uint16_t distance_arr_front[5] = {0,};
uint16_t distance_arr_right[5] = {0,};

void VL53L0X_Init();
void VL53L0X_MeasurementStart();


#endif /* SRC_VL53L0X_VL53L0X_H_ */