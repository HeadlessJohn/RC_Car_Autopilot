#include "vl53l0x.h"


void VL53L0X_Init(){
	Dev->I2cHandle = &hi2c1;
	Dev->I2cDevAddr = 0x52;

	Dev2->I2cHandle = &hi2c2;
	Dev2->I2cDevAddr = 0x52;

	Dev3->I2cHandle = &hi2c3;
	Dev3->I2cDevAddr = 0x52;

	VL53L0X_WaitDeviceBooted( Dev );
	VL53L0X_SetDeviceAddress(Dev, 0x52);
	VL53L0X_DataInit( Dev );
	VL53L0X_StaticInit( Dev );
	VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	
	VL53L0X_WaitDeviceBooted( Dev2 );
	VL53L0X_SetDeviceAddress(Dev2, 0x52);
	VL53L0X_DataInit( Dev2 );
	VL53L0X_StaticInit( Dev2 );
	VL53L0X_PerformRefCalibration(Dev2, &VhvSettings2, &PhaseCal2);
	VL53L0X_PerformRefSpadManagement(Dev2, &refSpadCount2, &isApertureSpads2);
	VL53L0X_SetDeviceMode(Dev2, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

	VL53L0X_WaitDeviceBooted( Dev3 );
	VL53L0X_SetDeviceAddress(Dev3, 0x52);
	VL53L0X_DataInit( Dev3 );
	VL53L0X_StaticInit( Dev3 );
	VL53L0X_PerformRefCalibration(Dev3, &VhvSettings3, &PhaseCal3);
	VL53L0X_PerformRefSpadManagement(Dev3, &refSpadCount3, &isApertureSpads3);
	VL53L0X_SetDeviceMode(Dev3, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

	// Enable/Disable Sigma and Signal check
	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
	
	VL53L0X_SetLimitCheckEnable(Dev2, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(Dev2, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(Dev2, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	VL53L0X_SetLimitCheckValue(Dev2, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev2, 33000);
	VL53L0X_SetVcselPulsePeriod(Dev2, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(Dev2, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

	VL53L0X_SetLimitCheckEnable(Dev3, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(Dev3, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(Dev3, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	VL53L0X_SetLimitCheckValue(Dev3, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev3, 33000);
	VL53L0X_SetVcselPulsePeriod(Dev3, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(Dev3, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

	VL53L0X_StartMeasurement(Dev);
	VL53L0X_StartMeasurement(Dev2);
	VL53L0X_StartMeasurement(Dev3);
	return;
}

void VL53L0X_MeasurementStart(){
	static uint8_t cnt = 0;
	static uint8_t cnt2 = 0;
	static uint8_t cnt3 = 0;
	uint8_t DataReady  = 0;
	uint8_t DataReady2 = 0;
	uint8_t DataReady3 = 0;

	// 데이터가 준비되면 DataReady가 1이 됨
	VL53L0X_GetMeasurementDataReady(Dev, &DataReady);
	VL53L0X_GetMeasurementDataReady(Dev2, &DataReady2);
	VL53L0X_GetMeasurementDataReady(Dev3, &DataReady3);

	// 데이터가 준비되면 데이터를 읽어옴
	if(DataReady == 1) {
		// 읽은 데이터를 RangingData.RangeMilliMeter에 저장
		VL53L0X_GetRangingMeasurementData(Dev, &RangingData);
		// 인터럽트 마스크를 초기화 하여 새로운 데이터 입력을 준비
		VL53L0X_ClearInterruptMask(Dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
		// 1000이상의 값이 들어오면 이전 값으로 대체함
		distance_arr_left[cnt] = RangingData.RangeMilliMeter;
		if (distance_arr_left[cnt] >= 1000) {
			distance_arr_left[cnt] = distance_arr_left[ (cnt==0) ? 2 : (cnt-1) ];
		}
		if (distance_arr_front[cnt] < 20){ //최소 20mm
			distance_arr_front[cnt] = 20;
		}
		cnt++;
		cnt %= 3;
	}
	if(DataReady2 == 1) {
		VL53L0X_GetRangingMeasurementData(Dev2, &RangingData2);
		VL53L0X_ClearInterruptMask(Dev2, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
		distance_arr_front[cnt2] = RangingData2.RangeMilliMeter;
		if (distance_arr_front[cnt2] >= 1000) {
			distance_arr_front[cnt2] = distance_arr_front[ (cnt2==0) ? 2: (cnt2-1) ];
		}
		if (distance_arr_front[cnt2] < 20){ //최소 20mm
			distance_arr_front[cnt2] = 20;
		}
		cnt2++;
		cnt2 %= 3;
	}
	if(DataReady3 == 1) {
		VL53L0X_GetRangingMeasurementData(Dev3, &RangingData3);
		VL53L0X_ClearInterruptMask(Dev3, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
		distance_arr_right[cnt3] = RangingData3.RangeMilliMeter;
		if (distance_arr_right[cnt3] >= 1000) {
			distance_arr_right[cnt3] = distance_arr_right[ (cnt3==0) ? 2 : (cnt3-1) ];
		}
		if (distance_arr_front[cnt3] < 20){ //최소 20mm
			distance_arr_front[cnt3] = 20;
		}
		cnt3++;
		cnt3 %= 3;
	}
	return;
}