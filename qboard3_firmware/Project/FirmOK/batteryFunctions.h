/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BATTERY_FUNCTIONS_H
#define __BATTERY_FUNCTIONS_H
/*
#define KO 0
#define CHARGING_CC 1
#define CHARGING_CV 2
#define CHARGED 3
#define DISCHARGING 4
*/
uint8_t getBatteryLevel(uint8_t cellCount, uint32_t cellNominalVoltage, uint32_t cellChargedVoltage, uint8_t batteryChargingState, uint32_t batteryVoltageLevel);

#endif /* __BATTERY_FUNCTIONS_H */
