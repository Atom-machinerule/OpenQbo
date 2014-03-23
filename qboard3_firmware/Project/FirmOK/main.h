/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "data_structures.h"
#include <stdio.h>
#include "debug.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define EASY_MODE
#define TR_INVERSE

#define EXTERNAL_POWER_PRESENT_THRESHOLD 145000 //This is 14.5 V
#define EXTERNAL_POWER_NOT_PRESENT_THRESHOLD 137000
#define DESIRED_REGULATED_OUTPUT_VALUE 125000 //This is 12.5 V
#define CURRENT_ON_OFF_THRESHOLD 5000 //This is 500 mA
#define BATTERY_KO_THRESHOLD 113000 //This is 11.3 V
#define MAX_BAT_CC_CURRENT_VALUE 60000 //This is 6000 mA
#define BAT_CC_CV_THRESHOLD 143000 //This is 14.3 V
#define BAT_CV_CHARGED_THRESHOLD 144000 //This is 14.4 V
#define BAT_CURRENT_CHARGED_THRESHOLD 10000 //This is 1000 mA

#define ADC_TO_MV_CONSTANT 42 //Value used to transform from ADC values to V values
#define ADC_TO_MA_CONSTANT 32 //Value used to transform from ADC values to A values

#define MAX_EXTERNAL_POWER_CURRENT 80000
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void TurnPowerBattery(void);
void TurnPowerExternal(void);
void TurnPowerOff(void);

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
