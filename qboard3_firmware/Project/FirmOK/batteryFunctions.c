#include "batteryFunctions.h"
#include "data_structures.h"

uint8_t percent=0;
bool firstPass = false;
uint8_t oldBatPercent=100;

uint8_t getBatteryLevel(uint8_t cellCount, uint32_t cellNominalVoltage_mV, uint32_t cellChargedVoltage, uint8_t batteryChargingState, uint32_t batteryVoltageLevel_01mV)
{
  //uint8_t percent=0;
  uint32_t batteryCellLevel=0;
  switch(batteryChargingState)
  {
  case KO:
    percent=0;
    oldBatPercent=0;
    firstPass = true;
    break;
  case CHARGING_CC:
  case CHARGING_CV:
	//percent=50;
	batteryCellLevel=(((batteryVoltageLevel_01mV-3250)*100)/*now it is in uV*//(cellCount*cellNominalVoltage_mV/*this is in mV*/)); /*This is scaled by 1000, so it is in %00 of cellMax*/
	/* a 0% charge is achieved with 86% of cellNominalVoltage */
	if (batteryCellLevel>860)
	{
	  percent=((batteryCellLevel-860)*100)/140;
          if (percent>100) percent=100;
          if (firstPass)
          {
            firstPass = false;
            oldBatPercent=percent;
          }
          else if (percent<oldBatPercent) percent=oldBatPercent;
          else oldBatPercent=percent;
          //if (percent>10) percent=(percent/5)*5;
	}
	else
        {
	  percent=0;
          oldBatPercent=0;
          firstPass = true;
        }
    break;
    /*
  case CHARGING_CV:
	percent=95;
    break;
    */
  case CHARGED:
	percent=100;
        oldBatPercent=100;
	break;
  case DISCHARGING:
	batteryCellLevel=((batteryVoltageLevel_01mV*100)/*now it is in uV*//(cellCount*cellNominalVoltage_mV/*this is in mV*/)); /*This is scaled by 1000, so it is in %00 of cellMax*/
	/* a 0% charge is achieved with 86% of cellNominalVoltage */
	if (batteryCellLevel>860)
	{
	  percent=((batteryCellLevel-860)*100)/140;
          if (percent>100) percent=100;
          if (firstPass)
          {
            firstPass = false;
            oldBatPercent=percent;
          }
          else if (percent>oldBatPercent) percent=oldBatPercent;
          else oldBatPercent=percent;
          //if (percent>10) percent=(percent/5)*5;
	}
	else
        {
	  percent=0;
          oldBatPercent=0;
          firstPass = true;
        }
	break;
  }
  return percent;
}
