/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DATA_STRUCTURES_H
#define __DATA_STRUCTURES_H

typedef struct {
	enum RegulatedOutputStateEnum { OFF=0, ON=1 } regulatedOutputState;
	enum UnegulatedOutputStateEnum { NOT_ACTIVE=0, ACTIVE=1 } unregulatedOutputState;
	enum ExternalPowerSourceStateEnum { DISCONNECTED=0, CONNECTED=1 } externalPowerSourceState;
	enum BatteryStateEnum { KO=0, CHARGING_CC=1, CHARGING_CV=2, CHARGED=3, DISCHARGING=4 } batteryState;
} StateStructure;

#define true 1
#define false 0

typedef uint8_t bool;

#endif /* __DATA_STRUCTURES_H */
