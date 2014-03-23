#ifndef IRbeanSerial_h
#define IRbeanSerial_h

#include <avr/interrupt.h>

namespace IRbeanSerial {
	//extern unsigned long msecs;
	extern void (*func)();
	//extern volatile unsigned long count;
	extern volatile char overflowing;
	extern volatile unsigned int tcnt2;
	
	void set(void (*f)());
	void start();
	void stop();
	void _overflow();
}

#endif
