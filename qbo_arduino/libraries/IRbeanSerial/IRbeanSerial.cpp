
#include <IRbeanSerial.h>

//unsigned long MsTimer2::msecs;
void (*IRbeanSerial::func)();
//volatile unsigned long MsTimer2::count;
volatile char IRbeanSerial::overflowing;
volatile unsigned int IRbeanSerial::tcnt2;
bool first_pass=true;

void IRbeanSerial::set(void (*f)()) {
	
	TIMSK2 &= ~(1<<TOIE2);
	TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
	TCCR2B &= ~(1<<WGM22);
	ASSR &= ~(1<<AS2);
	TIMSK2 &= ~(1<<OCIE2A);
	
	// prescaler set to 64
	TCCR2B |= (1<<CS22);
	TCCR2B &= ~((1<<CS21) | (1<<CS20));
	
	tcnt2 = 152;  //Interrumpimos cada 416us
		
	func = f;
}

void IRbeanSerial::start() {
	overflowing = 0;
  first_pass=true;

	//TCNT2 = tcnt2;
  TCNT2=255;
	TIMSK2 |= (1<<TOIE2);
}

void IRbeanSerial::stop() {
	TIMSK2 &= ~(1<<TOIE2);
}

void IRbeanSerial::_overflow() {
	
	if (!overflowing) {
		overflowing = 1;
		(*func)();
		overflowing = 0;
	}
}

ISR(TIMER2_OVF_vect) {
  if(first_pass)
  {
  	TCNT2 = 255-(255-IRbeanSerial::tcnt2)/2;
    first_pass=false;
  }
  else
  {
    TCNT2 = IRbeanSerial::tcnt2;
	  IRbeanSerial::_overflow();
  }
}

