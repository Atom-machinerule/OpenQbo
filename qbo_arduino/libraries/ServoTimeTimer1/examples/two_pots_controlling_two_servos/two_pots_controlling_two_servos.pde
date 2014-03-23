#include <ServoTimeTimer1.h>

#define servoPin1 9
#define servoPin2 10
#define potPin1 0
#define potPin2 1

ServoTimeTimer1 servo1;
ServoTimeTimer1 servo2;

int potVal1 = 0;
int potVal2 = 0;

void setup()
{
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  
}

void loop()
{

    potVal1 = analogRead(potPin1);
   
    potVal2 = analogRead(potPin2);
    
    servo1.write((potVal1*2)+500);
    servo2.write((potVal2*2)+500);
        
}
