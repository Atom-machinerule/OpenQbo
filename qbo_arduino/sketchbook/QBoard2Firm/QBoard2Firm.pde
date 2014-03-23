//-------------------------------------------------------------//
#include <ServoTimeTimer1.h>
#include <ServoTimer2.h>
//-------------------------------------------------------------//
#include "ArduBot.h"
#include "serialProtocol.h"
//-------------------------------------------------------------//
arduBot::ArduBot qbo;
arduBotSerial::SerialProtocol serialProtocol(&qbo);
//-------------------------------------------------------------//
void setup()
{
  qbo.begin();
}

void loop()
{
  serialProtocol.processSerial();
  qbo.spinOnce();     
}
