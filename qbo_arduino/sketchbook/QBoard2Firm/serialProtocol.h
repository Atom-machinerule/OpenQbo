/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, TheCorpora.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TheCorpora nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.

 *
 * \author Miguel Angel Julian
 *********************************************************************/

#ifndef SerialProtocol_h
#define SerialProtocol_h

#include "comands.h"
#include "ArduBot.h"

namespace arduBotSerial
{
  class CCommand
  {
    public:
      byte commandNumber;
      byte nInputData;
      byte inputData[128];
      byte nOutputData;
      byte outputData[128];
      CCommand()
      {
        commandNumber=0;
        nInputData=0;
        nOutputData=0;
      };
  };
  class SerialProtocol {
    private:
      const byte INPUT_FLAG;
      const byte OUTPUT_FLAG;
      const byte INPUT_ESCAPE;
      CCommand command_;
      bool isInputEscaped_;
      bool isInputCorrect_;
      arduBot::ArduBot* robot;

      void processCommands();
      boolean procesaEntrada(byte* buf, byte length);
      void sendResponse();
      byte sendDataByte(byte* data, byte nInputData, byte crc);
      byte sendDataByte(byte nInputData, byte crc);
      void sendNack();
    public:
      SerialProtocol(arduBot::ArduBot *robot);
      void processSerial();
  };
}

#endif

