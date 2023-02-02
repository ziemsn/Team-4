/*
    2  * Copyright (c) 2020 Teknic, Inc.
    3  *
    4  * Permission is hereby granted, free of charge, to any person obtaining a copy
    5  * of this software and associated documentation files (the "Software"), to deal
    6  * in the Software without restriction, including without limitation the rights
    7  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    8  * copies of the Software, and to permit persons to whom the Software is
    9  * furnished to do so, subject to the following conditions:
   10  *
   11  * The above copyright notice and this permission notice shall be included in
   12  * all copies or substantial portions of the Software.
   13  *
   14  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   15  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   16  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   17  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   18  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   19  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   20  * SOFTWARE.
   21  */
   22  
   23 #ifndef __CLEARCORE_H__
   24 #define __CLEARCORE_H__
   25 
   26 // Header files from the ClearCore hardware that define connectors available
   27 #include "AdcManager.h"
   28 #include "CcioBoardManager.h"
   29 #include "DigitalIn.h"
   30 #include "DigitalInAnalogIn.h"
   31 #include "DigitalInOut.h"
   32 #include "DigitalInOutAnalogOut.h"
   33 #include "DigitalInOutHBridge.h"
   34 #include "EthernetManager.h"
   35 #include "InputManager.h"
   36 #include "LedDriver.h"
   37 #include "EncoderInput.h"
   38 #include "MotorDriver.h"
   39 #include "MotorManager.h"
   40 #include "SdCardDriver.h"
   41 #include "SerialDriver.h"
   42 #include "SerialUsb.h"
   43 #include "StatusManager.h"
   44 #include "SysManager.h"
   45 #include "SysTiming.h"
   46 #include "XBeeDriver.h"
   47 
   48 
   49 namespace ClearCore {
   50 
   51 extern LedDriver ConnectorLed;              
   52 
   53 // IO connectors
   54 extern DigitalInOutAnalogOut ConnectorIO0;  
   55 extern DigitalInOut ConnectorIO1;           
   56 extern DigitalInOut ConnectorIO2;           
   57 extern DigitalInOut ConnectorIO3;           
   58 
   59 // H-Bridge type connectors
   60 extern DigitalInOutHBridge ConnectorIO4;    
   61 extern DigitalInOutHBridge ConnectorIO5;    
   62 
   63 // Digital input only connectors
   64 extern DigitalIn ConnectorDI6;              
   65 extern DigitalIn ConnectorDI7;              
   66 extern DigitalIn ConnectorDI8;              
   67 
   68 // Analog/Digital Inputs
   69 extern DigitalInAnalogIn ConnectorA9;       
   70 extern DigitalInAnalogIn ConnectorA10;      
   71 extern DigitalInAnalogIn ConnectorA11;      
   72 extern DigitalInAnalogIn ConnectorA12;      
   73 
   74 // Motor Connectors
   75 extern MotorDriver ConnectorM0;             
   76 extern MotorDriver ConnectorM1;             
   77 extern MotorDriver ConnectorM2;             
   78 extern MotorDriver ConnectorM3;             
   79 
   80 // Serial Port connectors
   81 extern SerialUsb    ConnectorUsb;           
   82 extern SerialDriver ConnectorCOM0;          
   83 extern SerialDriver ConnectorCOM1;          
   84 
   86 extern EthernetManager &EthernetMgr;
   87 
   89 extern CcioBoardManager &CcioMgr;
   90 
   92 extern MotorManager &MotorMgr;
   93 
   95 extern AdcManager &AdcMgr;
   96 
   98 extern InputManager &InputMgr;
   99 
  101 extern XBeeDriver XBee;
  102 
  104 extern EncoderInput EncoderIn;
  105 
  107 extern StatusManager &StatusMgr;
  108 
  110 extern SysTiming &TimingMgr;
  111 
  113 extern SdCardDriver SdCard;
  114 
  116 extern SysManager SysMgr;
  117 }
  118 
  119 using namespace ClearCore;
  120 
  121 #endif // __CLEARCORE_H__
