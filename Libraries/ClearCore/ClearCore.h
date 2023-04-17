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
     
#ifndef __CLEARCORE_H__
#define __CLEARCORE_H__

// Header files from the ClearCore hardware that define connectors available
#include "AdcManager.h"
#include "CcioBoardManager.h"
#include "DigitalIn.h"
#include "DigitalInAnalogIn.h"
#include "DigitalInOut.h"
#include "DigitalInOutAnalogOut.h"
#include "DigitalInOutHBridge.h"
#include "EthernetManager.h"
#include "InputManager.h"
#include "LedDriver.h"
#include "EncoderInput.h"
#include "MotorDriver.h"
#include "MotorManager.h"
#include "SdCardDriver.h"
#include "SerialDriver.h"
#include "SerialUsb.h"
#include "StatusManager.h"
#include "SysManager.h"
#include "SysTiming.h"
#include "XBeeDriver.h"


namespace ClearCore {

extern LedDriver ConnectorLed;              

// IO connectors
extern DigitalInOutAnalogOut ConnectorIO0;  
extern DigitalInOut ConnectorIO1;           
extern DigitalInOut ConnectorIO2;           
extern DigitalInOut ConnectorIO3;           

// H-Bridge type connectors
extern DigitalInOutHBridge ConnectorIO4;    
extern DigitalInOutHBridge ConnectorIO5;    

// Digital input only connectors
extern DigitalIn ConnectorDI6;              
extern DigitalIn ConnectorDI7;              
extern DigitalIn ConnectorDI8;              

// Analog/Digital Inputs
extern DigitalInAnalogIn ConnectorA9;       
extern DigitalInAnalogIn ConnectorA10;      
extern DigitalInAnalogIn ConnectorA11;      
extern DigitalInAnalogIn ConnectorA12;      

// Motor Connectors
extern MotorDriver ConnectorM0;             
extern MotorDriver ConnectorM1;             
extern MotorDriver ConnectorM2;             
extern MotorDriver ConnectorM3;             

// Serial Port connectors
extern SerialUsb    ConnectorUsb;           
extern SerialDriver ConnectorCOM0;          
extern SerialDriver ConnectorCOM1;          

extern EthernetManager &EthernetMgr;

extern CcioBoardManager &CcioMgr;

extern MotorManager &MotorMgr;

extern AdcManager &AdcMgr;

extern InputManager &InputMgr;

extern XBeeDriver XBee;

extern EncoderInput EncoderIn;

extern StatusManager &StatusMgr;

extern SysTiming &TimingMgr;

extern SdCardDriver SdCard;

extern SysManager SysMgr;
}

using namespace ClearCore;

#endif // __CLEARCORE_H__
