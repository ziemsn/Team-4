/////////////////////// GenieArduino 2022 - DEV ///////////////////////
//
//      Library to utilise the 4D Systems Genie interface to displays
//      that have been created using the Visi-Genie creator platform.
//      This is intended to be used with the Arduino platform.
//
//      Improvements/Updates by
//        Antonio Brewer & 4D Systems Engineering, May 2022, www.4dsystems.com.au
//		  Antonio Brewer & 4D Systems Engineering, February 2022, www.4dsystems.com.au
//		  Antonio Brewer & 4D Systems Engineering, January 2022, www.4dsystems.com.au
//		  Antonio Brewer & 4D Systems Engineering, July 2021, www.4dsystems.com.au
//		  Antonio Brewer & 4D Systems Engineering, June 2018, www.4dsystems.com.au
//        4D Systems Engineering, August 2017, www.4dsystems.com.au
//		  Antonio Brewer & 4D Systems Engineering, July 2017, www.4dsystems.com.au
//        4D Systems Engineering, October 2015, www.4dsystems.com.au
//        4D Systems Engineering, September 2015, www.4dsystems.com.au
//        4D Systems Engineering, August 2015, www.4dsystems.com.au
//        4D Systems Engineering, May 2015, www.4dsystems.com.au
//        Matt Jenkins, March 2015, www.majenko.com
//        Clinton Keith, January 2015, www.clintonkeith.com
//        4D Systems Engineering, July 2014, www.4dsystems.com.au
//        Clinton Keith, March 2014, www.clintonkeith.com
//        Clinton Keith, January 2014, www.clintonkeith.com
//        4D Systems Engineering, January 2014, www.4dsystems.com.au
//        4D Systems Engineering, September 2013, www.4dsystems.com.au
//      Written by
//        Rob Gray (GRAYnomad), June 2013, www.robgray.com
//      Based on code by
//        Gordon Henderson, February 2013, <projects@drogon.net>
//
//      Copyright (c) 2012-2021 4D Systems Pty Ltd, Sydney, Australia
/*********************************************************************
 * This file is part of genieArduino:
 *    genieArduino is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    genieArduino is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with genieArduino.
 *    If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************/
#if defined (SPARK)
   #include "application.h"
   #define lowByte(w) ((uint8_t)((w) & 0xFF))
   #define highByte(w) ((uint8_t)((w) >> 8))
 #else
   #if defined(ARDUINO) && ARDUINO >= 100
     #include "Arduino.h"
   #else
     #include "WProgram.h"
   #endif
 #endif

#include "genieArduinoDEV.h"
#include "genie_buffer.h"
#include <math.h>
#include <string.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2


// ######################################
// ## GENIE CLASS ####################### 
// ######################################
Genie::Genie() {
  UserHandler = nullptr;
  UserByteReader = nullptr;
  UserDoubleByteReader = nullptr;
  debugSerial = nullptr;
}

// ######################################
// ## Setup ############################# 
// ######################################
bool Genie::Begin(HardwareSerial &serial) {
  deviceSerial = &serial;
  tx_delay = 0;
  return Begin_common();
}

#if GENIE_SS_SUPPORT
	bool Genie::Begin(SoftwareSerial &serial) {
	  deviceSerial = &serial;
	  tx_delay = 1000;
	  return Begin_common();
	}
#endif

bool Genie::Begin(Stream &serial, uint16_t txDelay) {
  deviceSerial = &serial;
  tx_delay = txDelay;
  return Begin_common();
}

bool Genie::Begin_common() {
  genieStart = 1;
  _incomming_queue.clear();
  uint32_t timeout_start = millis(); // timeout timer
  while ( millis() - timeout_start <= 2000 ) { 
    if ( DoEvents() == GENIE_REPORT_OBJ && !genieStart ) return 1;
  }
  if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: Failed to detect display during setup"));
  if ( UserHandler ) {
    uint8_t buffer[6] = { GENIE_DISCONNECTED, 0, 0, 0, 0 };
    _incomming_queue.push_back(buffer, 6);
  }
  displayDetected = 0;
  return 0;
}

void Genie::AttachDebugStream(Stream &serial) {
  debugSerial = &serial;
}

bool Genie::IsOnline() {
  return displayDetected;
}

int16_t Genie::GetForm() {
  return currentForm;
}

void Genie::SetForm(uint8_t newForm) {
  WriteObject(GENIE_OBJ_FORM, newForm, (uint16_t)0x0000);
}

void Genie::SetRecoveryInterval(uint8_t pulses) {
  recover_pulse = pulses;
}

// ######################################
// ## AttachEventHandler ################ 
// ######################################

void Genie::AttachEventHandler(UserEventHandlerPtr userHandler) {
  UserHandler = userHandler;
  if ( !displayDetected ) {
    if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: Handler setup, display disconnected"));
    uint8_t buffer[6] = { GENIE_DISCONNECTED, 0, 0, 0, 0 };
    _incomming_queue.push_back(buffer, 6);
  }
  else {
    if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: Handler setup, display online"));
    uint8_t buffer[6] = { GENIE_READY, 0, 0, 0, 0 };
    _incomming_queue.push_back(buffer, 6);
  }
}

void Genie::AttachMagicByteReader(UserBytePtr userHandler) {
  UserByteReader = userHandler;
}

void Genie::AttachMagicDoubleByteReader(UserDoubleBytePtr userHandler) {
  UserDoubleByteReader = userHandler;
}

uint32_t Genie::GetUptime() {
  if ( displayDetected ) return millis() - display_uptime;
  else return 0;
}

// ######################################
// ## GetNextByte ####################### 
// ######################################
int16_t Genie::GetNextByte() {
  if ( magic_report_len-- < 1 ) {
    magic_report_len = 0;
    magic_overpull_count++;
    return -1;
  }
  uint32_t timeout = millis();
  while ( millis() - timeout < 100 && deviceSerial->available() < 1 );
  return deviceSerial->read();
}

// ######################################
// ## GetNextDoubleByte ################# 
// ######################################
int32_t Genie::GetNextDoubleByte() {
  // protection to be implemented
  uint32_t timeout = millis();
  while ( millis() - timeout < 200 && deviceSerial->available() < 2 );
  return ((uint16_t)(deviceSerial->read() << 8) | deviceSerial->read());
}

// ######################################
// ## Read Object ####################### 
// ######################################
int32_t Genie::ReadObject(uint8_t object, uint8_t index, bool now) {
  if ( !displayDetected ) {
    DoEvents();
    return -1;
  }
  DoEvents();
  uint8_t checksum = 0, buffer[5] = { (uint8_t)currentForm, (uint8_t)GENIE_READ_OBJ, object, index, 0 };
  for ( uint8_t i = 1; i < 4; i++ ) checksum ^= buffer[i];
  buffer[4] = checksum;
  if ( now && displayDetected ) {
    block_dequeue = 1; // disable dequeueing
    while ( pendingACK ) DoEvents(); // wait & finish pending ACKs
    handler_response_request = 1; // request widget value immediately
    handler_response_values[1] = object;
    handler_response_values[2] = index;
    writeMode(&buffer[1],4);
    block_dequeue = 0; // enable dequeueing
    uint32_t timeout = millis();
    while ( handler_response_request ) {
      if ( millis() - timeout > 100 ) {
        handler_response_request = 0;
        return -1;
      }
      DoEvents();
    }
    return ((int32_t)(handler_response_values[3] << 8) | handler_response_values[4]);
  }
  if ( !_outgoing_queue.replace(buffer,5,1,2,3) ) {
    if ( _outgoing_queue.size() == _outgoing_queue.capacity() ) if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: Overflow writing frames to queue!"));
    _outgoing_queue.push_back(buffer,5);
  }
  if ( now && !displayDetected ) return -1;
  return 1;
}

// ######################################
// ## Write WriteIntLedDigits ###########
// ######################################

uint16_t Genie::WriteIntLedDigits(uint16_t index, int16_t data) {
    return WriteObject(GENIE_OBJ_ILED_DIGITS_L, index, data);
}

uint16_t Genie::WriteIntLedDigits(uint16_t index, float data) {
    FloatLongFrame frame;
    frame.floatValue = data;
    WriteObject(GENIE_OBJ_ILED_DIGITS_H, index, frame.wordValue[1]);
    return WriteObject(GENIE_OBJ_ILED_DIGITS_L, index, frame.wordValue[0]);
}

uint16_t Genie::WriteIntLedDigits(uint16_t index, int32_t data) {
    FloatLongFrame frame;
    frame.longValue = data;
    WriteObject(GENIE_OBJ_ILED_DIGITS_H, index, frame.wordValue[1]);
    return WriteObject(GENIE_OBJ_ILED_DIGITS_L, index, frame.wordValue[0]);
}

// ######################################
// ## Write Object ######################
// ######################################
bool Genie::WriteObject(uint8_t object, uint8_t index, uint16_t data) {
  if ( !displayDetected ) {
    DoEvents();
    return 0;
  }
  DoEvents();

  if ( main_handler_active ) {
    if ( GENIE_OBJ_FORM == object ) currentForm = index;
    return WriteObjectPriority(object,index,data);
  }

  uint8_t checksum = 0, buffer[7] = { (uint8_t)currentForm, GENIE_WRITE_OBJ, object, index, (uint8_t)(data >> 8), (uint8_t)data, 0 };
  for ( uint8_t i = 1; i < 6; i++ ) checksum ^= buffer[i];
  buffer[6] = checksum;

  if ( object == GENIE_OBJ_SCOPE ) return WriteObjectPriority(object,index,data);
  if ( object == GENIE_OBJ_COOL_GAUGE ) return WriteObjectPriority(object,index,data);
  if ( !_outgoing_queue.replace(buffer,7,1,2,3) ) {
    if ( _outgoing_queue.size() == _outgoing_queue.capacity() ) if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: Overflow writing frames to queue!"));

    if ( GENIE_OBJ_FORM == object ) {
      WriteObjectPriority(object, index, data); /* write the form to display immediately */
      currentForm = index; /* update the local form state immediately */
    }
    else _outgoing_queue.push_back(buffer,7); /* queue normal objects */
  }
  return 1;
}


// ######################################
// ## Write Object Priority Task ########
// ######################################

bool Genie::WriteObjectPriority(uint8_t object, uint8_t index, uint16_t data) {
  if ( !displayDetected ) {
    DoEvents();
    return 0;
  }
  uint8_t checksum = 0, buffer[7] = { (uint8_t)currentForm, GENIE_WRITE_OBJ, object, index, (uint8_t)(data >> 8), (uint8_t)data, 0 };
  for ( uint8_t i = 1; i < 6; i++ ) checksum ^= buffer[i];
  buffer[6] = checksum;
  block_dequeue = 1; // disable dequeue
  while ( pendingACK ) DoEvents(); // wait pending ACKs

  writeMode(&buffer[1],6);

  pendingACK = 1; // enable ACK check
  pendingACK_timeout = millis(); // reset ACK check timer
  while ( pendingACK ) DoEvents(); // wait pending ACKs
  block_dequeue = 0; // re-enable dequeue

  return 1;
}




// ######################################
// ## Write Contrast #################### 
// ######################################
bool Genie::WriteContrast(uint8_t value) {
  DoEvents();
  uint8_t checksum = 0, buffer[4] = { (uint8_t)currentForm, GENIE_WRITE_CONTRAST, value, 0 };
  for ( uint8_t i = 1; i < 3; i++ ) checksum ^= buffer[i];
  buffer[3] = checksum;
  if ( !_outgoing_queue.replace(buffer,4,1,1,1) ) {
    _outgoing_queue.push_back(buffer,4);
    return 0;
  }
  return 1;
}


// ######################################
// ## User Ping #########################
// ######################################
void Genie::Ping(uint16_t interval) {
  if ( displayDetected && millis() - pingSpacer > interval ) {
    uint8_t buffer[4] = { (uint8_t)GENIE_READ_OBJ, GENIE_OBJ_FORM , 0, 10 };
    writeMode(buffer,4);
    pingRequest = 1;
    pingResponse = micros();
    pingSpacer = millis();
  }
}

// ######################################
// ## Write mode between bytes ##########
// ######################################
void Genie::writeMode(uint8_t *bytes, uint8_t len) {
  for ( uint8_t i = 0; i < len; i++ ) {
    deviceSerial->write(bytes[i]);
    delayMicroseconds(tx_delay);
  }
}

// ######################################
// ## Do Events #########################
// ######################################
inline int16_t Genie::DoEvents() {

  if ( !displayDetected ) {
    if ( deviceSerial->available() > 24) while(deviceSerial->available()) deviceSerial->read();
    currentForm = -1;
    pendingACK = 0;
  }

  /* Compatibility with sketches that include reset in setup, to prevent disconnection */
  if ( displayDetected && (millis() < 7000) ) display_uptime = millis();

  uint32_t autoPing_swapSpeed;

  if ( millis() - autoPingTimer > ( ( displayDetected && !NAK_detected ) ? autoPing_swapSpeed = AUTO_PING_CYCLE : autoPing_swapSpeed = recover_pulse ) ) {
    autoPingTimer = millis();
    if ( displayDetected && millis() - display_uptime > DISPLAY_TIMEOUT ) {
      display_uptime = millis();
       if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: disconnected by display timeout"));
      uint8_t buffer[6] = { GENIE_DISCONNECTED, 0, 0, 0, 0 };
      _incomming_queue.push_back(buffer, 6);
      displayDetected = 0;
    }
    uint8_t buffer[4] = { (uint8_t)GENIE_READ_OBJ, GENIE_OBJ_FORM , 0, 10 };
    writeMode(buffer,4);
    autoPingFlag = 1;
  }

  if ( deviceSerial->available() > 0 ) {
    switch ( deviceSerial->peek() ) {
      case GENIE_REPORT_OBJ: {
          if ( deviceSerial->available() >= 6 ) {
            uint8_t buffer[6], checksum = 0;
            for ( uint8_t i = 0; i < 6; i++ ) {
              buffer[i] = deviceSerial->read();
              if ( i < 5 ) checksum ^= buffer[i];
            }
            if ( checksum == buffer[5] ) {
              if ( GENIE_OBJ_FORM == buffer[1] ) {
                currentForm = buffer[4];
                if ( !displayDetected ) {
                  if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: online"));
                  uint8_t buffer[6] = { GENIE_READY, 0, 0, 0, 0 };
                  if ( UserHandler != nullptr ) _incomming_queue.push_back(buffer, 6);
                  displayDetected = 1;
                  display_uptime = millis();
                  genieStart = 0;
                  return GENIE_REPORT_OBJ;
                }
                if ( NAK_detected ) {
                  if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: Recovered from NAK(s)"));
                  NAK_recovery_counter = 0;
                  NAK_detected = 0;
                  return GENIE_REPORT_OBJ;
                }
                if ( autoPingFlag ) {
                  autoPingFlag = 0;
                  if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: AutoPing success!"));
                  display_uptime = millis();
                  return GENIE_REPORT_OBJ;
                }
                if ( pingRequest ) {
                  pingRequest = 0;
                  uint32_t _time = micros() - pingResponse;
                  uint8_t buffer[6] = { GENIE_PING, (uint8_t)(_time >> 24), (uint8_t)(_time >> 16), (uint8_t)(_time >> 8), (uint8_t)(_time) };
                  _incomming_queue.push_back(buffer, 6);
                  return GENIE_REPORT_OBJ;
                }
              }
              if ( !displayDetected ) return 0; // block the ping request events when offline
              if ( handler_response_request && handler_response_values[1] == buffer[1] && handler_response_values[2] == buffer[2] ) {
                memmove(handler_response_values, buffer, 6);
                handler_response_request = 0;
                return GENIE_REPORT_OBJ;
              }
              _incomming_queue.push_back(buffer, 6);
            }
          }
          return GENIE_REPORT_OBJ;
        }
      case GENIE_REPORT_EVENT: {
          if ( deviceSerial->available() >= 6 ) {
            uint8_t buffer[6], checksum = 0;
            for ( uint8_t i = 0; i < 6; i++ ) {
              buffer[i] = deviceSerial->read();
              if ( i < 5 ) checksum ^= buffer[i];
            }
            if ( checksum == buffer[5] ) {
              if ( GENIE_OBJ_FORM == buffer[1] ) currentForm = buffer[4];
              if ( GENIE_OBJ_4DBUTTON != buffer[1] &&
                   GENIE_OBJ_USERBUTTON != buffer[1] ) {
                if ( !_incomming_queue.replace(buffer,6,0,1,2 ) ) _incomming_queue.push_back(buffer, 6);
              }
              else _incomming_queue.push_back(buffer, 6);
            }
          }
          return GENIE_REPORT_EVENT;
        }

      case GENIEM_REPORT_BYTES: {
          if ( !displayDetected ) { deviceSerial->read(); return 0; }
          if ( deviceSerial->available() < 4 ) break; // magic report event less than 3 bytes? check again.
          uint8_t data[] = { (uint8_t)deviceSerial->read(), (uint8_t)deviceSerial->read(), (uint8_t)deviceSerial->read() };
          magic_report_len = data[2];
          magic_overpull_count = 0;
          if ( UserByteReader != nullptr ) {
            UserByteReader( data[1], data[2] );
            if ( magic_report_len > 0 ) {
              if ( debugSerial != nullptr ) {
                debugSerial->print(F("[Genie]: User forgot "));
                debugSerial->print(magic_report_len);
                debugSerial->println(F(" magic byte(s). Flushing rest..."));
              }
              uint32_t timeout = millis();
              while ( magic_report_len && millis() - timeout < 100 ) {
                if ( deviceSerial->available() > 0 ) {
                  magic_report_len--;
                  deviceSerial->read();
                  timeout = millis();
                }
              }
            }
            else {
              if ( debugSerial != nullptr ) {
                if ( !magic_overpull_count ) debugSerial->println(F("[Genie]: User captured all magic bytes!"));
                else {
                  debugSerial->print(F("[Genie]: User captured all magic bytes, but tried to pull more than provided! ("));
                  debugSerial->print(magic_overpull_count);
                  debugSerial->println(F(" byte(s))")); 
                }
              }
            }
            display_uptime = millis();
          }
          else {
            if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: MMagic bytes callback not set!"));
            for ( uint16_t i = 0; i < data[2]; i++) deviceSerial->read();
          }
          uint32_t timeout = millis();
          while ( millis() - timeout < 5000 && deviceSerial->available() < 1 );
          deviceSerial->read();
          return GENIEM_REPORT_BYTES;
        }

      case GENIEM_REPORT_DBYTES: {
          if ( !displayDetected ) { deviceSerial->read(); return 0; }
          if ( deviceSerial->available() < 3 ) break; // magic report event less than 3 bytes? check again.
          uint8_t data[] = { (uint8_t)deviceSerial->read(), (uint8_t)deviceSerial->read(), (uint8_t)deviceSerial->read() };
          if ( UserDoubleByteReader != nullptr ) {
            UserDoubleByteReader( data[1], data[2] );
            (void)GetNextByte();
            return GENIEM_REPORT_DBYTES;
            // over/under pulling protection to be implemented as above
          }
          else {
            if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: Magic double bytes callback not set!"));
            for ( uint16_t i = 0; i < 2 * data[2]; i++) deviceSerial->read();
          }
          uint32_t timeout = millis();
          while ( millis() - timeout < 5000 && deviceSerial->available() < 1 );
          deviceSerial->read();
          return GENIEM_REPORT_DBYTES;
        }

      case GENIE_ACK: {
          deviceSerial->read();
          if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: Received ACK!"));
          pendingACK = 0;
          return GENIE_ACK;
        }
      case GENIE_NAK: {
          while ( deviceSerial->peek() == GENIE_NAK ) deviceSerial->read();
          if ( !genieStart && !NAK_detected && debugSerial != nullptr ) debugSerial->println(F("[Genie]: Received NAK!"));
          NAK_detected = 1;
          NAK_recovery_counter++;
          if ( NAK_recovery_counter >= 2 ) {
            NAK_recovery_counter = 0;
            deviceSerial->write(0xFF);
          }
          return GENIE_NAK;
        }
      default: {
          if ( displayDetected && !NAK_detected && debugSerial != nullptr ) {
            debugSerial->print(F("[Genie]: Bad Byte: "));
            debugSerial->println(deviceSerial->read());
          }
          break;
        }
    }
  }
  dequeue_processing();
  if ( !main_handler_active && _incomming_queue.size() && UserHandler != nullptr ) {
    main_handler_active = 1;
    UserHandler();
    main_handler_active = 0;
  }
  return -1;
}

// ######################################
// ## Dequeue Processing ################
// ######################################

void Genie::dequeue_processing() {
  if ( pendingACK ) { /* check if ACK timeout, clear flag */
    if ( millis() - pendingACK_timeout >= 500 ) {
      if ( debugSerial != nullptr ) debugSerial->println(F("[Genie]: ACK timeout!"));
      pendingACK = 0;
    }
  }
  else { /* if no ACK is expected, send another request from queue */
    if ( !block_dequeue && displayDetected && !NAK_detected && _outgoing_queue.size() ) {
      uint8_t _dequeued_buffer[7];
      _outgoing_queue.pop_front(_dequeued_buffer, 7);
      switch ( _dequeued_buffer[1] ) {
        case GENIE_WRITE_CONTRAST: {
            writeMode(&_dequeued_buffer[1], 3); /* allow writing to any form pages. */
            // if ( _dequeued_buffer[0] == currentForm ) writeMode(&_dequeued_buffer[1], 3); /* only allow writes to current form */
            break;
          }
        case GENIE_READ_OBJ: { 
            writeMode(&_dequeued_buffer[1], 4);
            // if ( _dequeued_buffer[0] == currentForm ) writeMode(&_dequeued_buffer[1], 4);
            return;
          }
        case GENIE_WRITE_OBJ: {
            writeMode(&_dequeued_buffer[1], 6);
            // if ( _dequeued_buffer[0] == currentForm ) writeMode(&_dequeued_buffer[1], 6);
            break;
          }
      }
      pendingACK = 1;
      pendingACK_timeout = millis();
    }
  }
}

// ######################################
// ## Dequeue Event #####################
// ######################################
void Genie::DequeueEvent(genieFrame * buff) {
  if ( _incomming_queue.size() > 0) _incomming_queue.pop_front((uint8_t*)buff,6);
  event_frame = *buff;
}

// ######################################
// ## Write Strings #####################
// ######################################

bool Genie::WriteStr(uint8_t index, const char *string) {
  if ( !displayDetected ) {
    DoEvents();
    return 0;
  }
  uint8_t checksum = 0, buffer[4+strlen(string)];
  buffer[0] = GENIE_WRITE_STR;
  buffer[1] = index;
  buffer[2] = (uint8_t)strlen(string);
  memmove(&buffer[3],&string[0],strlen(string));
  for ( uint8_t i = 0; i < sizeof(buffer) - 1; i++ ) checksum ^= buffer[i];
  buffer[sizeof(buffer) - 1] = checksum;

  block_dequeue = 1; // disable dequeue
  while ( pendingACK ) DoEvents(); // wait pending ACKs
  writeMode(buffer,sizeof(buffer)); // write String
  pendingACK = 1; // enable ACK check
  pendingACK_timeout = millis(); // reset ACK check timer
  while ( pendingACK ) DoEvents(); // wait pending ACKs
  block_dequeue = 0; // re-enable dequeue

  return 1;
}

bool Genie::WriteStr(uint8_t index, String string) {
  return WriteStr(index, string.c_str());
}

#ifdef AVR
uint16_t Genie::WriteStr(uint16_t index, const __FlashStringHelper *ifsh){
	PGM_P p = reinterpret_cast<PGM_P>(ifsh);
	PGM_P p2 = reinterpret_cast<PGM_P>(ifsh);
	int len = 0;
	while (1) {
		unsigned char d = pgm_read_byte(p2++);
		len++;
		if (d == 0) break;
	}
  
 
	char arr[len];
	int x = 0;
	while (1) {
		unsigned char c = pgm_read_byte(p++);
		arr[x] = c;
		x++;
		if (c == 0) break;
	}
	return WriteStr(index, arr);
}
#endif

uint16_t Genie::WriteStr(uint16_t index, long n) { 
	char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
	char *str = &buf[sizeof(buf) - 1];
	
	long N = n;
	n = abs(n);

	*str = '\0';

	do {
		unsigned long m = n;
		n /= 10;
		char c = m - 10 * n;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while(n);
	
	if (N < 0) *--str = '-';
	
	return WriteStr(index, str);
}

uint16_t Genie::WriteStr(uint16_t index, long n, int base) { 
	char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
	char *str = &buf[sizeof(buf) - 1];
	
	long N;
	*str = '\0';
	if(n>=0) {
		// prevent crash if called with base == 1
		if (base < 2) base = 10;
		if(base == 10) {
			N = n;
			n = abs(n);
		}
	
		do {
			unsigned long m = n;
			n /= base;
			char c = m - base * n;
			*--str = c < 10 ? c + '0' : c + 'A' - 10;
		} while(n);
		
		if(base == 10) {
			if (N < 0) *--str = '-';
		}
			
	}
	
	else if(n<0) {
		unsigned long n2 = (unsigned long)n;
		uint8_t base2 = base;
		do {
		unsigned long m = n2;
		n2 /= base2;
		char c = m - base2 * n2;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
		} while(n2);
		
	}
    return WriteStr(index, str);
}

uint16_t Genie::WriteStr(uint16_t index, int n) { 
	return WriteStr (index, (long) n);
}

uint16_t Genie::WriteStr(uint16_t index, int n, int base) { 
	return WriteStr (index, (long) n, base);
}

uint16_t Genie::WriteStr(uint16_t index, unsigned long n) { 
	char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
	char *str = &buf[sizeof(buf) - 1];
	
	*str = '\0';

	do {
		unsigned long m = n;
		n /= 10;
		char c = m - 10 * n;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while(n);
	
	return WriteStr(index, str);
}

uint16_t Genie::WriteStr(uint16_t index, unsigned long n, int base) { 
	char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
	char *str = &buf[sizeof(buf) - 1];
	
	*str = '\0';

	// prevent crash if called with base == 1
	if (base < 2) base = 10;
	do {
		unsigned long m = n;
		n /= base;
		char c = m - base * n;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while(n);
				
    return WriteStr(index, str);
}

uint16_t Genie::WriteStr(uint16_t index, unsigned int n) { 
	return WriteStr (index, (unsigned long) n);
}

uint16_t Genie::WriteStr(uint16_t index, unsigned n, int base) { 
	return WriteStr (index, (unsigned long) n, base);
}


uint16_t Genie::WriteStr(uint16_t index, double number, int digits) { 
	char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
	char *str = &buf[sizeof(buf) - 1];
	*str = '\0';  

	double number2 = number;
	if (number < 0.0) number = -number;

	// Round correctly so that print(1.999, 2) prints as "2.00"
	double rounding = 0.5;
	for (int i=0; i<digits; ++i)
	rounding /= 10.0;

	number += rounding;

	unsigned long int_part = (unsigned long)number;
	double remainder = number - (double)int_part;

	// Extract digits from the remainder one at a time
	int digits2 = digits;
	str = &buf[sizeof(buf) - 1 - digits2];
	while (digits2-- > 0)
	{
	remainder *= 10.0;
	int toPrint = int(remainder);
	char c = toPrint + 48;
	*str++ = c;
	remainder -= toPrint; 
	}
	str = &buf[sizeof(buf) - 1 - digits];
	if (digits > 0) *--str = '.';

	// Extract the integer part of the number and print it  
	do {
	unsigned long m = int_part;
	int_part /= 10;
	char c = m - 10 * int_part;
	*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while(int_part);

	// Handle negative numbers
	if (number2 < 0.0) *--str = '-';

	return WriteStr(index, str);
}

uint16_t Genie::WriteStr(uint16_t index, double n){
	return WriteStr(index, n, 2);
}

/////////////////////// WriteStrU ////////////////////////
//
// Write a string to the display (Unicode)
// Unicode characters are 2 bytes each
//
uint16_t Genie::WriteStrU (uint16_t index, uint16_t *string) {
  uint16_t *p;
  unsigned int checksum = 0;
  int len = 0;
  p = string;
  while (*p++) len++;
  if (len > 255) return -1;

  uint8_t buffer[4+(len*2)];
  buffer[0] = GENIE_WRITE_STRU;
  buffer[1] = (uint8_t)index;
  buffer[2] = (uint8_t)(len);
  for ( uint8_t i = 0; i < 3; i++ ) checksum ^= buffer[i];

  p = string;
  int run = 2;

  while (*p) {
    run++;
    buffer[run] = (*p >> 8);
    checksum ^= *p >> 8;
    run++;
    buffer[run] = (*p);
    checksum ^= *p++ & 0xff;
  }
  buffer[(4+(len*2))-1] = checksum;

  block_dequeue = 1; // disable dequeue
  while ( pendingACK ) DoEvents(); // wait pending ACKs
  writeMode(buffer,sizeof(buffer)); // write String
  pendingACK = 1; // enable ACK check
  pendingACK_timeout = millis(); // reset ACK check timer
  while ( pendingACK ) DoEvents(); // wait pending ACKs
  block_dequeue = 0; // re-enable dequeue

  return 1;
}

// ######################################
// ## Write WriteInhLabel Strings #######
// ######################################

bool Genie::WriteInhLabel(uint8_t index, const char *string) {
  if ( !displayDetected ) {
    DoEvents();
    return 0;
  }
  uint8_t checksum = 0, buffer[4+strlen(string)];
  buffer[0] = GENIE_WRITE_INH_LABEL;
  buffer[1] = index;
  buffer[2] = (uint8_t)strlen(string);
  memmove(&buffer[3],&string[0],strlen(string));
  for ( uint8_t i = 0; i < sizeof(buffer) - 1; i++ ) checksum ^= buffer[i];
  buffer[sizeof(buffer) - 1] = checksum;

  block_dequeue = 1; // disable dequeue
  while ( pendingACK ) DoEvents(); // wait pending ACKs
  writeMode(buffer,sizeof(buffer)); // write String
  pendingACK = 1; // enable ACK check
  pendingACK_timeout = millis(); // reset ACK check timer
  while ( pendingACK ) DoEvents(); // wait pending ACKs
  block_dequeue = 0; // re-enable dequeue

  return 1;
}

bool Genie::WriteInhLabel(uint8_t index, String string) {
  return WriteInhLabel(index, string.c_str());
}

// ######################################
// ## Write WriteInhLabel Long ##########
// ######################################

uint16_t Genie::WriteInhLabel (uint16_t index) {
    return WriteObject(GENIE_OBJ_ILABELB, index, -1);
}

// ######################################
// ## Write WriteInhLabel Long ##########
// ######################################

uint16_t Genie::WriteInhLabel (uint16_t index, long n) { 
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  
  long N = n;
  n = abs(n);

  *str = '\0';

  do {
    unsigned long m = n;
    n /= 10;
    char c = m - 10 * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);
  
  if (N < 0) *--str = '-';
  
  return WriteInhLabel(index, str);
}

// ######################################
// ## Write WriteInhLabel Long w/Base ###
// ######################################

uint16_t Genie::WriteInhLabel (uint16_t index, long n, int base) { 
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  
  long N;
  *str = '\0';
  if(n>=0) {
    // prevent crash if called with base == 1
    if (base < 2) base = 10;
    if(base == 10) {
      N = n;
      n = abs(n);
    }
    do {
      unsigned long m = n;
      n /= base;
      char c = m - base * n;
      *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while(n);

    if(base == 10) {
      if (N < 0) *--str = '-';
    }
  }
  
  else if(n<0) {
    unsigned long n2 = (unsigned long)n;
    uint8_t base2 = base;
    do {
    unsigned long m = n2;
    n2 /= base2;
    char c = m - base2 * n2;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while(n2);
  }
  return WriteInhLabel(index, str);
}

// ######################################
// ## Write WriteInhLabel Int ###########
// ######################################

uint16_t Genie::WriteInhLabel (uint16_t index, int n) { 
  return WriteInhLabel (index, (long) n);
}

// ######################################
// ## Write WriteInhLabel Int w/Base ####
// ######################################

uint16_t Genie::WriteInhLabel (uint16_t index, int n, int base) { 
  return WriteInhLabel (index, (long) n, base);
}

// ######################################
// ## Write WriteInhLabel UL ############
// ######################################

uint16_t Genie::WriteInhLabel (uint16_t index, unsigned long n) { 
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  
  *str = '\0';

  do {
    unsigned long m = n;
    n /= 10;
    char c = m - 10 * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);
  
  return WriteInhLabel(index, str);
}

// ######################################
// ## Write WriteInhLabel UL w/Base #####
// ######################################

uint16_t Genie::WriteInhLabel (uint16_t index, unsigned long n, int base) { 
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  
  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;
  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);
        
  return WriteInhLabel(index, str);
}

uint16_t Genie::WriteInhLabel (uint16_t index, unsigned int n) { 
  return WriteInhLabel (index, (unsigned long) n);
}

uint16_t Genie::WriteInhLabel (uint16_t index, unsigned n, int base) { 
  return WriteInhLabel (index, (unsigned long) n, base);
}



// ######################################
// ## Write WriteInhLabel Floats #######
// ######################################

uint16_t Genie::WriteInhLabel (uint16_t index, double number, int digits) {
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  *str = '\0';

  double number2 = number;
  if (number < 0.0) number = -number;

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (int i = 0; i < digits; ++i) rounding /= 10.0;

  number += rounding;

  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;

  // Extract digits from the remainder one at a time
  int digits2 = digits;
  str = &buf[sizeof(buf) - 1 - digits2];
  while (digits2-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    char c = toPrint + 48;
    *str++ = c;
    remainder -= toPrint;
  }
  str = &buf[sizeof(buf) - 1 - digits];
  if (digits > 0) *--str = '.';

  // Extract the integer part of the number and print it
  do {
    unsigned long m = int_part;
    int_part /= 10;
    char c = m - 10 * int_part;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (int_part);

  // Handle negative numbers
  if (number2 < 0.0) *--str = '-';

  return WriteInhLabel(index, str);
}

// ######################################
// ## Write WriteInhLabel FlashString ###
// ######################################

#ifdef AVR
uint16_t Genie::WriteInhLabel(uint16_t index, const __FlashStringHelper *ifsh) {
  PGM_P p = reinterpret_cast<PGM_P>(ifsh);
  PGM_P p2 = reinterpret_cast<PGM_P>(ifsh);
  int len = 0;
  while (1) {
    unsigned char d = pgm_read_byte(p2++);
    len++;
    if (d == 0) break;
  }
  char arr[len];
  int x = 0;
  while (1) {
    unsigned char c = pgm_read_byte(p++);
    arr[x] = c;
    x++;
    if (c == 0) break;
  }
  return WriteInhLabel(index, arr);
}
#endif

uint8_t Genie::EventIs(genieFrame * e, uint8_t cmd, uint8_t object, uint8_t index) {
  return (e->reportObject.cmd == cmd && e->reportObject.object == object && e->reportObject.index == index);
}

uint16_t Genie::GetEventData(genieFrame * e) {
  return (e->reportObject.data_msb << 8) + e->reportObject.data_lsb;
}

// ######################################
// ## Write Magic Bytes #################
// ######################################

int8_t Genie::WriteMagicBytes(uint8_t index, uint8_t *bytes, uint8_t len, uint8_t report) {
  if ( !displayDetected ) {
    DoEvents();
    return -1;
  }
  uint8_t checksum = 0, buffer[4 + len];
  buffer[0] = GENIEM_WRITE_BYTES;
  buffer[1] = index;
  buffer[2] = len;
  memmove(&buffer[3], bytes, len);
  for ( uint8_t i = 0; i < sizeof(buffer) - 1; i++ ) checksum ^= buffer[i];
  buffer[sizeof(buffer) - 1] = checksum;

  block_dequeue = 1;
  while ( pendingACK ) DoEvents();
  writeMode(buffer, sizeof(buffer));

  if ( report ) {
    while ( pendingACK ) DoEvents();
    block_dequeue = 0;
    return 1;
  }

  uint32_t timeout = millis();
  while (millis() - timeout < 1000) {
    uint8_t result = DoEvents();
    switch ( result ) {
      case GENIEM_REPORT_BYTES: {
          block_dequeue = 0;
          return result;
        }
      case GENIE_ACK: {
          block_dequeue = 0;
          return result;
        }
      case GENIE_NAK: {
          block_dequeue = 0;
          return result;
        }
    }
  }
  block_dequeue = 0;
  return -1;
}

// ######################################
// ## Write Magic Double Bytes ##########
// ######################################

int8_t Genie::WriteMagicDBytes(uint8_t index, uint16_t *shorts, uint8_t len, uint8_t report) {
  if ( !displayDetected ) {
    DoEvents();
    return -1;
  }
  uint8_t checksum = 0, buffer[4+len*2];
  buffer[0] = GENIEM_WRITE_DBYTES;
  buffer[1] = index;
  buffer[2] = len;
  bool odd_or_even = ( len % 2 );
  for ( uint16_t i = 0, j = 0; i < len; i++ ) {
    buffer[j+3] = shorts[i] >> 8;
    buffer[j+4] = (uint8_t)shorts[i];
    j+=2;
  }
  if ( odd_or_even ) buffer[sizeof(buffer)-1] = shorts[len-2];
  for ( uint8_t i = 0; i < sizeof(buffer) - 1; i++ ) checksum ^= buffer[i];
  buffer[sizeof(buffer) - 1] = checksum;

  block_dequeue = 1;
  while ( pendingACK ) DoEvents();
  writeMode(buffer, sizeof(buffer));

  if ( report ) {
    block_dequeue = 0;
    while ( pendingACK ) DoEvents();
    return 1;
  }

  uint32_t timeout = millis();
  while (millis() - timeout < 1000) {
    uint8_t result = DoEvents();
    switch ( result ) {
      case GENIEM_REPORT_BYTES: {
          block_dequeue = 0;
          return result;
        }
      case GENIE_ACK: {
          block_dequeue = 0;
          return result;
        }
      case GENIE_NAK: {
          block_dequeue = 0;
          return result;
        }
    }
  }
  block_dequeue = 0;
  return -1;
}





// ######################################
// ## GenieObject Class #################
// ######################################

GenieObject::GenieObject(Genie& _instance, uint8_t obj, uint8_t idx) {
  object = obj;
  index = idx;
  instance = &_instance;
}

// ######################################
// ## GenieObject read ##################
// ######################################

int32_t GenieObject::read(bool state) {
  return instance->ReadObject(object, index, state);
}

// ######################################
// ## GenieObject write #################
// ######################################

void GenieObject::write(uint16_t data) {
  instance->WriteObject(object, index, data);
}

void GenieObject::write(const char * data) {
  if ( object == GENIE_WRITE_STR ) instance->WriteStr(index, data);
  else if ( object == GENIE_WRITE_INH_LABEL ) instance->WriteInhLabel(index, data);
}