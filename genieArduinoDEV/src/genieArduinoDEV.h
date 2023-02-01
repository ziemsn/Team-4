/////////////////////// GenieArduino DEV ///////////////////////
//
//      Library to utilize the 4D Systems Genie interface to displays
//      that have been created using the Visi-Genie creator platform.
//      This is intended to be used with the Arduino platform.
//
//      Improvements/Updates by
//        Antonio Brewer & 4D Systems Engineering, May 2022, www.4dsystems.com.au
//        Antonio Brewer & 4D Systems Engineering, February 2022, www.4dsystems.com.au
//        Antonio Brewer & 4D Systems Engineering, January 2022, www.4dsystems.com.au
//        Antonio Brewer & 4D Systems Engineering, July 2021, www.4dsystems.com.au
//        Antonio Brewer & 4D Systems Engineering, June 2018, www.4dsystems.com.au
//        4D Systems Engineering, August 2017, www.4dsystems.com.au
//        Antonio Brewer & 4D Systems Engineering, July 2017, www.4dsystems.com.au
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
//      Copyright (c) 2012-2022 4D Systems Pty Ltd, Sydney, Australia
/*********************************************************************
   This file is part of genieArduino:
      genieArduino is free software: you can redistribute it and/or modify
      it under the terms of the GNU Lesser General Public License as
      published by the Free Software Foundation, either version 3 of the
      License, or (at your option) any later version.

      genieArduino is distributed in the hope that it will be useful,
      but WITHOUT ANY WARRANTY; without even the implied warranty of
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
      GNU Lesser General Public License for more details.

      You should have received a copy of the GNU Lesser General Public
      License along with genieArduino.
      If not, see <http://www.gnu.org/licenses/>.
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

#include <inttypes.h>
#include "genie_buffer.h"
#include <stdint.h>

#define GENIE_SS_SUPPORT !defined(ARDUINO_ARCH_SAM) \
                      && !defined(ARDUINO_ARCH_SAMD) \
                      && !defined(ARDUINO_ARCH_RP2040) \
                      && !defined(ESP32) \
                      && !defined(ESP8266)
                      // This lists the known board families that
                      // don't support SoftwareSerial.
                      // If you add to this list, please contact us
                      // to gain official support in the library.

#if GENIE_SS_SUPPORT
#include <SoftwareSerial.h>
#endif

#ifndef genieArduinoDEV_h
#define genieArduinoDEV_h

#define GENIE_VERSION    "GenieArduino 2022"   // DD-MM-YYYY

// Genie commands & replys:

#define GENIE_ACK               0x06
#define GENIE_NAK               0x15
#define GENIE_PING              0x80
#define GENIE_READY             0x81
#define GENIE_DISCONNECTED      0x82

#define GENIE_READ_OBJ          0
#define GENIE_WRITE_OBJ         1
#define GENIE_WRITE_STR         2
#define GENIE_WRITE_STRU        3
#define GENIE_WRITE_CONTRAST    4
#define GENIE_REPORT_OBJ        5
#define GENIE_REPORT_EVENT      7
#define GENIEM_WRITE_BYTES      8
#define GENIEM_WRITE_DBYTES     9
#define GENIEM_REPORT_BYTES     10
#define GENIEM_REPORT_DBYTES    11
#define GENIE_WRITE_INH_LABEL   12

// Objects
//    the manual says:
//        Note: Object IDs may change with future releases; it is not
//        advisable to code their values as constants.

#define GENIE_OBJ_DIPSW                 0
#define GENIE_OBJ_KNOB                  1
#define GENIE_OBJ_ROCKERSW              2
#define GENIE_OBJ_ROTARYSW              3
#define GENIE_OBJ_SLIDER                4
#define GENIE_OBJ_TRACKBAR              5
#define GENIE_OBJ_WINBUTTON             6
#define GENIE_OBJ_ANGULAR_METER         7
#define GENIE_OBJ_COOL_GAUGE            8
#define GENIE_OBJ_CUSTOM_DIGITS         9
#define GENIE_OBJ_FORM                  10
#define GENIE_OBJ_GAUGE                 11
#define GENIE_OBJ_IMAGE                 12
#define GENIE_OBJ_KEYBOARD              13
#define GENIE_OBJ_LED                   14
#define GENIE_OBJ_LED_DIGITS            15
#define GENIE_OBJ_METER                 16
#define GENIE_OBJ_STRINGS               17
#define GENIE_OBJ_THERMOMETER           18
#define GENIE_OBJ_USER_LED              19
#define GENIE_OBJ_VIDEO                 20
#define GENIE_OBJ_STATIC_TEXT           21
#define GENIE_OBJ_SOUND                 22
#define GENIE_OBJ_TIMER                 23
#define GENIE_OBJ_SPECTRUM              24
#define GENIE_OBJ_SCOPE                 25
#define GENIE_OBJ_TANK                  26
#define GENIE_OBJ_USERIMAGES            27
#define GENIE_OBJ_PINOUTPUT             28
#define GENIE_OBJ_PININPUT              29
#define GENIE_OBJ_4DBUTTON              30
#define GENIE_OBJ_ANIBUTTON             31
#define GENIE_OBJ_COLORPICKER           32
#define GENIE_OBJ_USERBUTTON            33
#define GENIE_OBJ_MAGICOBJECT           34
#define GENIE_OBJ_SMARTGAUGE            35
#define GENIE_OBJ_SMARTSLIDER           36
#define GENIE_OBJ_SMARTKNOB             37
// Not advisable to use the below 3, use the above 3 instead.
#define GENIE_OBJ_ISMARTGAUGE           35 // Retained for backwards compatibility, however Users should use SMARTGAUGE instead of ISMARTGAUGE
#define GENIE_OBJ_ISMARTSLIDER          36 // Retained for backwards compatibility, however Users should use SMARTSLIDER instead of ISMARTSLIDER
#define GENIE_OBJ_ISMARTKNOB            37 // Retained for backwards compatibility, however Users should use SMARTKNOB instead of ISMARTKNOB
// Comment end
#define GENIE_OBJ_ILED_DIGITS_H         38
#define GENIE_OBJ_IANGULAR_METER        39
#define GENIE_OBJ_IGAUGE                40
#define GENIE_OBJ_ILABELB               41
#define GENIE_OBJ_IUSER_GAUGE           42
#define GENIE_OBJ_IMEDIA_GAUGE          43
#define GENIE_OBJ_IMEDIA_THERMOMETER    44
#define GENIE_OBJ_ILED                  45
#define GENIE_OBJ_IMEDIA_LED            46
#define GENIE_OBJ_ILED_DIGITS_L         47
#define GENIE_OBJ_ILED_DIGITS           47
#define GENIE_OBJ_INEEDLE               48
#define GENIE_OBJ_IRULER                49
#define GENIE_OBJ_ILED_DIGIT            50
#define GENIE_OBJ_IBUTTOND              51
#define GENIE_OBJ_IBUTTONE              52
#define GENIE_OBJ_IMEDIA_BUTTON         53
#define GENIE_OBJ_ITOGGLE_INPUT         54
#define GENIE_OBJ_IDIAL                 55
#define GENIE_OBJ_IMEDIA_ROTARY         56
#define GENIE_OBJ_IROTARY_INPUT         57
#define GENIE_OBJ_ISWITCH               58
#define GENIE_OBJ_ISWITCHB              59
#define GENIE_OBJ_ISLIDERE              60
#define GENIE_OBJ_IMEDIA_SLIDER         61
#define GENIE_OBJ_ISLIDERH              62
#define GENIE_OBJ_ISLIDERG              63
#define GENIE_OBJ_ISLIDERF              64
#define GENIE_OBJ_ISLIDERD              65
#define GENIE_OBJ_ISLIDERC              66
#define GENIE_OBJ_ILINEAR_INPUT         67

// Do not modify current values. Recommended settings.


#define DISPLAY_TIMEOUT         3000
#define AUTO_PING_CYCLE         1250


// Structure to store replys returned from a display

#define GENIE_FRAME_SIZE        6 // do NOT touch this.

struct FrameReportObj {
  uint8_t   cmd;
  uint8_t   object;
  uint8_t   index;
  uint8_t   data_msb;
  uint8_t   data_lsb;
};

struct MagicReportHeader {
  uint8_t   cmd;
  uint8_t   index;
  uint8_t   length;
};

union FloatLongFrame {
  float     floatValue;
  int32_t   longValue;
  uint32_t  ulongValue;
  int16_t   wordValue[2];
};

/////////////////////////////////////////////////////////////////////
// The Genie frame definition
//
// The union allows the data to be referenced as an array of uint8_t
// or a structure of type FrameReportObj, eg
//
//    genieFrame f;
//    f.bytes[4];
//    f.reportObject.data_lsb
//
//    both methods get the same byte
//
union genieFrame {
  uint8_t         bytes[GENIE_FRAME_SIZE] = { 0 };
  FrameReportObj  reportObject;
};

#define MAX_GENIE_EVENTS    16      // MUST be a power of 2

struct EventQueueStruct {
  genieFrame  frames[MAX_GENIE_EVENTS];
  uint8_t     rd_index = 0;
  uint8_t     wr_index = 0;
  uint8_t     n_events = 0;
};

typedef void  (*UserEventHandlerPtr) (void);
typedef void  (*UserBytePtr)(uint8_t, uint8_t);
typedef void  (*UserDoubleBytePtr)(uint8_t, uint8_t);

/////////////////////////////////////////////////////////////////////
// User API functions
// These function prototypes are the user API to the library
//
class Genie {
  public:
    Genie_Buffer < uint8_t, (uint32_t)pow(2, ceil(log(MAX_GENIE_EVENTS) / log(2))), 6 > _incomming_queue; /* currentForm, cmd, object, index, data1, data2 */
    Genie_Buffer < uint8_t, (uint32_t)pow(2, ceil(log(MAX_GENIE_EVENTS) / log(2))), 7 > _outgoing_queue; /* currentForm, cmd, object, index, data1, data2, crc */
    Genie                                     ();
#if GENIE_SS_SUPPORT
    bool          Begin                       (SoftwareSerial &serial);
#endif
    bool          Begin                       (HardwareSerial &serial);
    bool          Begin                       (Stream &serial, uint16_t txDelay = 0);
    void          AttachDebugStream           (Stream &serial);
    bool          IsOnline                    ();
    int16_t       GetForm                     ();
    void          SetForm                     (uint8_t newForm);
    void          SetRecoveryInterval         (uint8_t pulses);
    int32_t       ReadObject                  (uint8_t object, uint8_t index, bool now = 0);
    bool          WriteObject                 (uint8_t object, uint8_t index, uint16_t data);
    uint16_t      WriteIntLedDigits           (uint16_t index, int16_t data);
    uint16_t      WriteIntLedDigits           (uint16_t index, float data);
    uint16_t      WriteIntLedDigits           (uint16_t index, int32_t data);
    bool          WriteContrast               (uint8_t value);
    bool          WriteStr                    (uint8_t index, const char *string);
    bool          WriteStr                    (uint8_t index, String string);
    uint16_t      WriteStr                    (uint16_t index, long n) ;
    uint16_t      WriteStr                    (uint16_t index, long n, int base) ;
    uint16_t      WriteStr                    (uint16_t index, unsigned long n) ;
    uint16_t      WriteStr                    (uint16_t index, unsigned long n, int base) ;
    uint16_t      WriteStr                    (uint16_t index, int n) ;
    uint16_t      WriteStr                    (uint16_t index, int n, int base) ;
    uint16_t      WriteStr                    (uint16_t index, unsigned int n) ;
    uint16_t      WriteStr                    (uint16_t index, unsigned int n, int base) ;
#ifdef AVR
    uint16_t      WriteStr                    (uint16_t index, const __FlashStringHelper *ifsh);
#endif
    uint16_t      WriteStr                    (uint16_t index, double n, int digits);
    uint16_t      WriteStr                    (uint16_t index, double n);
    uint16_t      WriteStrU                   (uint16_t index, uint16_t *string);
    bool          WriteInhLabel               (uint8_t index, const char *string);
    bool          WriteInhLabel               (uint8_t index, String string);
    uint16_t      WriteInhLabel               (uint16_t index);
    uint16_t      WriteInhLabel               (uint16_t index, long n) ;
    uint16_t      WriteInhLabel               (uint16_t index, long n, int base) ;
    uint16_t      WriteInhLabel               (uint16_t index, unsigned long n) ;
    uint16_t      WriteInhLabel               (uint16_t index, unsigned long n, int base) ;
    uint16_t      WriteInhLabel               (uint16_t index, int n) ;
    uint16_t      WriteInhLabel               (uint16_t index, int n, int base) ;
    uint16_t      WriteInhLabel               (uint16_t index, unsigned int n) ;
    uint16_t      WriteInhLabel               (uint16_t index, unsigned int n, int base) ;
    uint16_t      WriteInhLabel               (uint16_t index, double n, int digits = 2);
#ifdef AVR
    uint16_t      WriteInhLabel               (uint16_t index, const __FlashStringHelper *ifsh);
#endif
    uint8_t       EventIs                     (genieFrame * e, uint8_t cmd, uint8_t object, uint8_t index);
    uint16_t      GetEventData                (genieFrame * e);
    void          DequeueEvent                (genieFrame * buff);
    int16_t       DoEvents                    ();
    void          Ping                        (uint16_t interval);
    void          AttachEventHandler          (UserEventHandlerPtr userHandler);
    void          AttachMagicByteReader       (UserBytePtr userHandler);
    void          AttachMagicDoubleByteReader (UserDoubleBytePtr userHandler);
    uint32_t      GetUptime                   ();

    // Genie Magic functions (ViSi-Genie Pro Only)

    int8_t        WriteMagicBytes             (uint8_t index, uint8_t *bytes, uint8_t len, uint8_t report = 0);
    int8_t        WriteMagicDBytes            (uint8_t index, uint16_t *bytes, uint8_t len, uint8_t report = 0);
    int16_t       GetNextByte                 ();
    int32_t       GetNextDoubleByte           ();

  protected:
  private:
    //////////////////////////////////////////////////////////////
    // A structure to hold up to MAX_GENIE_EVENTS events receive
    // from the display
    //
    EventQueueStruct EventQueue;

    Stream* deviceSerial;
    Stream* debugSerial;

    UserEventHandlerPtr UserHandler;
    UserBytePtr UserByteReader;
    UserDoubleBytePtr UserDoubleByteReader;

    bool          WriteObjectPriority         (uint8_t object, uint8_t index, uint16_t data);
    void          writeMode                   (uint8_t *bytes, uint8_t len);
    bool          Begin_common                ();

    // used internally by the library, do not modify!
    bool          pendingACK = 0;
    uint32_t      pendingACK_timeout = 0;
    int16_t       currentForm = -1;
    uint32_t      autoPingTimer = millis();
    bool          autoPingFlag = 0;
    bool          NAK_detected = 0;
    uint8_t       NAK_recovery_counter = 0;
    bool          displayDetected = 0;
    bool          pingRequest = 0;
    uint32_t      pingResponse = 0;
    uint32_t      pingSpacer = 0;
    uint8_t       recover_pulse = 50;
    uint32_t      display_uptime = 0;
    bool          genieStart = 1;
    bool          block_dequeue = 0;
    void          dequeue_processing();
    uint8_t       magic_report_len = 0;
    bool          main_handler_active = 0;
    bool          handler_response_request = 0;
    uint8_t       handler_response_values[6];
    uint8_t       magic_overpull_count = 0;
    uint16_t      tx_delay = 0;
    genieFrame    event_frame;
    friend class  GenieObject;
};

class GenieObject {
  public:
    GenieObject   (Genie& _instance, uint8_t obj, uint8_t idx);
    int32_t read  (bool state = 1);
    void write    (uint16_t data);
    void write    (const char * data);

  private:
    uint8_t object = 0;
    uint8_t index = 0;
    Genie* instance = nullptr;
};

#endif
