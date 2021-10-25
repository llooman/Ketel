#ifndef _Ketel_H_
#define _Ketel_H_

// #define DEBUG

#define JL_VERSION 20000
/****************************
2021-02-05 2.0.0 ketel usb: thermo & boilerOnOff
2021-02-03 refactor netw prep 2.0.0
2020-09-27 1.3.2 add tempWW
				 when ww stop timmings for verwarming.	

2020-09-18 1.3.1 refactor to myTimers and Relais2

2017-05-04 TWI_FREQ 200000L

2017-01-12 TWI_FREQ 100000L

2017-01-12 TWI_FREQ 25000L

2017-02-11 bootPeriode3
		   hang if sentHealthCount > 3
           NetwBase
           parentNode.TwoWire::setTimeout(1);
           if(req->data.cmd == 'S') msgSize = sizeof(Set);
           perpare 'S' 'E'

2017-01-17 new Netw.h

2017-01-15 new Parms
           Netw.h i2c 50KHz

2017-01-11 add bootCount
2017-01-10 retry
		   HomeNetH.h 2017-01-10  refactor bootTimer
		   DS18B20.h  2017-01-10  sendCurrentTemp retry
		   Relais.h   2017-01-10  sendDataWhenAvailable retry
2017-01-09 new EEProm class replacing EEParms and Parms
		   System::ramFree()
		   set netw.netwTimer   moved to NETW class
		   HomeNetH.h 2017-01-10  netw.sendData retry on send error
2017-01-08 HomeNetH.h 2017-01-08 fix volatile
2017-01-07 add netw.execParms(cmd, id, value) in execParms
           add if(req->data.cmd != 21 || parms.parmsUpdateFlag==0 ) in loop.handle netw request
           move parms.loop(); before loop.handle netw request
           HomeNetH.h 2017-01-06
2017-01-07 TODO eeparms.exeParms  case 3: netw.netwTimer = millis() + (value * 1000); //  netwSleep

 the  chint from the rs232 !! the upload will fail when the chint is still connected
 // Ketel v 5.2  2016-06-02 device/id
 // Ketel v 5.0  2016-04-03 parms class
 // Ketel v 1.73 2015-12-05 aync DS18B20
 // Ketel v 1.72 2015-11-27 add update
 // Ketel v 1.71 2015-11-07 event queue + insert of update + val val2 en val3, type 4
 // Ketel v 1.7  2015-11-01 temp events using the DS18B20.h
 // Ketel v 1.61 2015-09-20 latest EEProm + ic2 timmings
 // Ketel v 1.6  2015-08-22 latest HomeNet
 // Ketel v 1.5  2015-04-04 add test pin 12, use last Chint.h skip, autoReboot
 // Ketel v 1.4  2015-01-21 add last error
 // Ketel v 1.3  2014-06-22 timer & read > chint.h, & use errorCnt and retCode from object
 // Ketel v 1.2  2014-06-14 retries in homenet and here
 // Ketel v 1.1  2014-06-14 kwaliteit meeting return min
 // Ketel v 1.0  2014-05-18 copy of PompSchakelaar v2.3
 */

#include "Arduino.h"
#include <inttypes.h>
#include <DS18B20.h>
#include "EEUtil.h"
#include <MyTimers.h>
#include <avr/wdt.h>

#include <NetwTWI.h>
#include <Relais2.h>

#define NODE_ID 		5
#define NODE_TYPE 		5

#define NUMBER_OF_DUTY_LEVELS 10

#define VERWARMING_PIN  4
#define TEMP_PIN 		3
#define BOILER_PIN 		5

#define BOILER_ID 		30

#define TEMP_CV 			41
// #define TEMP_BOILER 		47
#define TEMP_WW 			48

#define TEMP_RETOUR_BAD		24
#define TEMP_RETOUR_RECHTS	25
#define TEMP_RETOUR_LINKS	26


#define VERWARMEN_ID       	60

#define VERWARMING_ACTIVE_ID 52

#define TWI_PARENT
NetwTWI parentNode;

//#define SPI_PARENT
//#include <NetwSPI.h>
//NetwSPI parentNode(NODE_ID);

#define TRACE_SEC 3

const byte ds103[] PROGMEM = {0x28, 0x2f, 0xec, 0x54, 0x05, 0x00, 0x00, 0x94};  // waterdicht
const byte ds01[]  PROGMEM = {0x28, 0x80, 0x7e, 0x00, 0x05, 0x00, 0x00, 0x0e};
const byte ds105[] PROGMEM = {0x28, 0xFF, 0x37, 0x36, 0x66, 0x14, 0x03, 0x9D};  //28 FF 37 36 66 14 3 9D
const byte ds108[] PROGMEM = {0x28, 0xff, 0x5e, 0x97, 0x66, 0x14, 0x02, 0x07};  //28 FF 5E 97 66 14 2 7
const byte ds19[]  PROGMEM = {0x28, 0xff, 0x1c, 0x5e, 0x00, 0x16, 0x01, 0xc0};  // 28 FF 1C 5E 0 16 1 C0

const byte dsKraan[] PROGMEM = {0x28, 0xff, 0xab, 0x19, 0xa0, 0x16, 0x03, 0xf2};  //28 FF AB 19 A0 16 3 F2
const byte ds104[] PROGMEM = {0x28, 0xde, 0xc3, 0x57, 0x05, 0x00, 0x00, 0xce};  // waterdicht
const byte ds107[] PROGMEM = {0x28, 0x37, 0xDB, 0x8C, 0x06, 0x00, 0x00, 0x42};  // waterdicht zelf   28 37 DB 8C 6 0 0 42

#ifdef DEBUG
	DS18B20 tempCV( TEMP_PIN     );  //, ds101  ds103 , ( test=ds102 ) 3
#else
	DS18B20 tempCV( TEMP_PIN, ds103 );  //  ds103 TEMP_SENSOR_CV
	DS18B20 tempWW (TEMP_PIN, ds19 );  //

	DS18B20 tempRetourBad	(TEMP_PIN, ds01 );  //  , ds104  rood
	DS18B20 tempRetourRechts(TEMP_PIN, ds105 );  //  , ds104  rood
	DS18B20 tempRetourLinks	(TEMP_PIN, ds108 );  //  , ds104  rood

	DS18B20 tempBuiten(TEMP_PIN, ds107);  //, ds107  wit , 21
	DS18B20 tempDouch( TEMP_PIN, ds104);  // ds04, ds103 TEMP_SENSOR_CV , 22
	DS18B20 tempKraan( TEMP_PIN, dsKraan);  //  ds103 TEMP_SENSOR_CV , 23
#endif

 
Relais2 verwarming( VERWARMING_PIN ); // a relais is used to simulatie the aan/uit thermostaat
 


#define TIMERS_COUNT 7
MyTimers myTimers(TIMERS_COUNT);
#define TIMER_TRACE 0
#define TIMER_UPLOAD_LED 1
#define TIMER_UPLOAD_ON_BOOT 2
#define TIMER_UPLOAD_VERWARMING_ACTIVE 3
#define TIMER_UPLOAD_POWER 4
#define TIMER_PAUSE 5
#define TIMER_UPLOAD_BOILER 6

 
int 	uploadOnBootCount=0; 
int 	power = 0;
bool	isVerwarmingAktief = false;


int evaluateVerwarming(bool on, bool isRunning);
void localSetVal(int id, long val);
int  upload(int id, long val, unsigned long timeStamp);
int  upload(int id, long val) ;
int  upload(int id);
int  uploadError(int id, long val);
int  handleParentReq( RxItem *rxItem) ;
int  localRequest(RxItem *rxItem);
void trace();

#endif