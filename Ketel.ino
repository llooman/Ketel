#include "Arduino.h"   // !!! DISCONNECT CHINT RS232  !!!   nano optiboot  COM9

//#define DEBUG

//#define DEBUG_TEMP_41

#define OTHER_TEMPS

#define NODE_ID 5
#define THERMOSTAAT_PIN  4
#define TEMP_PIN 3

//#define SERIAL_PARENT
//#include <NetwSerial.h>
//NetwSerial parentNode;

//
//#define SERIAL_CONSOLE 77
//#include <NetwSerial.h>
//NetwSerial console;


#define TWI_PARENT
#include <NetwTWI.h>
NetwTWI parentNode;

//#define SPI_PARENT
//#include <NetwSPI.h>
//NetwSPI parentNode(NODE_ID);



#define TRACE_MSEC 3000

/****************************

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


#include <DS18B20.h>
#include <Relais.h>    // library for switching on value + some safety
#include <ArdUtils.h>      // utilties
//#include <Voltage.h>


const byte ds04[] PROGMEM = {0x28, 0x2b, 0x57, 0x2b, 0x05, 0x00, 0x00, 0x45};	// zwart rood
const byte ds05[] PROGMEM = {0x28, 0x53, 0x06, 0x01, 0x05, 0x00, 0x00, 0x4f};   // geel groen
const byte ds103[] PROGMEM = {0x28, 0x2f, 0xec, 0x54, 0x05, 0x00, 0x00, 0x94};  // waterdicht
const byte ds104[] PROGMEM = {0x28, 0xde, 0xc3, 0x57, 0x05, 0x00, 0x00, 0xce};  // waterdicht
const byte ds107[] PROGMEM = {0x28, 0x37, 0xDB, 0x8C, 0x06, 0x00, 0x00, 0x42};  // waterdicht zelf   28 37 DB 8C 6 0 0 42

const byte ds01[] PROGMEM = {0x28, 0x80, 0x7e, 0x00, 0x05, 0x00, 0x00, 0x0e};
const byte ds105[] PROGMEM = {0x28, 0xFF, 0x37, 0x36, 0x66, 0x14, 0x03, 0x9D};  //28 FF 37 36 66 14 3 9D
const byte ds108[] PROGMEM = {0x28, 0xff, 0x5e, 0x97, 0x66, 0x14, 0x02, 0x07};  //28 FF 5E 97 66 14 2 7

#ifdef DEBUG_TEMP_41
DS18B20 tempCV( TEMP_PIN, 41   );  //, ds101  ds103 , ( test=ds102 ) 3
#else
DS18B20 tempCV( TEMP_PIN, ds103, 41);  //  ds103 TEMP_SENSOR_CV
#endif
//dsdsdfsfd

#ifdef OTHER_TEMPS
DS18B20 tempBoiler		(TEMP_PIN, ds104, 47);  //  , ds104  rood
DS18B20 tempBuiten		(TEMP_PIN, ds107, 43);  //, ds107  wit

DS18B20 tempRetourBad	(TEMP_PIN, ds01, 24);  //  , ds104  rood
DS18B20 tempRetourRechts(TEMP_PIN, ds105, 25);  //  , ds104  rood
DS18B20 tempRetourLinks	(TEMP_PIN, ds108, 26);  //  , ds104  rood

#endif

ArdUtils util ;                  // utilities

#ifdef THERMOSTAAT_PIN
Relais thermostaat( THERMOSTAAT_PIN, false, 52); // a relais is used to simulatie the aan/uit thermostaat
boolean thermostaatActive = false;
unsigned long thermostaatTimer;
#endif

unsigned long traceTimer_t = TRACE_MSEC;


class EEParms2: public EEProm    // TODO
{
public:
	volatile int thermostaatPeriode_min = 15;
	long chk1  = 0x00010203;


	EEParms2() { }
	virtual ~EEParms2(){ }

	void setThermostaatPeriode_min( long newVal)
	{
		thermostaatPeriode_min = (int) newVal;
		if(thermostaatPeriode_min < 5) thermostaatPeriode_min = 5;
		if(thermostaatPeriode_min > 30) thermostaatPeriode_min = 30;
		changed = true;
	}

    void setup( ) //
    {
		if( readLong(offsetof(EEParms2, chk1)) == chk1  )
    	{
    		readAll();
    		setThermostaatPeriode_min( readInt(offsetof(EEParms2, thermostaatPeriode_min)));
    		changed = false;
			#ifdef DEBUG
				Serial.println(F("EEProm.readAll"));
			#endif
    	}
		else
		{
			bootCount=0;
		}
    }

	void loop()
	{
		if(changed)
		{
			#ifdef DEBUG
				Serial.println(F("Parms.loop.changed"));
			#endif
			write(offsetof(EEParms2, thermostaatPeriode_min), thermostaatPeriode_min);
			write(offsetof(EEParms2, chk1), chk1);
			EEProm::writeAll();
			changed = false;
		}

		EEProm::loop();
	}
};
EEParms2 eeParms2;

#ifdef SPI_CONSOLE
	ISR (SPI_STC_vect){	console.handleInterupt();}
#endif
#ifdef	TWI_PARENT
	ISR(TWI_vect){ parentNode.tw_int(); }
#endif
#ifdef SPI_PARENT
	ISR (SPI_STC_vect){parentNode.handleInterupt();}
#endif

int localSetVal(int id, long val)
{
	switch(id )
	{

	#ifdef THERMOSTAAT_PIN
		case 52: if(val > 0) thermostaat.on(); else thermostaat.off();  break;
		case 53: eeParms2.setThermostaatPeriode_min(val); 				break;
	#endif

	default:
		eeParms2.setVal( id,  val);
		parentNode.setVal( id,  val);
		break;
	}

	if(id==2)
	{
		tempCV.setSyncInterval(eeParms2.samplePeriode_sec);
		#ifdef OTHER_TEMPS
			tempBoiler.setSyncInterval(eeParms2.samplePeriode_sec);
			tempBuiten.setSyncInterval(eeParms2.samplePeriode_sec);
		#endif
		thermostaat.syncInterval = eeParms2.samplePeriode_sec * 1000L;  // TODO check if  working ok
	}
}


int upload(int id)
{
	int ret = 0;

	switch( id )
	{
	case 41: tempCV.upload();  		break;

	#ifdef VOLTAGE_PIN
		case 11: upload(id, vin.val);    		break;
	#endif

	#ifdef THERMOSTAAT_PIN
		case 52: thermostaat.upload() ;   	break;
		case 53: upload(id, eeParms2.thermostaatPeriode_min);  	break;
	#endif

	#ifdef OTHER_TEMPS
		case 24: tempRetourBad.upload(); break;
		case 25: tempRetourRechts.upload(); break;
		case 26: tempRetourLinks.upload(); break;
		case 43: tempBuiten.upload(); break;
		case 47: tempBoiler.upload(); break;
	#endif

	default:
		if( 1==2
		 ||	parentNode.upload(id)>0
		 ||	eeParms2.upload(id)>0
		){}
		break;
	}
	return ret;
}

int upload(int id, long val) { return upload(id, val, millis()); }
int upload(int id, long val, unsigned long timeStamp)
{
	return parentNode.txUpload(id, val, timeStamp);
}

int uploadError(int id, long val)
{
	return parentNode.txError(id, val);
}

int handleParentReq( RxItem *rxItem)  // cmd, to, parm1, parm2
{
	#ifdef DEBUG
		parentNode.debug("Prnt<", rxItem);
	#endif

	if( rxItem->data.msg.node==2
	 || rxItem->data.msg.node==0
	 || rxItem->data.msg.node==parentNode.nodeId )
	{
		return localRequest(rxItem);
	}

	parentNode.debug("skip", rxItem);

//	if(parentNode.nodeId==0)
//	{
//		#ifdef DEBUG
//			parentNode.debug("skip", rxItem);
//		#endif
//
//		return 0;
//	}
//
//	#ifdef DEBUG
//		parentNode.debug("forward", rxItem);
//	#endif
//
//	#ifndef SKIP_CHILD_NODES
//	//	return childNodes.putBuf( req );
//	#endif
}


void setup()   //TODO
{
	wdt_reset();
	// First thing, turn it off
	wdt_disable();
	wdt_enable(WDTO_8S);

	Serial.begin(115200);

	eeParms2.onUpload(upload);
	eeParms2.setup();

	#ifdef SERIAL_CONSOLE
		console.nodeId = SERIAL_CONSOLE;
		console.isConsole = true;
		console.onReceive( localRequest );
		console.setup(0, 115200);
	#endif

	#ifdef DEBUG
		Serial.println(F("DEBUG ketel ..."));
	#else
		Serial.println(F("Strt ketel ..."));
	#endif

	//thermostaat.setDebounce_s(3);
	//	thermostaat.id = 52;
	#ifdef THERMOSTAAT_PIN
		thermostaat.onUpload(upload);
		thermostaat.setMinimalActive_s(3);
		thermostaat.setMinimalInactive_s(7);
		thermostaat.syncInterval = eeParms2.samplePeriode_sec * 1000L;
		thermostaat.available = 1;         // upload latest status
	#endif
	// temp default resolution = 9;
	// temp default delta check between samples = 20  to prevent errors
	// temp default samples every 7 seconds


	int nodeId = NODE_ID;

	parentNode.onError(uploadError);
	parentNode.onUpload(upload);
	parentNode.onReceive( handleParentReq );
	parentNode.nodeId = nodeId;
	parentNode.isParent = true;

	#ifdef SERIAL_PARENT
	//	delay(100);
	//	parentNode.setup(0,115200);
	#endif

	#ifdef TWI_PARENT
		parentNode.begin();
		//parentNode.txBufAutoCommit = true;
	#endif


	#ifdef NETW_PARENT_SPI
		bool isSPIMaster = false;
		parentNode.setup( SPI_PIN, isSPIMaster);
		parentNode.isParent = true;
	#endif

	parentNode.txUpload(0, eeParms2.bootCount);

	tempCV.loop(3);
	tempCV.onUpload(upload);
	tempCV.onError(uploadError);
	tempCV.setSyncInterval(eeParms2.samplePeriode_sec);

	#ifdef OTHER_TEMPS
		tempBoiler.deltaTempMax = 0;  // disable check for temp schommelingen
		tempBoiler.loop(3);
		tempBoiler.onUpload(upload);
		tempBoiler.onError(uploadError);
		tempBoiler.setSyncInterval(eeParms2.samplePeriode_sec);

		tempBuiten.loop(3);
		tempBuiten.onUpload(upload);
		tempBuiten.onError(uploadError);
		tempBuiten.setSyncInterval(eeParms2.samplePeriode_sec);

		tempRetourBad.loop(3);
		tempRetourBad.onUpload(upload);
		tempRetourBad.onError(uploadError);
		tempRetourBad.setSyncInterval(eeParms2.samplePeriode_sec);
		tempRetourRechts.loop(3);
		tempRetourRechts.onUpload(upload);
		tempRetourRechts.onError(uploadError);
		tempRetourRechts.setSyncInterval(eeParms2.samplePeriode_sec);
		tempRetourLinks.loop(3);
		tempRetourLinks.onUpload(upload);
		tempRetourLinks.onError(uploadError);
		tempRetourLinks.setSyncInterval(eeParms2.samplePeriode_sec);


	#endif

   // delay(100);
   // parentNode.pullUpsOff();

}

int localRequest(RxItem *rxItem)
{
	#ifdef DEBUG
		parentNode.debug("local", rxItem);
	#endif

	int ret=0;

	switch (  rxItem->data.msg.cmd)
	{
	case 't':trace( true ); break;
	//case 'x': parentNode.tw_restart(); break;

	case 's':
	case 'S':
		localSetVal(rxItem->data.msg.id, rxItem->data.msg.val);
		ret = upload(rxItem->data.msg.id);
		break;
	case 'r':
	case 'R':
		ret = upload(rxItem->data.msg.id);
		break;

	default:
		eeParms2.handleRequest(rxItem);
		util.handleRequest(rxItem);
		break;
	}

	return ret;
}


void loop()  //TODO
{
	wdt_reset();

	#if defined(SERIAL_CONSOLE) || defined(SPI_CONSOLE)
		console.loop();
	#endif

	tempCV.loop();


	#ifdef OTHER_TEMPS
		tempBoiler.loop();
		tempBuiten.loop();
		tempRetourBad.loop();
		tempRetourRechts.loop();
		tempRetourLinks.loop();
	#endif

//	case 24: tempRetourBad.upload(); break;
//	case 25: tempRetourRechts.upload(); break;
//	case 26: tempRetourLinks.upload(); break;


#ifdef THERMOSTAAT_PIN
	// when the thermostaat was switched on we set the timer.

	thermostaat.loop(0);

	if(thermostaatActive != thermostaat.ist() )
	{
		thermostaatActive = thermostaat.ist();            // remember state
		if(thermostaatActive)
		{
			thermostaatTimer =  millis() + eeParms2.thermostaatPeriode_min * 60000;
		}
	}

	//  deactivate when timer expired.
	if( thermostaatActive
	 && millis() > thermostaatTimer )  //
	{
		thermostaat.off();
	}

//    if(thermostaat.available>0 )
//    {
//    	localUpload(52);
//    	thermostaat.commit();
//    }

#endif

	eeParms2.loop();

	parentNode.loop();

	util.loop();


	if(  parentNode.isReady() //millis() > parentNode.netwTimer
	 && millis() > eeParms2.bootTimer
	 && eeParms2.bootMessages < 20
	 && parentNode.nodeId > 0)
	{
		int ret=0;
		eeParms2.bootTimer = millis() + TWI_SEND_INTERVAL + 10;
		switch( eeParms2.bootMessages )
		{
		case 1: ret = upload(1); break;
		case 2: ret = upload(50); break;
		case 3: ret = upload(81); break; 	// reads
		case 4: ret = upload(84); break;   // writes
		case 5: ret = upload(86); break;   // write retries

		#ifdef VOLTAGE_PIN
			case 3: ret =  upload(11); break;  	// Vin
		#endif

		}

		if(ret<0)
			{ eeParms2.bootTimer = millis() + TWI_SEND_ERROR_INTERVAL;}
		else
			{eeParms2.bootMessages++;}

	}

	#ifdef DEBUG
		trace(false);
	#endif

		//Serial.println(F("Done trace"));
} // end loop

void trace(bool force)
{
	if( ! force && traceTimer_t > millis() ) return;
	traceTimer_t = millis() + TRACE_MSEC;

	Serial.print("@ ");
	Serial.print(millis() / 1000);
	Serial.print(": ");
	Serial.print(F("ramFree="));Serial.print(System::ramFree()  );

	#ifdef THERMOSTAAT_PIN
		Serial.print(F("Term onVal="));Serial.print(thermostaat._switchOnValue);
		Serial.print(F(", offVal="));Serial.print(thermostaat._switchOffValue);
	#endif

	Serial.println();

//	Serial.print(F("aanUitThermostaat isActive()="));
//	Serial.print(thermostaat.isActive());
//	Serial.print(F(", ist()="));
//	Serial.print(thermostaat.ist());
//	Serial.print(F(", soll()="));
//	Serial.println(thermostaat.soll());

	tempCV.trace("cv");

	#ifdef OTHER_TEMPS
		tempBoiler.trace("boil");
		tempBuiten.trace("buit");
		tempRetourBad.trace("bad");
		tempRetourRechts.trace("rechts");
		tempRetourLinks.trace("links");
	#endif

		//	case 24: tempRetourBad.upload(); break;
		//	case 25: tempRetourRechts.upload(); break;
		//	case 26: tempRetourLinks.upload(); break;

    parentNode.trace("parentNode");
   // console.trace("console");
	Serial.flush();

//	if(tparmRed){Serial.println("tparmRed");tparmRed = false;}
//	if(tparmSet){Serial.println("tparmSet");tparmSet = false;}
//	if(testVal > 0){Serial.print("testVal");Serial.println(testVal);testVal = 0;}
}

//#include <Arduino.h>
//#include <inttypes.h>

//#define Aref 2860.0    // in mV
//#define Aref 5000.0    // in mV

// WHEN ARef is connected THEN set analogReference to external before analogRead to prevent DAMAGE
//analogReference(INTERNAL );  // INTERNAL EXTERNAL  Mega only INTERNAL1V1    INTERNAL2V56

