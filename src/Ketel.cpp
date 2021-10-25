#include "Ketel.h"

class EEParms2: public EEUtil    // TODO
{
public:
	volatile int thermostaatPeriode_min = 5;
	long chk1  = 0x00010204;

	EEParms2() { }
	virtual ~EEParms2(){ }

    void setup( ) //
    {
		if( readLong(offsetof(EEParms2, chk1)) == chk1  )
    	{
    		readAll();
    		setThermostaatPeriode_min( readInt(offsetof(EEParms2, thermostaatPeriode_min)));
    		changed = false;
    		resync();
			#ifdef DEBUG
				Serial.println(F("EEUtil.readAll"));
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
			EEUtil::writeAll();
			changed = false;
			resync();
		}
	}

	void setThermostaatPeriode_min( long newVal)
	{
		thermostaatPeriode_min = (int) newVal;
		if(thermostaatPeriode_min < 1) thermostaatPeriode_min = 1;
		if(thermostaatPeriode_min > 30) thermostaatPeriode_min = 30;
		changed = true;
	}

	void resync()
	{
		#ifdef DEBUG
			Serial.println("EEParms2.resync");
		#endif

		verwarming.uploadInterval_s = samplePeriode_sec;
		tempCV.setSyncInterval(samplePeriode_sec);

		#ifndef DEBUG		
			tempRetourBad.setSyncInterval(samplePeriode_sec);
			tempRetourRechts.setSyncInterval(samplePeriode_sec);
			tempRetourLinks.setSyncInterval(samplePeriode_sec);
		#endif
	}
};
EEParms2 eeParms;

#ifdef SPI_CONSOLE
	ISR (SPI_STC_vect){	console.handleInterupt();}
#endif
#ifdef	TWI_PARENT
	ISR(TWI_vect){ parentNode.tw_int(); }
#endif
#ifdef SPI_PARENT
	ISR (SPI_STC_vect){parentNode.handleInterupt();}
#endif

void localSetVal(int id, long val)
{
	switch(id )
	{
	case 52: 
		#ifndef DEBUG
			isVerwarmingAktief = val > 0;
			power = isVerwarmingAktief? 100 : 0;
		#endif
		break;

	case VERWARMEN_ID-1: 	// controle heating

		power = val;
		break;

	case 53: 
		eeParms.setThermostaatPeriode_min(val); 
		verwarming.setDebounce_s(eeParms.thermostaatPeriode_min * 60) ;		 //				
		break;

	default:
		eeParms.setVal( id,  val);
		parentNode.setVal( id,  val);
		break;
	}

	if(id==2)
	{
		tempCV.setSyncInterval(eeParms.samplePeriode_sec);
		#ifndef DEBUG
			tempRetourBad.setSyncInterval(eeParms.samplePeriode_sec);
			tempRetourRechts.setSyncInterval(eeParms.samplePeriode_sec);
			tempRetourLinks.setSyncInterval(eeParms.samplePeriode_sec);
		#endif
		verwarming.uploadInterval_s = eeParms.samplePeriode_sec  ;  // TODO check if  working ok
	}
}

void nextUpload(int id){
	
	switch( id ){
 		case VERWARMING_ACTIVE_ID: myTimers.nextTimer(TIMER_UPLOAD_VERWARMING_ACTIVE);		break;
		case VERWARMEN_ID-1: myTimers.nextTimer(TIMER_UPLOAD_POWER);		break;
		case 50: myTimers.nextTimer(TIMER_UPLOAD_LED);		break;
		case BOILER_ID: myTimers.nextTimer(TIMER_UPLOAD_BOILER);		break;
	}
}



int upload(int id)
{
	int ret = 0;

	switch( id )
	{

	case 2:
		upload(id, NODE_TYPE );   
		break;	
	case 8:
		upload(id, JL_VERSION );   
		break;

	case BOILER_ID:
		upload(id, ! digitalRead(BOILER_PIN) );   
		break;	
 
	case TEMP_CV: tempCV.upload();  		
		break;

	#ifdef VOLTAGE_PIN
		case 11: upload(id, vin.val);    		break;
	#endif

	case VERWARMING_ACTIVE_ID:  upload(id, isVerwarmingAktief)  ;	break;

	case VERWARMEN_ID-1: upload(id, power);				break;
	case VERWARMEN_ID:   verwarming.uploadIsRunning(); 	break;
	case VERWARMEN_ID+1: verwarming.uploadState();		break;
	case VERWARMEN_ID+2: verwarming.uploadManual();    	break;	


	// case VERWARMEN_ID: verwarming.upload() ;   	break;
	case 53: upload(id, eeParms.thermostaatPeriode_min);  	break;


	#ifndef DEBUG
		case TEMP_RETOUR_BAD: tempRetourBad.upload(); break;
		case TEMP_RETOUR_RECHTS: tempRetourRechts.upload(); break;
		case TEMP_RETOUR_LINKS: tempRetourLinks.upload(); break;
		case TEMP_WW: tempWW.upload(); break;
	#endif

	default:
		if( 1==2
		 ||	parentNode.upload(id)>0
		 ||	eeParms.upload(id)>0
		){}
		break;
	}
	return ret;
}

int upload(int id, long val) { return upload(id, val, millis()); }
int upload(int id, long val, unsigned long timeStamp)
{
	nextUpload(id);
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

	return 0;
}

int localRequest(RxItem *rxItem)
{
	#ifdef DEBUG
		parentNode.debug("local", rxItem);
	#endif

	int ret=0;
	int subCmd;
	int val;

	switch (  rxItem->data.msg.cmd)
	{
	case 'L':
		switch (rxItem->data.msg.id)
		{
		case 1:
			trace();
			break;
		}
		break;

	case 'l':
	    subCmd = rxItem->data.msg.val & 0x000000ff ;
    	val = rxItem->data.msg.val >> 8;
		switch (rxItem->data.msg.id)
		{
		case 1:
			parentNode.localCmd(subCmd, val);
			break;
 		case 2:
			parentNode.localCmd(subCmd+100, val);
			break;
		}
		break; 	

	case 's':
		localSetVal(rxItem->data.msg.id, rxItem->data.msg.val);
		break;

	case 'S':
		localSetVal(rxItem->data.msg.id, rxItem->data.msg.val);
		ret = upload(rxItem->data.msg.id);
		break;
	case 'r':
	case 'R':
		ret = upload(rxItem->data.msg.id);
		break;
	case 'B':
		wdt_enable(WDTO_15MS);
		while(true){
			delay(500);
			asm volatile ("  jmp 0");
		}
	default:
		eeParms.handleRequest(rxItem);
		break;
	}

	return ret;
}



void setup()   //TODO
{
	wdt_reset();
	wdt_disable();
	wdt_enable(WDTO_8S);

	pinMode(LED_BUILTIN, OUTPUT);
	// pinMode(BOILER_PIN, INPUT);
	

	Serial.begin(115200);

	eeParms.onUpload(upload);
	eeParms.setup();

	#ifdef DEBUG
		Serial.println(F("DEBUG ketel ..."));
	#else
		Serial.println(F("Strt ketel ..."));
	#endif


	int nodeId = NODE_ID;

	parentNode.onReceive( handleParentReq);
	parentNode.onError(uploadError);
	parentNode.onUpload(upload);
	parentNode.nodeId = nodeId;
	parentNode.isParent = true;

	#ifdef TWI_PARENT
		parentNode.begin();
		//parentNode.txBufAutoCommit = true;
	#endif


	#ifdef NETW_PARENT_SPI
		bool isSPIMaster = false;
		parentNode.setup( SPI_PIN, isSPIMaster);
		parentNode.isParent = true;
	#endif

	tempCV.onUpload(upload,TEMP_CV);
	tempCV.onError(uploadError);

	#ifndef DEBUG
		tempRetourBad.onUpload(upload, TEMP_RETOUR_BAD);
		tempRetourBad.onError(uploadError);

		tempRetourRechts.onUpload(upload, TEMP_RETOUR_RECHTS);
		tempRetourRechts.onError(uploadError);

		tempRetourLinks.onUpload(upload, TEMP_RETOUR_LINKS);
		tempRetourLinks.onError(uploadError);

		tempWW.onUpload(upload, TEMP_WW);
		tempWW.onError(uploadError);
	#endif

	verwarming.onUpload(upload, VERWARMEN_ID);
	verwarming.onCheck(evaluateVerwarming);
	verwarming.setDebounce_s(eeParms.thermostaatPeriode_min * 60 ) ; //
	// verwarming.sayHello = true;
	verwarming.dutyCycleMode = true;
	verwarming.numberOfStates = NUMBER_OF_DUTY_LEVELS;
	#ifdef DEBUG
		verwarming.test(true);  // show buildin led
	#endif

	myTimers.nextTimer(TIMER_TRACE, 3);
	myTimers.nextTimer(TIMER_UPLOAD_ON_BOOT, 0);
	myTimers.nextTimer(TIMER_UPLOAD_VERWARMING_ACTIVE, 0);
	myTimers.nextTimer(TIMER_UPLOAD_POWER, 0);

	#ifdef DEBUG
		isVerwarmingAktief = true;
		power = 50;
	#endif


	wdt_reset();
}


int evaluateVerwarming(bool on, bool isRunning){

	// return 0; // 0=off, 1=on, -1=nop, >1 = %
 
	// don't stress when duty is allready set
	// local power is controlling the 
	if( power > 0 ){ 

		return power>0 ? power: 1;	

	} else {

		return on ? 0 : -1;
	}

	return -1;
}

 
void loop()  //TODO
{
	wdt_reset();

	tempCV.loop();

	#ifndef DEBUG
		tempRetourBad.loop();
		tempRetourRechts.loop();
		tempRetourLinks.loop();
		tempWW.loop();
	#endif

	// compensate heating time

	// if( ! digitalRead( BOILER_PIN )){

	// 	if( myTimers.isTime(TIMER_PAUSE)
	// 	 &&	verwarming.isRunning()	
	// 	){
	// 		verwarming.pause(1);
	// 		myTimers.nextTimer(TIMER_PAUSE, 60);
	// 	}
	
	// } else {

	// 	myTimers.timerOff(TIMER_PAUSE)
	// } 
	 
	verwarming.loop();

	eeParms.loop();

	parentNode.loop();

	if( parentNode.isReady() 
	 && parentNode.isTxEmpty()
	){
		if( myTimers.isTime(TIMER_UPLOAD_ON_BOOT)
		){
			myTimers.nextTimerMillis(TIMER_UPLOAD_ON_BOOT, TWI_SEND_ERROR_INTERVAL);
			switch( uploadOnBootCount )
			{
				case 1:
					if(millis()<60000) upload(1);
					break;    // boottimerr


				case 2: upload(50); break;
				case 3: upload(3); break; 	// reads

				case 4: upload(VERWARMING_ACTIVE_ID); break;  // aan?
				case 5: upload(53); break;  // aan?

				case 6: upload(VERWARMEN_ID); break;  // aan?
				case 7: upload(VERWARMEN_ID+1); break;  // aan?
				case 8: upload(VERWARMEN_ID+2); break;  // aan?

				case 9: upload(2); break;
				case 10: upload(8); break;	// version 
				case 11: upload(BOILER_ID); break;	// version 

				case 30: myTimers.timerOff(TIMER_UPLOAD_ON_BOOT); break;			
			}

			uploadOnBootCount++;			
		}


		if( myTimers.isTime(TIMER_UPLOAD_LED)){
			myTimers.nextTimer(TIMER_UPLOAD_LED);
			upload(50);
		}
		if( myTimers.isTime(TIMER_UPLOAD_VERWARMING_ACTIVE)){
			myTimers.nextTimer(TIMER_UPLOAD_VERWARMING_ACTIVE);
			upload(VERWARMING_ACTIVE_ID);

		}
		if( myTimers.isTime(TIMER_UPLOAD_POWER)){
			myTimers.nextTimer(TIMER_UPLOAD_POWER);
			upload(VERWARMEN_ID-1);
		}	
		if( myTimers.isTime(TIMER_UPLOAD_BOILER)){
			myTimers.nextTimer(TIMER_UPLOAD_BOILER);
			upload(BOILER_ID);
		}	

			
	} 

	#ifdef DEBUG		
		if( myTimers.isTime(TIMER_TRACE)){ trace();}
	#endif

	
			 
} // end loop


void trace( )
{
	myTimers.nextTimer(TIMER_TRACE, TRACE_SEC);

	Serial.print(F("@"));
	Serial.print(millis() / 1000);
	Serial.print(F(" - "));
	// Serial.print(F("ramFree="));Serial.print(System::ramFree()  );

 	Serial.print(F("debounce:"));Serial.print(eeParms.thermostaatPeriode_min);
	Serial.print(F(", verwAktief:"));Serial.print(isVerwarmingAktief);
	Serial.print(F(", power:"));Serial.print(power);
	Serial.print(F(", manTimLeft:"));Serial.print(verwarming.manualTimeLeft());
	Serial.print(F(", boiler:"));Serial.print(!digitalRead(BOILER_PIN));
	
	Serial.println();

	tempCV.trace("cv");
	verwarming.trace("verw");

    // parentNode.trace("parentNode");
	Serial.flush();
}

//#include <Arduino.h>
//#include <inttypes.h>

//#define Aref 2860.0    // in mV
//#define Aref 5000.0    // in mV

// WHEN ARef is connected THEN set analogReference to external before analogRead to prevent DAMAGE
//analogReference(INTERNAL );  // INTERNAL EXTERNAL  Mega only INTERNAL1V1    INTERNAL2V56
