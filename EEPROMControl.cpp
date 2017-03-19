#include "EEPROMControl.h"
#define DEBUG

EEPROMControl* EEPROMControl::eepromInstance = NULL;

EEPROMControl* EEPROMControl::getInstance(){
	if(!eepromInstance)
		eepromInstance = new EEPROMControl;
	return eepromInstance;
}

EEPROMControl::EEPROMControl(){
//  EEPROM.begin(512);
}

void EEPROMControl::clear(){
  for(int i = 0; i < 512 ; i++){
    EEPROM.write(i,0);
  }
}

void EEPROMControl::save(saveType type,char* dataWrite,int size){
	if(size < LENGTH_ADDR){
		switch(type){
			case SSID:
					for(int i = ADDR_SSID; i < size ; i++)
						EEPROM.write(i,(int)dataWrite[i]);
					EEPROM.write(size,'\n');
          			EEPROM.commit();
				break;

			case PASSWORD:
					for(int i = 0; i < size ; i++)
						EEPROM.write((i+ADDR_PASSWORD),(int)dataWrite[i]);
					EEPROM.write((size + ADDR_PASSWORD),'\n');
          			EEPROM.commit();
				break;

			case CODE:
					for(int i = 0; i < size ; i++)
						EEPROM.write((i+ADDR_CODE),(int)dataWrite[i]);
					EEPROM.write((size+ADDR_CODE),'\n');
					EEPROM.commit();
				break;
		}
	}
}

String EEPROMControl::read(saveType type){
  String dataRead = "";
	switch(type){
		case SSID:
		        #ifdef DEBUG
		          Serial.print(F("SSID first is "));
		          Serial.println(EEPROM.read(ADDR_SSID));
		        #endif
		      
		        if(EEPROM.read(ADDR_SSID) == 0){
		          return String("");
		        }
				for(int i = ADDR_SSID; EEPROM.read(i)!='\n' && i < (ADDR_SSID+LENGTH_ADDR) ; i++)
					dataRead += (char)EEPROM.read(i);
			break;

		case PASSWORD:
		        #ifdef DEBUG
		          Serial.print(F("PASSWORD first is "));
		          Serial.println(EEPROM.read(ADDR_PASSWORD));
		        #endif
		        
		        if(EEPROM.read(ADDR_PASSWORD) == 0){
		          return String("");
		        }
				for(int i = ADDR_PASSWORD; EEPROM.read(i)!='\n' && i < (ADDR_PASSWORD+LENGTH_ADDR) ; i++){
					dataRead += (char)EEPROM.read(i);
				}
			break;

		case CODE:
				#ifdef DEBUG
				  Serial.print(F("CODE first is "));
		          Serial.println(EEPROM.read(ADDR_CODE));
				#endif

		          if(EEPROM.read(ADDR_CODE) == 0){
		          	return String("");
		          }
		          for(int i = ADDR_CODE; EEPROM.read(i)!='\n' && i < (ADDR_CODE+LENGTH_ADDR); i++){
		          	dataRead += (char)EEPROM.read(i);
		          }
		    break;
	}
  return dataRead;
}

