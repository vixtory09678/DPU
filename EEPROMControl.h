#ifndef EEPROM_CONTROL_H
#define EEPROM_CONTROL_H

#include "Arduino.h"
#include "EEPROM.h"

#define ADDR_SSID 0
#define ADDR_PASSWORD 25
#define ADDR_CODE 50
#define LENGTH_ADDR 25

enum {  // waiting more implements
	SSID,
	PASSWORD,
	CODE
} typedef saveType;

class EEPROMControl
{
	public:
    static EEPROMControl* getInstance();
		void save(saveType type,char* dataWrite,int size);
		String read(saveType type);
		static EEPROMControl* eepromInstance;
    void clear();
   private:
    EEPROMControl();
};


#endif
