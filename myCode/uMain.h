/*
 * uMain.h
 *
 *  Created on: Feb 10, 2022
 *      Author: Grzegorz
 */

#ifndef UMAIN_H_
#define UMAIN_H_

#include "myDef.h"
#include "IOStream.h"

extern "C" OutStream* getOutStream();

class GlobalVar {
public:
	struct{
		float tidsTemp;
		float vemlLight;
		bool isNight;
	}local;
	bool kyTiltSet;  // stan klawisza
	bool kyEmergency; // stan klawisza
	bool manipChg; // nowe dane odczytane z manipulatora
	bool sendLightState; // wyślij dane o jasności
	bool manipOk; // czy dane z manipulatora poprawne
	struct {
		bool remote;
		bool autom;
		uint8_t level;
	} encoder;
	struct {
		uint16_t val;
		bool IR_Visual; //1-Visul_Leds, 0 - IR_Leds
		bool photoActive;
		bool nightLevel;
		bool tiltBypass;
		bool heaterEnable;
	} dpSw;
	bool newProjVal; // nowe dane odczytane z projektora
	bool projektOk;
	struct {
		uint16_t devStatus;  // definicja bitów w dokumentacji Modbusa
		uint16_t devStatus2; // definicja bitów w dokumentacji Modbusa
		float tempSens1;
		float tempSens2;
		float tempSht35;
		float humiditiSht35;
		float angX;
		float angY;
	} projector;

	struct{
		uint16_t val;
		bool chg;
	}remoteDt;

	void setDipSw(uint8_t v);
	void setEncoder(uint8_t v);
	void setNoPulpit();
	uint16_t getLevelCpx();
	bool getGlobError();
	void checkDay_Night();
	void getAutoTab(uint16_t *tab);
	void setRemoteVal(uint16_t val);

};

extern GlobalVar globalVar;
extern "C" void reboot(int tm);
extern "C" void wdgPulse();


#endif /* UMAIN_H_ */
