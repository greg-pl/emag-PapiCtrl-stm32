/*
 * Cfg.h
 *
 *  Created on: Jul 21, 2021
 *      Author: Grzegorz
 */

#ifndef CFG_H_
#define CFG_H_

#include "stdint.h"
#include "IOStream.h"
#include "ErrorDef.h"
#include <Cpx.h>

enum {
	spd4800 = 0, //
	spd9600, //
	spd19200, //
	spd57600, //
	spd115200, //
};

typedef enum {
	encREMOTE=1, // 1
	encAUTO, // 2
	encOFF, //
	encLEVEL1, // 3
	encLEVEL2, // 4
	encLEVEL3, // 5
	encLEVEL4, // 6
	encLEVEL5, // 7
	encLEVEL6, // 8

} EncoderPos;

typedef enum {
	styleUnknown = 0,  //
	styleA = 1, // czytanie danych z manipulatora i radia
	styleB = 2,  //  czytanie danych z zasilacza,
} PapiStyle;

typedef struct {
	uint8_t mdbDebug;  // poziom logów dla Modbus
	uint8_t MdbSpeed;
	uint8_t xbeeDebug;
	uint8_t S847_MdbNr; // numer na modbus karty S847
	struct {
		//dipSwitch
		bool IR_Visual; //1-Visul_Leds, 0 - IR_Leds
		bool photoActive;
		bool nightLevel;
		bool tiltBypass;
		bool heaterEnable;
		uint8_t encoder;
	} altPulp;
	float lighNightLimit;
	float lighNightLimitHist;
	float nightLevelWhite1;
	float nightLevelWhite2;
	float nightLevelRed1;
	float nightLevelRed2;
	float nightLevelIR1;
	float nightLevelIR2;
	uint8_t xbeeChannel;

} UserCfg;

#define CFG_REC_MAX_SIZE 0x100

typedef union {
	uint8_t tab_b[CFG_REC_MAX_SIZE];
	uint32_t tab_32[CFG_REC_MAX_SIZE / 4];
	struct {
		uint8_t tab_bm4[CFG_REC_MAX_SIZE - 4];
		uint16_t free;
		uint16_t Crc;
	};
	struct {
		uint32_t Sign;
		uint16_t Size;
		uint16_t Ver;
		UserCfg R;
	};

} CfgRec;

class Cfg {
private:
	enum {
		TM_MIG_MAX = 10000, // maksymalny czas podczas migania
		TM_TEST_MAX = 500, // maksymalny czas testu
		TM_TEST_MIN = 5, // 5[ms] - minimalny czas testu
	};

	TStatus WriteToMyFlash(const uint8_t *src, int size);
	bool CheckFloatItem(float *pf, const CpxFloatDefItem *def, float defau);
	bool CheckIntItem(int *pf, const CpxIntDefItem *def, int defau);
	bool CheckByteItem(uint8_t *pf, const CpxIntDefItem *def, uint8_t defau);

	bool CheckCfg();
public:
	CfgRec dt;
	Cfg();
	TStatus Default();
	void shell(OutStream *strm, const char *cmd);
	TStatus save();
	TStatus init();
	int GetMdbSpeed();
	uint8_t getAltDipSwitch();
};

#endif /* CFG_H_ */
