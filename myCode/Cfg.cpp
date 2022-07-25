/*
 * Cfg.cpp
 *
 *  Created on: Jul 21, 2021
 *      Author: Grzegorz
 */
#include "string.h"
#include "stdio.h"

#include <main.h>
#include <Cfg.h>
#include <ShellItem.h>
#include <Token.h>
#include <utils.h>

#define CFG_SEC   __attribute__ ((section (".cfgdata")))

static CFG_SEC CfgRec RomCfg;
#define CFG_SIGN  0x37824A0B

const char* FormatOnePlace(int idx) {
	return "%.1f";
}
const CpxFloatDefItem PwmDef = { min:0, max:100, getFrm : FormatOnePlace };
const CpxIntDefItem FreqDef = { min:50, max:150 };
const CpxIntDefItem MigDef = { min:0, max:10000 };
const CpxIntDefItem TestTimeDef = { min:5, max:500 };
const CpxIntDefItem DebugLevelDef = { min:0, max:5 };
const CpxIntDefItem S847_MdbNrDef = { min:10, max:18 };
const CpxFloatDefItem LuxLimitDef = { min:0, max:10000, getFrm : FormatOnePlace };
const CpxFloatDefItem LuxHisterDef = { min:2, max:100, getFrm : FormatOnePlace };

const CpxIntDefItem XBeeChannelDef = { min:1, max:5 };




const CpxSelectItem UartSpeedTab[] = { //
		{ val:-1, cap:"???" }, //  wartość nieoznaczona
				{ val:0, cap:"4800" }, //
				{ val:1, cap:"9600" }, //
				{ val:2, cap:"19200" }, //
				{ val:3, cap:"57600" }, //
				{ val:4, cap:"115200" }, //
				{ val:0, NULL } //
		};

const CpxSelectItem PapiStyleTab[] = { //
		{ val:0, cap:"Style_???" }, //  wartość nieoznaczona
				{ val:1, cap:"StyleA" }, //
				{ val:2, cap:"StyleB" }, //
				{ val:0, NULL } //
		};

const CpxSelectItem EncoderPosTab[] = { //
		{ val:0, cap:"???" }, //  wartość nieoznaczona
				{ val:1, cap:"REMOTE" }, //
				{ val:2, cap:"AUTO" }, //
				{ val:3, cap:"OFF" }, //
				{ val:4, cap:"LEVEL1" }, //
				{ val:5, cap:"LEVEL2" }, //
				{ val:6, cap:"LEVEL3" }, //
				{ val:7, cap:"LEVEL4" }, //
				{ val:8, cap:"LEVEL5" }, //
				{ val:9, cap:"LEVEL6" }, //
				{ val:0, NULL } //
		};

const CpxDescr ConfigDscr[] = { //
		{ ctype : cpxSELECT_BYTE, id:1, ofs: offsetof(CfgRec, R.MdbSpeed), Name : "MdbSpeed", size:sizeof(CfgRec::R.MdbSpeed), exPtr:UartSpeedTab }, //
				{ ctype : cpxBYTE, id:2, ofs: offsetof(CfgRec, R.mdbDebug), Name : "MdbDebug", size:sizeof(CfgRec::R.mdbDebug), exPtr:&DebugLevelDef }, //
				{ ctype : cpxBYTE, id:4, ofs: offsetof(CfgRec, R.S847_MdbNr), Name : "S847_MdbNr", size:sizeof(CfgRec::R.S847_MdbNr), exPtr:&S847_MdbNrDef }, //
				//dipSwitch
				{ ctype : cpxBOOL, id:5, ofs: offsetof(CfgRec, R.altPulp.IR_Visual), Name : "alt_IR_Visual", size:sizeof(CfgRec::R.altPulp.IR_Visual), exPtr:NULL }, //
				{ ctype : cpxBOOL, id:6, ofs: offsetof(CfgRec, R.altPulp.photoActive), Name : "alt_photoActive", size:sizeof(CfgRec::R.altPulp.photoActive), exPtr:NULL }, //
				{ ctype : cpxBOOL, id:7, ofs: offsetof(CfgRec, R.altPulp.nightLevel), Name : "alt_nightLevel", size:sizeof(CfgRec::R.altPulp.nightLevel), exPtr:NULL }, //
				{ ctype : cpxBOOL, id:8, ofs: offsetof(CfgRec, R.altPulp.tiltBypass), Name : "alt_tiltBypass", size:sizeof(CfgRec::R.altPulp.tiltBypass), exPtr:NULL }, //
				{ ctype : cpxBOOL, id:9, ofs: offsetof(CfgRec, R.altPulp.heaterEnable), Name : "alt_heaterEnable", size:sizeof(CfgRec::R.altPulp.heaterEnable), exPtr:NULL }, //
				//encoder
				{ ctype : cpxSELECT_BYTE, id:11, ofs: offsetof(CfgRec, R.altPulp.encoder), Name : "alt_encoder", size:sizeof(CfgRec::R.altPulp.encoder), exPtr:EncoderPosTab }, //

				{ ctype : cpxFLOAT, id:12, ofs: offsetof(CfgRec, R.lighNightLimit), Name : "lighNightLimit", size:sizeof(CfgRec::R.lighNightLimit), exPtr:&LuxLimitDef }, //
				{ ctype : cpxFLOAT, id:13, ofs: offsetof(CfgRec, R.lighNightLimitHist), Name : "lighNightLimitHist", size:sizeof(CfgRec::R.lighNightLimitHist), exPtr:&LuxHisterDef }, //
				{ ctype : cpxFLOAT, id:14, ofs: offsetof(CfgRec, R.nightLevelWhite1), Name : "nightLevelWhite1", size:sizeof(CfgRec::R.nightLevelWhite1), exPtr:&PwmDef }, //
				{ ctype : cpxFLOAT, id:15, ofs: offsetof(CfgRec, R.nightLevelWhite2), Name : "nightLevelWhite2", size:sizeof(CfgRec::R.nightLevelWhite2), exPtr:&PwmDef }, //
				{ ctype : cpxFLOAT, id:16, ofs: offsetof(CfgRec, R.nightLevelRed1), Name : "nightLevelRed1", size:sizeof(CfgRec::R.nightLevelRed1), exPtr:&PwmDef }, //
				{ ctype : cpxFLOAT, id:17, ofs: offsetof(CfgRec, R.nightLevelRed2), Name : "nightLevelRed2", size:sizeof(CfgRec::R.nightLevelRed2), exPtr:&PwmDef }, //
				{ ctype : cpxFLOAT, id:18, ofs: offsetof(CfgRec, R.nightLevelIR1), Name : "nightLevelIR1", size:sizeof(CfgRec::R.nightLevelIR1), exPtr:&PwmDef }, //
				{ ctype : cpxFLOAT, id:19, ofs: offsetof(CfgRec, R.nightLevelIR2), Name : "nightLevelIR2", size:sizeof(CfgRec::R.nightLevelIR2), exPtr:&PwmDef }, //

				{ ctype : cpxBYTE, id:3, ofs: offsetof(CfgRec, R.xbeeDebug), Name : "XBeeDebug", size:sizeof(CfgRec::R.xbeeDebug), exPtr:&DebugLevelDef }, //
				{ ctype : cpxBYTE, id:3, ofs: offsetof(CfgRec, R.xbeeChannel), Name : "XBeeChannel", size:sizeof(CfgRec::R.xbeeChannel), exPtr:&XBeeChannelDef }, //


				{ ctype : cpxNULL } };

//const void *exPtr;

Cfg::Cfg() {
	// TODO Auto-generated constructor stub

}

TStatus Cfg::init() {
	dt = RomCfg;
	TStatus st = stCfgDataErr;
	if (TCrc::Check((uint8_t*) &dt, sizeof(dt))) {
		if (dt.Sign == CFG_SIGN) {
			if (dt.Size == sizeof(dt)) {
				st = stOK;
			}

		}
	}
	if (st != stOK) {
		Default();
	}
	if (CheckCfg()) {
		save();
	}

	return st;
}

bool Cfg::CheckFloatItem(float *pf, const CpxFloatDefItem *def, float defau) {
	if (*pf < def->min || *pf > def->max) {
		*pf = defau;
		return true;
	}
	return false;
}

bool Cfg::CheckIntItem(int *pf, const CpxIntDefItem *def, int defau) {
	if (*pf < def->min || *pf > def->max) {
		*pf = defau;
		return true;
	}
	return false;
}

bool Cfg::CheckByteItem(uint8_t *pf, const CpxIntDefItem *def, uint8_t defau) {
	if (*pf < def->min || *pf > def->max) {
		*pf = defau;
		return true;
	}
	return false;
}

bool Cfg::CheckCfg() {
	bool ret = false;

	ret |= CheckByteItem(&dt.R.S847_MdbNr, &S847_MdbNrDef, S847_MdbNrDef.min);
	ret |= CheckFloatItem(&dt.R.lighNightLimit, &LuxLimitDef, 80);
	ret |= CheckFloatItem(&dt.R.lighNightLimitHist, &LuxHisterDef, 10);
	ret |= CheckFloatItem(&dt.R.nightLevelWhite1, &PwmDef, 60);
	ret |= CheckFloatItem(&dt.R.nightLevelWhite2, &PwmDef, 99);
	ret |= CheckFloatItem(&dt.R.nightLevelRed1, &PwmDef, 60);
	ret |= CheckFloatItem(&dt.R.nightLevelRed2, &PwmDef, 99);
	ret |= CheckFloatItem(&dt.R.nightLevelIR1, &PwmDef, 60);
	ret |= CheckFloatItem(&dt.R.nightLevelIR2, &PwmDef, 99);
	ret |= CheckByteItem(&dt.R.xbeeChannel, &XBeeChannelDef, 1);


	return ret;
}

TStatus Cfg::save() {
	dt.Sign = CFG_SIGN;
	dt.Size = sizeof(dt);
	dt.Ver = 1;
	TCrc::Set((uint8_t*) &dt, sizeof(dt) - 2);
	return WriteToMyFlash((const uint8_t*) &dt, sizeof(dt));
}

TStatus Cfg::WriteToMyFlash(const uint8_t *src, int size) {
	dword *srcPtr = (dword*) (int) src;
	HAL_StatusTypeDef st1;

	st1 = HAL_FLASH_Unlock();
	if (st1 == HAL_OK) {
		FLASH_EraseInitTypeDef eraseDt;
		eraseDt.TypeErase = FLASH_TYPEERASE_PAGES;
		eraseDt.PageAddress = (uint32_t) &RomCfg;
		eraseDt.NbPages = 1;
		eraseDt.Banks = FLASH_BANK_1;
		uint32_t PageError;
		st1 = HAL_FLASHEx_Erase(&eraseDt, &PageError);
		if (st1 == HAL_OK) {
			uint32_t prgAdr = (uint32_t) &RomCfg;
			while (size > 0) {
				dword dt = *srcPtr++;
				st1 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, prgAdr, dt);
				prgAdr += 4;
				size -= 4;
			}
		}
	}
	HAL_FLASH_Lock();

	if (memcmp(src, &RomCfg, sizeof(RomCfg)) != 0) {
		return stFlashWriteError;
	} else
		return stOK;
}

TStatus Cfg::Default() {
	memset(&dt, 0, sizeof(dt));
	dt.R.MdbSpeed = spd9600;
	dt.R.S847_MdbNr = 10;

	dt.R.lighNightLimit = 80;
	dt.R.lighNightLimitHist = 10;
	dt.R.nightLevelWhite1 = 60;
	dt.R.nightLevelWhite2 = 99;
	dt.R.nightLevelRed1 = 60;
	dt.R.nightLevelRed2 = 99;
	dt.R.nightLevelIR1 = 60;
	dt.R.nightLevelIR2 = 99;
	dt.R.xbeeChannel = 1;
	dt.R.altPulp.encoder = encAUTO;

	return save();
}

int Cfg::GetMdbSpeed() {
	switch (dt.R.MdbSpeed) {
	case spd4800:
		return 4800;
	default:
	case spd9600:
		return 9600;
	case spd19200:
		return 19200;
	case spd57600:
		return 57600;
	case spd115200:
		return 115200;
	}
}

uint8_t Cfg::getAltDipSwitch() {
	uint8_t v = 0;
	if (dt.R.altPulp.IR_Visual)
		v |= 0x01;
	if (dt.R.altPulp.photoActive)
		v |= 0x02;
	if (dt.R.altPulp.nightLevel)
		v |= 0x04;
	if (dt.R.altPulp.tiltBypass)
		v |= 0x08;
	if (dt.R.altPulp.heaterEnable)
		v |= 0x010;
	return v;
}

const ShellItem menuCfg[] = { //
		{ "list", "pokaż ustawienia, parametr [ |rtc|flash]" }, //
				{ "set", "ustaw wartość" }, //
				{ "info", "informacje o nastawie" }, //
				{ "default", "wartości domyślne" }, //
				{ "save", "save to Flash" }, //
				{ "init", "reload cfg from Flash" }, //
				{ NULL, NULL } };

void Cfg::shell(OutStream *strm, const char *cmd) {
	char tok[20];
	int idx = -1;
	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(menuCfg, tok);
	switch (idx) {
	case 0:  //list
	{
		Cpx cpx;

		cpx.init(ConfigDscr, &dt);
		cpx.list(strm);
	}
		break;
	case 1: //set
	{
		if (Token::get(&cmd, tok, sizeof(tok))) {
			char valB[60];
			valB[0] = 0;
			Token::get(&cmd, valB, sizeof(valB));
			Cpx cpx;
			cpx.init(ConfigDscr, &dt);
			if (cpx.set(tok, valB)) {
				strm->oMsgX(colGREEN, "[%s]=(%s) OK", tok, valB);
			} else {
				strm->oMsgX(colRED, "[%s]=(%s) Error", tok, valB);
			}
		}
	}
		break;
	case 2: //info
	{
		Cpx cpx;

		cpx.init(ConfigDscr, &dt);
		cpx.info(strm);
	}

		break;
	case 3: //default
		Default();
		strm->oMsgX(colWHITE, "Defaults loaded ");
		break;
	case 4: //save
	{
		TStatus st = save();
		if (st == stOK)
			strm->oMsgX(colGREEN, "Save Ok.");
		else
			strm->oMsgX(colRED, "Save error, code=%d", st);

	}
		break;
	case 5: //init
	{
		TStatus st = init();
		if (st == stOK)
			strm->oMsgX(colGREEN, "Init Ok.");
		else
			strm->oMsgX(colRED, "Init error, code=%d", st);

	}
		break;
	default:
		showHelp(strm, "Config Menu", menuCfg);
		break;
	};

}
