/*
 * uMain.cpp
 *
 *  Created on: Feb 10, 2022
 *      Author: Grzegorz
 */

#include "uMain.h"
#include "utils.h"
#include "Hdw.h"
#include "shell.h"
#include "cfg.h"
#include "MdbMaster.h"
#include "XBee.h"
#include <I2cDev.h>

GlobalVar globalVar;
VerInfo mSoftVer;

Cfg *cfg;
Shell *shell;
MdbPAPI *mdbPapi;
XBee *xBee = NULL;
I2cBus *i2c1 = NULL;
PapiManipDevPub *papiManip = NULL;
TidsDevPub *tids = NULL;
Veml6030DevPub *veml = NULL;
extern IWDG_HandleTypeDef hiwdg;

//------------------------------------------------------------------------------
//         Label
//------------------------------------------------------------------------------
#define SEC_LABEL   __attribute__ ((section (".label")))

SEC_LABEL char DevLabel[] = "------------"
		"TTT             "
		"                "
		"                "
		"                "
		"****************"
		"* EMAG Serwis  *"
		"* PAPI-Ctrl    *"
		"****************";

extern "C" OutStream* getOutStream() {
	return shell;
}

void GlobalVar::setDipSw(uint8_t v) {
	dpSw.val = v & 0x1F;
	dpSw.IR_Visual = ((v & 0x01) != 0);
	dpSw.photoActive = ((v & 0x02) != 0);
	dpSw.nightLevel = ((v & 0x04) != 0);
	dpSw.tiltBypass = ((v & 0x08) != 0);
	dpSw.heaterEnable = ((v & 0x10) != 0);
	manipChg = true;
}

void GlobalVar::setEncoder(uint8_t v) {
	encoder.remote = (v == encREMOTE);
	encoder.autom = (v == encAUTO);
	if (v >= encOFF)
		encoder.level = v - encOFF;
	manipChg = true;
}

void GlobalVar::setRemoteVal(uint16_t val) {
	if (val != remoteDt.val) {
		remoteDt.val = val;
		remoteDt.chg = true;
	}
}

void GlobalVar::setNoPulpit() {
	setEncoder(cfg->dt.R.altPulp.encoder);
	setDipSw(cfg->getAltDipSwitch());
	manipChg = true;
}

uint16_t GlobalVar::getLevelCpx() {
	uint16_t w = 0;

	uint16_t lev = encoder.level & 0x0F;
	if (kyEmergency)
		lev = 7;

	if (dpSw.IR_Visual) {
		//kanały widzialne: WHITE+RED
		w |= lev;
		w |= (lev << 8);
	} else {
		//kanały niewidzialne: IR1 + IR2
		w |= (lev << 4);
		w |= (lev << 12);
	}
	return w;
}

void GlobalVar::checkDay_Night() {

	if (!local.isNight) {
		if (local.vemlLight < cfg->dt.R.lighNightLimit) {
			local.isNight = true;
			sendLightState = true;
		}
	} else {
		if (local.vemlLight > cfg->dt.R.lighNightLimit + cfg->dt.R.lighNightLimitHist) {
			local.isNight = false;
			sendLightState = true;
		}
	}
}

//tab - tablica o długości 4 !!!!
void GlobalVar::getAutoTab(uint16_t *tab) {
	uint16_t pwmW = 0;
	uint16_t pwmR = 0;
	uint16_t pwmIR = 0;
	if (encoder.autom) {
		if (dpSw.photoActive) {
			if (local.isNight) {
				if (dpSw.nightLevel) {
					pwmW = 10 * cfg->dt.R.nightLevelWhite1;
					pwmR = 10 * cfg->dt.R.nightLevelRed1;
					pwmIR = 10 * cfg->dt.R.nightLevelIR1;
				} else {
					pwmW = 10 * cfg->dt.R.nightLevelWhite2;
					pwmR = 10 * cfg->dt.R.nightLevelRed2;
					pwmIR = 10 * cfg->dt.R.nightLevelIR2;
				}
			}
		}
	} else if (encoder.remote) {
		pwmW = remoteDt.val;
		pwmR = remoteDt.val;
		pwmIR = remoteDt.val;
	}





	tab[0] = tab[1] = tab[2] = tab[3] = 0;
	if (dpSw.IR_Visual) {
		//kanały widzialne: WHITE+RED
		tab[0] = pwmW;
		tab[2] = pwmR;
	} else {
		//kanały niewidzialne: IR1+IR2
		tab[1] = pwmIR;
		tab[3] = pwmIR;
	}
}

#define PROJ_ERR_MASK 0xF8FF

bool GlobalVar::getGlobError() {
	bool q = false;
	if (!manipOk)
		q = true;
	if (!projektOk) {
		q = true;
	} else {
		if (projector.devStatus != 0)
			q = true;
		if (projector.devStatus2 != 0)
			q = true;
	}
	return q;
}

extern "C" void execTiltSetKey(bool key) {
	static bool memKey = false;
	static uint32_t tick = 0;
	static uint32_t showTick = 0;

	if (key != memKey) {

		if (tick == 0)
			tick = HAL_GetTick();
		if (HAL_GetTick() - tick > 500) {
			memKey = key;
			globalVar.kyTiltSet = key;
			tick = 0;
			if (key) {
				mdbPapi->sendSetBasePosition();
				showTick = HAL_GetTick();
			}
		}

	} else {
		tick = 0;
	}
	if (showTick != 0) {
		uint32_t dt = HAL_GetTick() - showTick;
		if (dt < 2000) {
			uint32_t dxt = dt % 200;
			bool qq = dxt < 100;
			Hdw::ledKey1(qq);
		} else {
			Hdw::ledKey1(false);
			showTick = 0;
		}
	} else {
		Hdw::ledKey1(false);
	}
}

extern "C" void execSetAltPhotoActive(bool activ) {
	cfg->dt.R.altPulp.photoActive = activ;
	cfg->save();
	globalVar.dpSw.photoActive = activ;
	globalVar.manipChg = true;
}

struct {
	volatile bool flag;
	uint32_t tick;
	uint32_t time;
} rebootRec = { 0 };

extern "C" void reboot(int tm) {
	rebootRec.tick = HAL_GetTick();
	rebootRec.time = tm;
	rebootRec.flag = true;
}

extern "C" void MxStart(void);

extern "C" void wdgPulse() {
//	HAL_IWDG_Refresh(&hiwdg);
}

extern "C" void uMain() {

	MxStart();
	loadSoftVer(&mSoftVer, &DevLabel[0x1C]);

	cfg = new Cfg();
	cfg->init();

	shell = new Shell();
	shell->Init(115200);
	shell->oMsgX(colWHITE, "\r\n\n______PAPI_Ctrl_____");

	mdbPapi = new MdbPAPI(TUart::myUART2, 2);
	mdbPapi->Init(cfg->GetMdbSpeed(), TUart::parityNONE);

	xBee = new XBee(TUart::myUART3, 3);
	xBee->Init(9600);

	i2c1 = new I2cBus(I2cBus::iicBUS1);
	papiManip = PapiManipDevPub::createDev(i2c1, 0x5E);
	tids = TidsDevPub::createDev(i2c1, 0x7E);
	tids->mReadAuto = true;

	veml = Veml6030DevPub::createDev(i2c1, 0x20);
	veml->mReadAuto = true;

	//odczytnie stanu początkowego z manipulatora
	uint8_t encoder;
	uint8_t dipSwitch;
	HAL_StatusTypeDef st = papiManip->getData(&encoder, &dipSwitch);
	if (st == HAL_OK) {
		globalVar.setEncoder(encoder);
		globalVar.setDipSw(dipSwitch);
	} else {
		globalVar.setNoPulpit();
	}

	bool led = false;
	uint32_t ledTm = HAL_GetTick();
	while (1) {
		if (HAL_GetTick() - ledTm > 250) {
			ledTm = HAL_GetTick();
			Hdw::ledOK(led);
			led = !led;
			if (globalVar.getGlobError() == false) {
				Hdw::ledR(false);
				Hdw::ledG(led);
			} else {
				Hdw::ledR(led);
				Hdw::ledG(false);
			}
			tids->getData(&globalVar.local.tidsTemp);
			veml->getData(&globalVar.local.vemlLight);
			globalVar.checkDay_Night();
		}
		shell->tick();
		xBee->tick();
		i2c1->tick();

		execTiltSetKey(Hdw::rdKey1());

		bool ky2 = Hdw::rdKey2();
		bool ky2Chg = (ky2 != globalVar.kyEmergency);
		globalVar.kyEmergency = ky2;
		Hdw::ledKey2(ky2);
		if (ky2Chg || globalVar.remoteDt.chg) {
			globalVar.remoteDt.chg = false;
			mdbPapi->sendLightState();
		}

		if (globalVar.manipChg) {
			globalVar.manipChg = false;
			mdbPapi->sendManipChg();
		}
		if (globalVar.sendLightState) {
			globalVar.sendLightState = false;
			mdbPapi->sendLightState();
		}

		mdbPapi->tick();

		if (rebootRec.flag) {
			if (HAL_GetTick() - rebootRec.tick > rebootRec.time) {
				NVIC_SystemReset();
			}
		}
		wdgPulse();
	}

}
