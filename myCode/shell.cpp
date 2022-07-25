/*
 * shell.cpp
 *
 *  Created on: 21 lip 2021
 *      Author: Grzegorz
 */

#include "stdlib.h"
#include "string.h"
#include "stdio.h"

#include "shell.h"
#include "EscTerminal.h"
#include "ShellItem.h"
#include "Token.h"
#include "Cfg.h"
#include "Hdw.h"
#include "uMain.h"
#include "MdbMaster.h"
#include "XBee.h"
#include "uMain.h"
#include <I2cDev.h>

extern Cfg *cfg;
extern VerInfo mSoftVer;
extern MdbPAPI *mdbPapi;
extern XBee *xBee;
extern I2cBus *i2c1;

extern "C" void execSetAltPhotoActive(bool activ);

//---------------------------------------------------------

Shell *Shell::Me;

Shell::Shell() :
		TUart::TUart(myUART5, 1) {
	Me = this;
	rxBuf = new RxTxBuf(64);
	txBuf = new RxTxBuf(2048);
	term = new EscTerminal(this);
	memset(&rxRec, 0, sizeof(rxRec));
}

HAL_StatusTypeDef Shell::Init(int BaudRate) {
	HAL_StatusTypeDef st = TUart::Init(BaudRate);
	StartRecive();
	return st;
}

void Shell::putOut(const void *mem, int len) {
	flgSendAny = true;
	Portion portion;
	portion.dt = (const char*) mem;
	portion.len = len;
	txBuf->addBuf(&portion);
}

void Shell::StartRecive() {
	rxRec.reciveSt = HAL_UART_Receive_IT(&mHuart, (uint8_t*) &rxRec.rxChar, 1);
}

void Shell::RxCpltCallback() {
	rxBuf->add(rxRec.rxChar);
	StartRecive();
}

void Shell::TxCpltCallback() {
	TUart::TxCpltCallback();
	trySend();
}

void Shell::ErrorCallback() {
	rxRec.ErrCnt++;
}

void Shell::trySend() {

	if (!isSending()) {
		if (txBuf->isLockRegion())
			txBuf->unlockRegion();
		const char *ptr;
		int cnt;

		if (txBuf->lockLinearRegion(&ptr, &cnt, 0x100)) {
			writeBuf(ptr, cnt);
		}
	}

}

void Shell::showHdwState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("KEY1    : %u", Hdw::rdKey1());
		strm->oMsg("KEY2    : %u", Hdw::rdKey2());
		strm->oMsg("GPS_PPS : %u", Hdw::rdGpsPPS());
		strm->oMsg("EXP_INT1: %u", Hdw::rdExpInt1());
		strm->oMsg("EXP_INT2: %u", Hdw::rdExpInt2());
		strm->oClose();
	}
}

void Shell::showState() {
	static int showCnt = 0;
	if (oOpen(colYELLOW)) {
		oMsg("\nStan aktualny : %u", showCnt++);
		oMsg("----------------------");
		oSetColor(colWHITE);
		oMsg("Wersja : %u.%03u", mSoftVer.ver, mSoftVer.rev);
		oMsg("GlobVar-local");
		oMsg(" temp.TIDS  :%.1f[*C]", globalVar.local.tidsTemp);
		oMsg(" light.VEML :%.1f[lux]", globalVar.local.vemlLight);
		oMsg(" isNight    :%u", globalVar.local.isNight);

		oMsg("GlobVar-manipulator");
		oMsg(" ManipOk     :%u", globalVar.manipOk);
		oMsg(" Auto        :%u", globalVar.encoder.autom);
		oMsg(" Remote      :%u", globalVar.encoder.remote);
		oMsg(" Level       :%u", globalVar.encoder.level);
		oMsg(" IR_Visual   :%u", globalVar.dpSw.IR_Visual);
		oMsg(" HeaterEn    :%u", globalVar.dpSw.heaterEnable);
		oMsg(" NightLevel  :%u", globalVar.dpSw.nightLevel);
		oMsg(" PhotoActive :%u", globalVar.dpSw.photoActive);
		oMsg(" TiltBypass  :%u", globalVar.dpSw.tiltBypass);
		oMsg(" EmergencyKey:%u", globalVar.kyEmergency);
		oMsg("Remote");
		oMsg(" LastVal     :%.1f[%%]", globalVar.remoteDt.val/10.0);

		oMsg("GlobVar-projektor");
		oMsg(" ProjektOk  :%u", globalVar.projektOk);
		oMsg(" devStatus  :0x%04X", globalVar.projector.devStatus);
		oMsg(" devStatus2 :0x%04X", globalVar.projector.devStatus2);
		oMsg(" temp.Sens1 :%.1f[*C]", globalVar.projector.tempSens1);
		oMsg(" temp.Sens2 :%.1f[*C]", globalVar.projector.tempSens2);
		oMsg(" temp.SHT35 :%.1f[*C]", globalVar.projector.tempSht35);
		oMsg(" humidity   :%.0f[%%]", globalVar.projector.humiditiSht35);
		oMsg(" ang.dX     :%.1f[*]", globalVar.projector.angX);
		oMsg(" ang.dY     :%.1f[*]", globalVar.projector.angY);

		oClose();
	}

}

void Shell::setLeds(const char *cmd) {
	char tok[10];
	Token::get(&cmd, tok, sizeof(tok));
	if (strlen(tok) == 3) {
		Hdw::ledKey1(tok[0] == '1');
		Hdw::ledKey2(tok[1] == '1');
		uint8_t w = tok[2] - '0';
		Hdw::ledR((w & 0x01) != 0);
		Hdw::ledG((w & 0x02) != 0);
	} else
		oMsgX(colRED, "Use: leds 112");

}

const ShellItem mainMenu[] = { //
		{ "s", "[F2] status urządzenia" }, //
				{ "h", "[F3] stan hardware" }, //
				{ "alt_PhotoAct_0", "[F4] ustawienie alt_photoActive=0" }, //
				{ "alt_PhotoAct_1", "[F5] ustawienie alt_photoActive=1" }, //
				{ "reboot", "reboot procesora" }, //
				{ "cfg", ">> menu konfiguracji" }, //
				{ "mdb", ">> modbus menu" }, //
				{ "xbee", ">> xbee menu" }, //
				{ "i2c", ">> i2c bus" }, //
				{ "sendS847", "wymuszenie wyslania ustawien do S847" }, //
				{ "setTilt", "rozkaz do S847: ustaw położenie bazowe" }, //

				{ NULL, NULL } };

void Shell::execCmdLine(const char *cmd) {

	char tok[20];
	int idx = -1;
	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(mainMenu, tok);
	switch (idx) {
	case 0: //s
		showState();
		break;
	case 1: //h
		showHdwState(this);
		break;
	case 2: // alt_PhotoAct_0
		oMsgX(colWHITE, "Set  alt_photoActive=0");
		execSetAltPhotoActive(false);
		break;
	case 3: // alt_PhotoAct_1
		oMsgX(colWHITE, "Set  alt_photoActive=1");
		execSetAltPhotoActive(true);
		break;

	case 4: //reboot
		oMsgX(colRED, "R E B O O T");
		reboot(1000);
		break;

	case 5: //cfg
		cfg->shell(this, cmd);
		break;
	case 6: //mdb
		mdbPapi->shell(this, cmd);
		break;
	case 7: //xbee
		xBee->shell(this, cmd);
		break;
	case 8: //i2c1
		i2c1->shell(this, cmd);
		break;
	case 9: //sendS847
		globalVar.setNoPulpit();
		oMsgX(colWHITE, "Send  settings to S847");
		break;
	case 10: //setTilt
		mdbPapi->sendSetBasePosition();
		oMsgX(colWHITE, "Set base position in S847");
		break;

	default: {
		char txt[100];
		snprintf(txt, sizeof(txt), "PAPI_Ctrl MainMenu");
		showHelp(this, txt, mainMenu);
	}
		break;
	}
}

void Shell::execAltChar(char altChar) {
	oMsgX(colRED, "AltChar=%u [%c]", altChar, altChar);
}

void Shell::execFunKey(FunKey funKey) {
	//msg(colYELLOW, "FunKey=%u", funKey);
	switch (funKey) {
	default:
	case fnF1:
		showHelp(this, "MainMenu", mainMenu);
		break;
	case fnF2:
		showState();
		break;
	case fnF3:
		showHdwState(this);
		break;
	case fnF4:
		execSetAltPhotoActive(false);
		break;
	case fnF5:
		execSetAltPhotoActive(true);
		break;
	}
}

void Shell::tick() {
	trySend();
	char key;
	if (rxBuf->pop(&key)) {
		TermAct act = term->inpChar(key);

		switch (act) {
		case actNOTHING:
			break;
		case actLINE:
			flgSendAny = false;
			execCmdLine(term->mCmd);
			if (!flgSendAny)
				term->showLineMx();
			break;
		case actALTCHAR:
			execAltChar(term->mAltChar);
			break;
		case actFUNKEY:
			execFunKey(term->mFunKey);
			break;
		}
	}
}
