/*
 * MdbMaster.cpp
 *
 *  Created on: 14 lut 2022
 *      Author: Grzegorz
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <MdbMaster.h>
#include <utils.h>
#include "ShellItem.h"
#include "shell.h"
#include "Token.h"
#include "Cfg.h"
#include "UMain.h"

extern Shell *shell;
extern Cfg *cfg;

//------------ MdbUart -------------------------------------------------------------------------------
MdbUart::MdbUart(int PortNr, int Priority) :
		TUart::TUart(PortNr, Priority) {
	memset(&rxRec, 0, sizeof(rxRec));
}

HAL_StatusTypeDef MdbUart::Init(int BaudRate, int parity) {
	mUseRts = true;
	mHalfDuplex = false;
	HAL_StatusTypeDef st = TUart::Init(BaudRate, parity);
	rxRec.ptr = 0;

	if (st == HAL_OK) {
		HAL_UART_Receive_IT(&mHuart, &rxRec.rxData, 1);
	}
	return st;
}

void MdbUart::setTxEn(bool txON) {
	switch (mPortNr) {
	case myUART2:
		if (txON)
			HAL_GPIO_WritePin(ZAS_UART2_RTS_GPIO_Port, ZAS_UART2_RTS_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(ZAS_UART2_RTS_GPIO_Port, ZAS_UART2_RTS_Pin, GPIO_PIN_RESET);
		break;
	}
}

void MdbUart::TxCpltCallback() {
	TUart::TxCpltCallback();
	setTxEn(false);
}

void MdbUart::RxCpltCallback() {

	rxRec.tick = HAL_GetTick();
	rxRec.globCnt++;
	if (rxRec.ptr < FRAME_LEN) {
		rxRec.buf[rxRec.ptr] = rxRec.rxData;
		rxRec.ptr++;
	}
	HAL_UART_Receive_IT(&mHuart, &rxRec.rxData, 1);
}

void MdbUart::clearRxBuf() {
	rxRec.ptr = 0;
	rxRec.tick = 0;
}

void MdbUart::writeBuf(const void *buf, int len) {
	setTxEn(true);
	TUart::writeBuf(buf, len);
}

//------------ MdbMaster -------------------------------------------------------------------------------
MdbMaster::MdbMaster(int MdbNr, int PortNr, int Priority) {
	mMdbNr = MdbNr;
	mUart = new MdbUart(PortNr, Priority);
	memset(&state, 0, sizeof(state));
	memset(&reqConsola, 0, sizeof(reqConsola));
	mDbgLevel = cfg->dt.R.mdbDebug;
	menu.tab = NULL;
	menu.baseCnt = 0;

}

HAL_StatusTypeDef MdbMaster::Init(int BaudRate, int parity) {
	return mUart->Init(BaudRate, parity);
}

void MdbMaster::sendMdbFun(ReqSrc reqSrc, uint8_t *buf, int len) {
	TCrc::Set(buf, len);
	len += 2;
	mUart->clearRxBuf();
	mUart->writeBuf(buf, len);
	state.sent.currReq = reqSrc;
	state.sent.devNr = buf[0];
	state.sent.code = buf[1];
	state.sent.tick = HAL_GetTick();
}

void MdbMaster::SetWord(uint8_t *p, uint16_t w) {
	p[0] = w >> 8;
	p[1] = w & 0xff;
}

uint16_t MdbMaster::GetWord(const uint8_t *p) {
	return (p[0] << 8) | p[1];
}

float MdbMaster::GetIntAsFloat(const uint8_t *p) {
	uint16_t w = GetWord(p);
	if (w != 0x8000) {
		int16_t wi = (int16_t) w;
		float f = wi;
		return f;
	} else {
		return NAN;
	}
}

float MdbMaster::GetFloat(const uint8_t *p) {
	float w;
	uint16_t *pw = (uint16_t*) &w;
	pw[0] = GetWord(&p[0]);
	pw[1] = GetWord(&p[2]);
	return w;
}

//Holding Registers
void MdbMaster::sendMdbFun3(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt) {
	state.sent.regAdr = regAdr;
	state.sent.regCnt = regCnt;

	sendBuf[0] = DevNr;
	sendBuf[1] = 3;
	SetWord(&sendBuf[2], regAdr - 1);
	SetWord(&sendBuf[4], regCnt);
	sendMdbFun(reqSrc, sendBuf, 6);
}

//Input Registers
void MdbMaster::sendMdbFun4(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt) {
	state.sent.regAdr = regAdr;
	state.sent.regCnt = regCnt;

	sendBuf[0] = DevNr;
	sendBuf[1] = 4;
	SetWord(&sendBuf[2], regAdr - 1);
	SetWord(&sendBuf[4], regCnt);
	sendMdbFun(reqSrc, sendBuf, 6);
}

void MdbMaster::sendMdbFun6(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regVal) {
	state.sent.regAdr = regAdr;
	state.sent.regVal = regVal;

	sendBuf[0] = DevNr;
	sendBuf[1] = 6;
	SetWord(&sendBuf[2], regAdr - 1);
	SetWord(&sendBuf[4], regVal);
	sendMdbFun(reqSrc, sendBuf, 6);
}

void MdbMaster::sendMdbFun16(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt, uint16_t *regTab) {
	if (regCnt < 120) {
		state.sent.regAdr = regAdr;
		state.sent.regCnt = regCnt;

		sendBuf[0] = DevNr;
		sendBuf[1] = 16;
		SetWord(&sendBuf[2], regAdr - 1);
		SetWord(&sendBuf[4], regCnt);
		sendBuf[6] = regCnt << 1;
		for (int i = 0; i < regCnt; i++) {
			SetWord(&sendBuf[7 + 2 * i], regTab[i]);
		}
		sendMdbFun(reqSrc, sendBuf, 7 + 2 * regCnt);
	}
}

void MdbMaster::sendConsolaReq() {
	if (state.sent.currReq == reqEMPTY) {
		switch (reqConsola.fun) {
		case 3:
			sendMdbFun3(reqCONSOLA, reqConsola.devNr, reqConsola.regAdr, reqConsola.regCnt);
			break;
		case 4:
			sendMdbFun4(reqCONSOLA, reqConsola.devNr, reqConsola.regAdr, reqConsola.regCnt);
			break;
		case 6:
			sendMdbFun6(reqCONSOLA, reqConsola.devNr, reqConsola.regAdr, reqConsola.regVal[0]);
			break;
		case 16:
			sendMdbFun16(reqCONSOLA, reqConsola.devNr, reqConsola.regAdr, reqConsola.regCnt, reqConsola.regVal);
			break;
		}
		reqConsola.fun = 0;
	}
}

void MdbMaster::buidMsgRec(MsgV *m) {
	m->err = (mDbgLevel >= 1) || (state.sent.currReq == reqCONSOLA);
	m->info = (mDbgLevel >= 2) || (state.sent.currReq == reqCONSOLA);
	m->dat = (mDbgLevel >= 3) || (state.sent.currReq == reqCONSOLA);
}

void MdbMaster::proceedRecFrame() {
	int n = mUart->getRxCharCnt();
	if (n > 4) {
		const uint8_t *inBuf = mUart->getRxBuf();
		MsgV m;
		buidMsgRec(&m);

		if (TCrc::Check(inBuf, n)) {
			if (inBuf[0] != state.sent.devNr) {
				if (m.err)
					getOutStream()->oMsgX(colRED, "MDB%u: replay DEVNR not agree: %u<->%u", mMdbNr, state.sent.devNr, inBuf[0]);
			} else {
				uint8_t rxCmd = inBuf[1] & 0x7F;
				if (rxCmd != state.sent.code) {
					if (m.err)
						getOutStream()->oMsgX(colRED, "MDB%u: replay FUN not agree: %u<->%u", mMdbNr, state.sent.code, rxCmd);
				} else {
					state.sent.currReq = reqEMPTY;
					if ((inBuf[1] & 0x80) == 0) {
						switch (rxCmd) {
						case 3:
						case 4: {
							int n = inBuf[2] >> 1;
							if (n != state.sent.regCnt) {
								if (m.err)
									getOutStream()->oMsgX(colRED, "MDB%u: Fun%u REPLY ERROR", mMdbNr, rxCmd);
								onReciveData(false, rxCmd, NULL, 0);

							} else {
								if (m.dat) {
									char txt[200];
									int m = snprintf(txt, sizeof(txt), "MDB%u: %04X>", mMdbNr, state.sent.regAdr);
									for (int i = 0; i < n; i++) {
										uint16_t w = GetWord(&inBuf[3 + 2 * i]);
										m += snprintf(&txt[m], sizeof(txt) - m, "%02X,", w);
									}
									getOutStream()->oMsgX(colWHITE, txt);
								}
								onReciveData(true, rxCmd, &inBuf[3], n);
							}

						}
							break;
						case 6: {
							uint16_t reg = GetWord(&inBuf[2]) + 1;
							uint16_t v = GetWord(&inBuf[4]);
							if ((reg == state.sent.regAdr) && (v == state.sent.regVal)) {
								onReciveData(true, rxCmd, NULL, 0);
								if (m.info)
									getOutStream()->oMsgX(colWHITE, "MDB%u: Fun6 ACK", mMdbNr);
							} else {
								if (m.err)
									getOutStream()->oMsgX(colRED, "MDB%u: Fun6 ACK ERROR", mMdbNr);
							}
						}
							break;
						case 16: {
							uint16_t reg = GetWord(&inBuf[2]) + 1;
							uint16_t cnt = GetWord(&inBuf[4]);
							if ((reg == state.sent.regAdr) && (cnt == state.sent.regCnt)) {
								onReciveData(true, rxCmd, NULL, 0);
								if (m.info)
									getOutStream()->oMsgX(colWHITE, "MDB%u: Fun16 ACK", mMdbNr);
							} else {
								onReciveData(false, rxCmd, NULL, 0);
								if (m.err)
									getOutStream()->oMsgX(colRED, "MDB%u: Fun16 ACK ERROR", mMdbNr);
							}
						}
							break;
						}
					} else {
						onReciveData(false, rxCmd, NULL, 0);
						if (m.err)
							getOutStream()->oMsgX(colRED, "MDB%u: Modbus exception %u", mMdbNr, inBuf[2]);
					}
				}

			}

		} else {
			state.crcErrCnt++;
		}
	}
	mUart->clearRxBuf();

}

void MdbMaster::tick() {
	uint32_t tt = HAL_GetTick();

	MsgV m;
	buidMsgRec(&m);

	uint32_t tr = mUart->getLastRxCharTick();
	if (tr != 0 && tt - tr > 10) {
		state.proceedFrameTick = HAL_GetTick();
		proceedRecFrame();
	}

	if (reqConsola.fun != 0) {
		sendConsolaReq();
	}

	//czy TimeOUT  dla funkcji Modbus
	if (state.sent.currReq != reqEMPTY) {
		if (tt - state.sent.tick > MAX_MDB_REPL_TM) {
			state.sent.currReq = reqEMPTY;
			state.timeOutCnt++;
			doOnTimeOut();
			if (m.err)
				getOutStream()->oMsgX(colRED, "MDB%u: TimeOut DevNr=%u Fun=%u", mMdbNr, state.sent.devNr, state.sent.code);
		}
	}

}

void MdbMaster::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("TimeOutCnt=%d", state.timeOutCnt);
		strm->oMsg("CrcErrCnt=%d", state.crcErrCnt);

		strm->oClose();
	}
}

const ShellItem menuMdb[] = { //
		{ "dbg", "zmien poziom logów (0..4)" }, //
				{ "s", "stan" }, //
				{ "rdreg", "read registers MDB3: devNr,Addr,cnt" }, //
				{ "rdinp", "read analog input MDB4: devNr,Addr,cnt" }, //
				{ "wrreg", "write register MDB6: devNr,Addr,val" }, //
				{ "wrmul", "write registers MDB16: devNr,Addr,val1,val2,..valX" }, //

				{ NULL, NULL } };

void MdbMaster::buildMenu(const ShellItem *toAddMenu) {
	const ShellItem *item = menuMdb;
	int k = 0;
	while (item->cmd != NULL) {
		k++;
		item++;
	}
	menu.baseCnt = k;
	item = toAddMenu;
	while (item->cmd != NULL) {
		k++;
		item++;
	}
	menu.tab = (ShellItem*) malloc((k + 1) * sizeof(ShellItem));

	k = 0;
	item = menuMdb;
	while (item->cmd != NULL) {
		menu.tab[k].cmd = item->cmd;
		menu.tab[k].descr = item->descr;
		k++;
		item++;
	}
	item = toAddMenu;
	while (item->cmd != NULL) {
		menu.tab[k].cmd = item->cmd;
		menu.tab[k].descr = item->descr;
		k++;
		item++;
	}
	menu.tab[k].cmd = NULL;
	menu.tab[k].descr = NULL;
}

const ShellItem* MdbMaster::getMenu() {
	return menuMdb;
}

const char* MdbMaster::getMenuName() {
	return "Modbus Menu";
}

void MdbMaster::shell(OutStream *strm, const char *cmd) {
	char tok[20];
	int idx = -1;

	const ShellItem *menu = getMenu();

	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(menu, tok);

	if (!execMenuItem(strm, idx, cmd)) {
		showHelp(strm, getMenuName(), menu);
	}
}

bool MdbMaster::execMenuItem(OutStream *strm, int idx, const char *cmd) {

	switch (idx) {
	case 0:  //dbg
		Token::getAsInt(&cmd, &mDbgLevel);
		break;
	case 1:  //s
		showState(strm);
		break;
	case 2: //rdreg
	case 3: { //rdinp
		int devNr;
		int adr;
		int cnt;
		if (Token::getAsInt(&cmd, &devNr)) {
			if (Token::getAsInt(&cmd, &adr)) {
				if (Token::getAsInt(&cmd, &cnt)) {
					reqConsola.devNr = devNr;
					reqConsola.regAdr = adr;
					reqConsola.regCnt = cnt;
					const char *nm;
					if (idx == 2) {
						reqConsola.fun = 3;
						nm = "RdReg";
					} else {
						reqConsola.fun = 4;
						nm = "RdInp";
					}
					strm->oMsgX(colWHITE, "DevNr=%u %s %u,%u", reqConsola.devNr, nm, reqConsola.regAdr, reqConsola.regCnt);
				}
			}
		}
	}
		break;

	case 4: { //wrreg
		int devNr;
		int adr;
		int val;
		if (Token::getAsInt(&cmd, &devNr)) {
			if (Token::getAsInt(&cmd, &adr)) {
				if (Token::getAsInt(&cmd, &val)) {
					reqConsola.fun = 6;
					reqConsola.devNr = devNr;
					reqConsola.regAdr = adr;
					reqConsola.regVal[0] = val;
					strm->oMsgX(colWHITE, "DevNr=%u WrReg %u: %u", reqConsola.devNr, reqConsola.regAdr, reqConsola.regVal[0]);
				}
			}
		}
	}
		break;
	case 5: { //wrmul
		int devNr;
		int adr;
		if (Token::getAsInt(&cmd, &devNr)) {
			if (Token::getAsInt(&cmd, &adr)) {
				int n = 0;
				while (n < MAX_VAL_CNT) {
					int val;
					if (Token::getAsInt(&cmd, &val)) {
						reqConsola.regVal[n] = val;
						n++;
					} else
						break;
				}
				if (n > 0) {
					reqConsola.fun = 16;
					reqConsola.devNr = devNr;
					reqConsola.regAdr = adr;
					reqConsola.regCnt = n;
					strm->oMsgX(colWHITE, "DevNr=%u WrMulReg %u: n=%u", reqConsola.devNr, reqConsola.regAdr, n);
				}
			}
		}
	}
		break;
	default:
		return false;
	};
	return true;
}

//-----------------------------------------------------------------------------------------
// MdbPAPI
//-----------------------------------------------------------------------------------------
const ShellItem menuMdbPapi[] = { //
		{ "m", "stan PAPI" }, //
				{ NULL, NULL } };

MdbPAPI::MdbPAPI(int PortNr, int Priority) :
		MdbMaster::MdbMaster(1, PortNr, Priority) {
	buildMenu(menuMdbPapi);
	memset(&autoRd, 0, sizeof(autoRd));
}

const ShellItem* MdbPAPI::getMenu() {
	return menu.tab;
}

bool MdbPAPI::execMenuItem(OutStream *strm, int idx, const char *cmd) {
	bool q = true;
	switch (idx - menu.baseCnt) {
	case 0:
		break;
	default:
		q = MdbMaster::execMenuItem(strm, idx, cmd);
	}
	return q;
}

const char* MdbPAPI::getMenuName() {
	return "MdbPAPI Menu";
}

void MdbPAPI::doOnTimeOut() {
	MsgV m;
	buidMsgRec(&m);

	switch (autoRd.phase) {
	case 2:
	case 4:
	case 6:
	case 12:
	case 16:
		autoRd.phase = 0;
		globalVar.projektOk = false;
		if (m.err) {
			switch (autoRd.phase) {
			case 2:
				getOutStream()->oMsgX(colRED, "MDB%u: TIMEOUT, write regLEVEL_CPX", mMdbNr);
				break;
			case 4:
				getOutStream()->oMsgX(colRED, "MDB%u: read Projector TIMEOUT", mMdbNr);
				break;
			case 6:
				getOutStream()->oMsgX(colRED, "MDB%u: TIMEOUT, write PWM1..4", mMdbNr);
				break;
			case 12:
				getOutStream()->oMsgX(colRED, "MDB%u: TIMEOUT, write SET_TILT", mMdbNr);
				break;
			case 16:
				getOutStream()->oMsgX(colRED, "MDB%u: TIMEOUT, write CTRL_EX", mMdbNr);
				break;
			}

		}
		break;

	default:
		if (m.err)
			getOutStream()->oMsgX(colRED, "MDB%u: TIMEOUT", mMdbNr);
		break;
	}
	if (autoRd.phase != 0) {
		strcpy(autoRd.statusTxt, "TimeOut");
	}
}

void MdbPAPI::onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
	if (autoRd.phase > 0) {
		if (replOK) {

			switch (autoRd.phase) {
			case 2:
			case 6:
				globalVar.projektOk = true;
				papiState.wrPowerTick = HAL_GetTick();
				autoRd.phase = 3;
				break;
			case 4:
				globalVar.projektOk = true;
				globalVar.projector.devStatus2 = GetWord(&tab[0]);
				globalVar.projector.devStatus = GetWord(&tab[2]);
				globalVar.projector.tempSens1 = GetIntAsFloat(&tab[4]) / 10;
				globalVar.projector.tempSens2 = GetIntAsFloat(&tab[6]) / 10;
				globalVar.projector.tempSht35 = GetIntAsFloat(&tab[8]) / 10;
				globalVar.projector.humiditiSht35 = GetIntAsFloat(&tab[10]);
				globalVar.projector.angX = GetIntAsFloat(&tab[12]) / 10;
				globalVar.projector.angY = GetIntAsFloat(&tab[14]) / 10;

				globalVar.newProjVal = true;

				autoRd.phase = 0;
				autoRd.redTick = HAL_GetTick();
				strcpy(autoRd.statusTxt, "OK");
				break;
			case 16:
			case 12:
				autoRd.phase = 0;
				break;

			}
		} else {
			globalVar.projektOk = false;
		}
	}

}

// automat wymiany danych przez modbus
// zadania:
// 1. odczyt danych z zasilacza
// 2. odczyt statusu z PapiProjektor
// 3. wysłanie danych do PapiProjektor
// 4. wysłanie statusu do zasilacza

void MdbPAPI::tick() {

	MdbMaster::tick();

	if (autoRd.phase == 0) {
		// po naciśnięciu klawisza Set_Tilt
		if (autoRd.triger.setBasePosition) {
			autoRd.triger.setBasePosition = 0;
			autoRd.phase = 11;
			getOutStream()->oMsgX(colYELLOW, "sendSetBasePosition");
		}
	}

	if (autoRd.phase == 0) {
		//zmian położenia DipSwitch - zapis rejestru regCTRL_EX
		if (autoRd.triger.sendDipSwitchPos) {
			autoRd.triger.sendDipSwitchPos = 0;
			autoRd.phase = 15;
			getOutStream()->oMsgX(colYELLOW, "sendDipSwitchPos");
		}
	}

	if (autoRd.phase == 0) {
		if ((HAL_GetTick() - autoRd.tick > TM_AUTO_RD) || autoRd.triger.startNow) {
			autoRd.tick = HAL_GetTick();
			autoRd.triger.startNow = false;
			if (!globalVar.encoder.autom || globalVar.kyEmergency) {
				//zapis rejestru regLEVEL_CPX
				autoRd.phase = 1;
			} else {
				//zapis do rejestru regPWM1, ... , regPWM4
				autoRd.phase = 5;
			}
		}
	}

	if (autoRd.phase != 0) {
		switch (autoRd.phase) {
		case 1:
			//zapis rejestru regLEVEL_CPX
			if (state.sent.currReq == reqEMPTY) {
				autoRd.phase = 2;
				sendMdbFun6(reqSYS, cfg->dt.R.S847_MdbNr, regLEVEL_CPX, globalVar.getLevelCpx());
			}

			break;
		case 3:
			if (HAL_GetTick() - state.proceedFrameTick > 50) {
				if (state.sent.currReq == reqEMPTY) {
					autoRd.phase = 4;
					autoRd.reqCnt++;
					sendMdbFun4(reqSYS, cfg->dt.R.S847_MdbNr, regDEV_STATUS2, 8);
				}
			}
			break;
		case 5:
			if (state.sent.currReq == reqEMPTY) {
				autoRd.phase = 6;
				uint16_t tab[4];
				globalVar.getAutoTab(tab);
				sendMdbFun16(reqSYS, cfg->dt.R.S847_MdbNr, regPWM1, 4, tab);
			}
			break;

		case 11:
			sendMdbFun6(reqSYS, cfg->dt.R.S847_MdbNr, regSET_TILT, SET_TILT_VAL);
			autoRd.phase = 12;
			break;
		case 15:
			sendMdbFun6(reqSYS, cfg->dt.R.S847_MdbNr, regCTRL_EX, globalVar.dpSw.val);
			autoRd.phase = 12;
			break;
		case 2:
		case 4:
		case 6:
		case 12:
		case 16:
			if (HAL_GetTick() - state.sent.tick > MAX_TIME_REPL)
				autoRd.phase = 0;
			break;
		default:
			autoRd.phase = 0;
			break;
		}
	}

}

void MdbPAPI::sendLightState() {
	autoRd.triger.startNow = true;
}

void MdbPAPI::sendSetBasePosition() {
	autoRd.triger.setBasePosition = true;
}

//zmian na manipulatorze pozycji DipSwitchy lub Encodera
void MdbPAPI::sendManipChg() {
	autoRd.triger.sendDipSwitchPos = true;
	autoRd.triger.startNow = true;
}
