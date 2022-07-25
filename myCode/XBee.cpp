/*
 * XBee.cpp
 *
 *  Created on: 20 mar 2022
 *      Author: Grzegorz
 */

#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include <main.h>
#include <Cfg.h>
#include <ShellItem.h>
#include <Token.h>
#include <cfg.h>

#include <XBee.h>
#include <uMain.h>

extern Cfg *cfg;

//---------------------------------------------------------------------------------------------
// API Frames
//---------------------------------------------------------------------------------------------
enum {
	apiAT_CMD = 0x08, //AT Command
	apiAT_CMD_REQ = 0x09, //AT Command - Queue Parameter Value
	apiTRANS_REQ = 0x10, //Transmit Request
	apiADR_CMD = 0x11, //Explicit Addressing Command Frame
	apiRMT_CMD_REQ = 0x17, //Remote Command Request
	apiAT_CMD_RESP = 0x88, //AT Command Response
	apiMODEM_STATUS = 0x8A, //Modem Status
	apiTRANSM_STATUS = 0x8B, //Transmit Status
	apiREC_PKT = 0x90, //Receive Packet (AO=0)
	apiREC_INDICATOR = 0x91, //Explicit Rx Indicator (AO=1)
	apiDATA_RX_INDICATOR = 0x92, //I/O Data Sample RX Indicator
	apiNODE_INDICATOR = 0x95, //Node Identification Indicator (AO=0)
	apiRMT_CMD_RESP = 0x97, //Remote Command Response
};

XBee::XBee(int PortNr, int Priority) :
		TUart::TUart(PortNr, Priority) {
	mDbgLevel = cfg->dt.R.xbeeDebug;
	mHexMode = true;
	mAtCmdIdx = 0;
	mAtCmdIdx = 1;

	memset(&rxRec, 0, sizeof(rxRec));
	memset(&txRec, 0, sizeof(txRec));

}

HAL_StatusTypeDef XBee::Init(int BaudRate) {
	HAL_StatusTypeDef st = TUart::Init(BaudRate);

	if (st == HAL_OK) {
		resetPulse();
		HAL_UART_Receive_IT(&mHuart, &rxRec.rxData, 1);
	}
	return st;
}

void XBee::TxCpltCallback() {
	TUart::TxCpltCallback();
	txRec.cpltCnt++;
}

void XBee::RxCpltCallback() {

	rxRec.tick = HAL_GetTick();
	rxRec.globCnt++;
	if (rxRec.ptr < FRAME_LEN) {
		rxRec.buf[rxRec.ptr] = rxRec.rxData;
		rxRec.ptr++;
	}
	HAL_UART_Receive_IT(&mHuart, &rxRec.rxData, 1);
}

void XBee::setResetHd(bool res) {
	res = !res;
	HAL_GPIO_WritePin(X_RST_GPIO_Port, X_RST_Pin, (GPIO_PinState) res);
}

void XBee::setSleepReqHd(bool sleep) {
	HAL_GPIO_WritePin(X_SLEEP_REQ_GPIO_Port, X_SLEEP_REQ_Pin, (GPIO_PinState) sleep);
}

void XBee::resetPulse() {
	setResetHd(true);
	setResetHd(true);
	HAL_Delay(3);
	setResetHd(false);
	HAL_Delay(5);
	setResetHd(false);
}

void XBee::setTxAsGPIO() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = X_UART3_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(X_UART3_TX_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(X_UART3_TX_GPIO_Port, X_UART3_TX_Pin, GPIO_PIN_RESET);
}

void XBee::setTxAsUART() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = X_UART3_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(X_UART3_TX_GPIO_Port, &GPIO_InitStruct);
}

void XBee::resetPulseToCmMode() {
	setTxAsGPIO();
	setResetHd(true);
	HAL_Delay(3);
	setResetHd(false);
	HAL_Delay(3);
	setTxAsUART();
}

void XBee::longBrkToCmMode() {
	setTxAsGPIO();
	for (int i = 0; i < 7; i++) {
		HAL_Delay(1000);
		wdgPulse();
	}
	setTxAsUART();
}

bool XBee::checkSumFrame(const uint8_t *inp, int len) {
	if (len >= 0) {
		uint8_t sum = 0;
		for (int i = 0; i < len + 1; i++) {
			sum += inp[3 + i];
		}
		return (sum == 0xff);
	} else {
		return false;
	}

}

void XBee::addEnvelope(uint8_t cmdID, const char *inp, int len) {
	if (len < FRAME_LEN - 5) {
		txRec.sndBuf[0] = 0x7E;
		int len1 = len + 1;
		txRec.sndBuf[1] = len1 >> 8;
		txRec.sndBuf[2] = len1 & 0xff;
		txRec.sndBuf[3] = cmdID;
		memcpy(&txRec.sndBuf[4], inp, len);
		uint8_t sum = cmdID;
		for (int i = 0; i < len; i++) {
			sum += txRec.sndBuf[4 + i];
		}
		txRec.sndBuf[len + 4] = 0xff - sum;
		int sndLen = len + 5;
		writeBuf(txRec.sndBuf, sndLen);

		if (mDbgLevel >= 2) {
			if (!mHexMode)
				getOutStream()->oBinBufX(colMAGENTA, txRec.sndBuf, sndLen);
			else
				getOutStream()->oBinBufHexX(colMAGENTA, txRec.sndBuf, sndLen);
			bool q = checkSumFrame((uint8_t*) txRec.sndBuf, sndLen);
			getOutStream()->oMsgX(colMAGENTA, "SumOK=%u", q);

		}

	} else {
		getOutStream()->oMsgX(colRED, "Frame too big to send");
	}
}

//wysyła komendy AT w trybie API1
//inp - komenda dez znaków 'AT'
void XBee::senApiCmd(const char *inp, int len, uint8_t *cmdIdx) {
	char cmd[40];

	if (len < (int) sizeof(cmd) - 1) {
		mAtCmdIdx++;
		cmd[0] = mAtCmdIdx;
		memcpy(&cmd[1], inp, sizeof(cmd) - 1);

		addEnvelope(apiAT_CMD, cmd, len + 1);
		*cmdIdx = mAtCmdIdx;
	}
}

bool XBee::rdStatus() {
	return (HAL_GPIO_ReadPin(X_STATUS_GPIO_Port, X_STATUS_Pin) == GPIO_PIN_SET);
}

bool XBee::rdAttn() {
	return (HAL_GPIO_ReadPin(X_SPI1_ATTN_GPIO_Port, X_SPI1_ATTN_Pin) == GPIO_PIN_SET);
}
bool XBee::rdRssi() {
	return (HAL_GPIO_ReadPin(X_RSSI_GPIO_Port, X_RSSI_Pin) == GPIO_PIN_SET);
}

void XBee::writeStr(const char *dt) {
	while (isSending()) {

	}
	strlcpy(txRec.sndBuf, dt, sizeof(txRec.sndBuf));
	writeBuf(txRec.sndBuf, strlen(txRec.sndBuf));
}

void XBee::eneterCommandMode() {
	writeStr("+++");
	HAL_Delay(1200);
	writeStr("+++");
}

void XBee::recTransmitStatus(const uint8_t *dt, int len) {
	OutStream *s = getOutStream();
	s->oMsgX(colGREEN, "TransmitStatus: len=%u", len);
}

void XBee::recModemStatus(uint8_t recSt) {
	OutStream *s = getOutStream();
	switch (recSt) {
	case 0x00:
		s->oMsgX(colGREEN, "ModemStatus:Hardware reset");
		break;
	default:
		s->oMsgX(colGREEN, "ModemStatus:Code=0x%02X", recSt);
		break;
	}
}

bool XBee::cmp2B(const void *x1, const void *x2) {
	const uint8_t *p1 = (const uint8_t*) x1;
	const uint8_t *p2 = (const uint8_t*) x2;
	return (p1[0] == p2[0]) && (p1[1] == p2[1]);
}

const char* XBee::getStatusTxt(uint8_t st) {
	switch (st) {
	case 0:
		return "OK";
	case 1:
		return "Error";
	case 2:
		return "Invalid cmd";
	case 3:
		return "Invalid param.";
	default:
		return "Unkn status";
	}
}
uint16_t XBee::getWord(const uint8_t *dt) {
	return dt[0] << 8 | dt[1];
}

//\0x00\0x13\0xA2\0x00A\0xC6\0x9D\0xC4\0xFF\0xFE\0xC2C.K_OFF
//0x00 0x13 0xA2 0x00 0x41 0xC6 0x9D 0xC4 0xFF 0xFE 0xC2
void XBee::recRemoteFrame(const uint8_t *dt, int len) {
	if (mDbgLevel >= 3) {
		getOutStream()->oBinBufX(colYELLOW, dt, len);
	}
	char buf[20];
	int n = len - 11;
	if (n > 20 - 1) {
		getOutStream()->oMsgX(colRED, "rec data too long, n=%u", n);
		n = 20 - 1;
	}
	memcpy(buf, &dt[11], n);
	buf[n] = 0;
	if (mDbgLevel >= 2) {
		getOutStream()->oMsgX(colYELLOW, "XBeeRec=[%s]", buf);
	}

	//jeśli manipulator jest ustawiony na remote
	if (globalVar.encoder.remote) {
		int channel = buf[0] - 'A' + 1;
		if (channel == cfg->dt.R.xbeeChannel) {
			bool fnd = false;
			uint16_t val = 0;

			if ((strcmp(&buf[1], ".D_OFF") == 0) || (strcmp(&buf[1], ".K_OFF") == 0)) {
				fnd = true;
				val = 0;
			} else {
				//E.KP_ON 300
				//A.KP_ON 300

				char buf2[10];
				memcpy(buf2, &buf[1], 7);
				buf2[7] = 0;
				if (strcmp(buf2, ".KP_ON ") == 0) {
					fnd = true;
					val = atoi(&buf[8]);
				}
			}
			if (fnd) {
				if (mDbgLevel >= 1) {
					getOutStream()->oMsgX(colYELLOW, "XBeeRec setLamp %.1f[%%]", val / 1.0);
				}
				globalVar.setRemoteVal(val);
			}
		}
	}

}

//+------+--------+--------+--------+-----------
//| CmIdx| ATCMD1 | ATCMD2 | STATUS | Data
//+------+--------+--------+--------+-----------

void XBee::recAtCommand(const uint8_t *dt, int len) {
	OutStream *s = getOutStream();
	uint8_t cmIdx = dt[0];
	const uint8_t *atCmd = &dt[1];
	uint8_t cmStatus = dt[3];
	const uint8_t *valPtr = &dt[4];
	int dtLen = len - 3 - 1;

	if (s->oOpen(colGREEN)) {
		s->oMsgNN("LocalAT:Idx=%d St=%s ", cmIdx, getStatusTxt(cmStatus));

		if (cmp2B(atCmd, "TP")) {
			char temp = (char) valPtr[1];
			s->oMsg("Temper=%d", temp);
		} else if (cmp2B(atCmd, "%V")) {
			uint16_t vol = getWord(valPtr);
			s->oMsg("Vol=%d[mV]", vol);

			//} else if (cmp2B(atCmd, "NI")) {
		} else if (cmp2B(atCmd, "VL")) {
			s->oMsgNN("VL=<");
			s->oBinBuf(valPtr, dtLen);
			s->oMsgNN(">\r\n");
		} else {
			char cmd[3];
			cmd[0] = atCmd[0];
			cmd[1] = atCmd[1];
			cmd[2] = 0;
			s->oMsgNN("Cmd=%s, lenDt=%d", cmd, dtLen);
			if (dtLen > 0) {
				s->oMsgNN(", buf:");
				s->oBinBufHex(&dt[4], dtLen);
			}
		}
		s->oMsgNN("\r\n");
		s->oClose();
	}
}

//+------+-------------+--------+-----------  ---+--------+
//|START |   length    | API Typ| Data           |CheckSum|
//+------+------+------+--------+-----------  ---+--------+
//| 0x7E | MSB  | LSB  | TYP    | Data           | byte   |
//+------+------+--------+------+-----------  ---+--------+

void XBee::tickReciveBuf() {
	if (rxRec.ptr > 0) {
		if (HAL_GetTick() - rxRec.tick > 50) {

			int idx = 0;
			while (idx < rxRec.ptr) {
				uint8_t *frame = &rxRec.buf[idx];
				if (frame[0] != 0x7E) {
					break;
				}
				uint16_t len_d = (frame[1] << 8) | (frame[2]);  // długość danych ramki
				if (idx + len_d + 3 + 1 > rxRec.ptr) {
					break;
				}
				bool q = checkSumFrame(frame, len_d);
				if (!q) {
					getOutStream()->oBinBufHexX(colRED, rxRec.buf, rxRec.ptr);
					getOutStream()->oMsgX(colRED, "SumErr");
					break;
				}

				if (mDbgLevel >= 4)
					getOutStream()->oBinBufHexX(colGREEN, frame, len_d + 4);
				switch (frame[3]) {
				case apiAT_CMD_RESP: //AT Command Response
					recAtCommand(&frame[4], len_d - 1);
					break;
				case apiMODEM_STATUS: //Modem Status
					recModemStatus(frame[4]);
					break;
				case apiTRANSM_STATUS: //Transmit Status
					recTransmitStatus(&frame[4], len_d - 1);
					break;
				case apiREC_PKT: //Receive Packet (AO=0)
					recRemoteFrame(&frame[4], len_d - 1);
					break;
				case apiREC_INDICATOR: //Explicit Rx Indicator (AO=1)
					break;
				case apiDATA_RX_INDICATOR: //I/O Data Sample RX Indicator
					break;
				case apiNODE_INDICATOR: //Node Identification Indicator (AO=0)
					break;
				case apiRMT_CMD_RESP: //Remote Command Response
					break;
				default:
					getOutStream()->oMsgX(colGREEN, "RecCmd=0x%02X len=%d", frame[3], len_d);
					break;
				}

				idx += len_d + 4;
			}
			rxRec.ptr = 0;
		}

	}

}

void XBee::tick() {
	tickReciveBuf();
}

void XBee::sendAtCmd(OutStream *strm, const char *cmd) {
	char cm[4];
	char typCm[4];
	char valCm[30];

	if (Token::get(&cmd, cm, sizeof(cm))) {
		char oBuf[40];
		oBuf[0] = cm[0];
		oBuf[1] = cm[1];
		int sLen = -1;

		if (Token::get(&cmd, typCm, sizeof(typCm))) {
			if (Token::get(&cmd, valCm, sizeof(valCm))) {
				switch (typCm[0]) {
				case 'B': {
					uint8_t v = atoi(valCm);
					oBuf[2] = v;
					sLen = 3;
				}
					break;
				case 'W': {
					uint16_t v = atoi(valCm);
					oBuf[2] = v >> 8;
					oBuf[3] = v & 0xff;
					sLen = 4;
				}
				case 'H': {
					uint16_t v = strtol(valCm, NULL, 16);
					oBuf[2] = v >> 8;
					oBuf[3] = v & 0xff;
					sLen = 4;
				}
				case 'X': {
					uint32_t v = strtol(valCm, NULL, 16);
					oBuf[2] = (v >> 24) & 0xff;
					oBuf[3] = (v >> 16) & 0xff;
					oBuf[4] = (v >> 8) & 0xff;
					oBuf[5] = v & 0xff;
					sLen = 6;
				}
					break;
				case 'S':
					strlcpy(&oBuf[2], valCm, sizeof(oBuf) - 2);
					sLen = strlen(oBuf);
					break;
				}

			}
		} else {
			sLen = 2;
		}
		if (sLen > 0) {
			uint8_t atidx;
			senApiCmd(oBuf, sLen, &atidx);
			strm->oMsgX(colYELLOW, "SendCmd [%s] idx=%u", cm, atidx);
			return;
		}

	}
	strm->oMsgX(colYELLOW, "Use: hexATcmd AT (cmd)");
	strm->oMsgX(colYELLOW, "     hexATcmd AT (cmd), typ (B|W|S|H|X), param_val");
	strm->oMsgX(colYELLOW, "     B-uint8, W-uint16, S-string, H-hex_uint16, X-hex_uint32");
}

void XBee::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("STATUS_LINE=%u", rdStatus());
		strm->oMsg("ATTN_LINE=%u", rdAttn());
		strm->oMsg("RSSI_LINE=%u", rdRssi());

		strm->oMsg("Debug=%d", mDbgLevel);
		strm->oMsg("HexMode=%u", mHexMode);
		strm->oMsg("RxRecCnt=%u", rxRec.globCnt);
		strm->oMsg("TxCptlCnt=%u", txRec.cpltCnt);

		strm->oClose();
	}

}

const ShellItem menuXBee[] = { //
		{ "dbg", "zmien poziom logów (0..4)" }, //
				{ "hex", "set HexMode" }, //
				{ "s", "pokaż status" }, //
				{ "rst", "Reset pulse" }, //
				{ "cm", "eneter command mode" }, //
				{ "wr", "test write" }, //
				{ "rr", "set reset line" }, //
				{ "brk", "send BRK while RESET" }, //
				{ "longbrk", "eneter comman mode - send LongBRK" }, //
				{ "hexATcmd", "send AT command in API1 Prams: AT (cmd), typ (B|W|S), param_val" }, //

				{ NULL, NULL } };

//ATBD - odczyt prędkości
//ATD0 4,AC - usawienia stanu niskiego na D0 (pin 33)
//ATD0 5,AC - usawienia stanu wysokiego na D0 (pin 33)

void XBee::shell(OutStream *strm, const char *cmd) {
	char tok[20];
	int idx = -1;
	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(menuXBee, tok);
	switch (idx) {
	case 0:  //dbg
		Token::getAsInt(&cmd, &mDbgLevel);
		break;
	case 1:  //hex
		Token::getAsBool(&cmd, &mHexMode);
		break;
	case 2:  //s
		showState(strm);
		break;
	case 3:  //rst
		strm->oMsgX(colWHITE, "reset");
		resetPulse();
		break;
	case 4:  //cm
		strm->oMsgX(colWHITE, "EnterCommandMode");
		eneterCommandMode();
		break;
	case 5: { //wr
		char cm[20];
		Token::trim(&cmd);
		strlcpy(cm, cmd, sizeof(cm));
		int m = strlen(cm);
		if (m < (int) sizeof(cm) - 2) {
			cm[m] = '\r';
			cm[m + 1] = 0;
			writeStr(cm);
			cm[m] = 0;
			strm->oMsgX(colYELLOW, "SendCmd [%s]", cm);
		}
	}
		break;
	case 6: {  //rr
		bool q;
		if (Token::getAsBool(&cmd, &q)) {
			strm->oMsgX(colWHITE, "Set reset : %u", q);
			setResetHd(q);
		}
	}
		break;
	case 7: //brk
		resetPulseToCmMode();
		strm->oMsgX(colWHITE, "Send BRK\r\n");
		break;
	case 8: //longbrk
		longBrkToCmMode();
		strm->oMsgX(colWHITE, "Send longBRK\r\n");
		break;
	case 9: //hexATcmd
		sendAtCmd(strm, cmd);
		break;

	default:
		showHelp(strm, "XBee Menu", menuXBee);
		break;
	};

}
