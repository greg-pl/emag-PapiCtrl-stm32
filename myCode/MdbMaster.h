/*
 * MdbMaster.h
 *
 *  Created on: 14 lut 2022
 *      Author: Grzegorz
 */

#ifndef MDBMASTER_H_
#define MDBMASTER_H_

#include "stdint.h"
#include "IOStream.h"
#include "uart.h"
#include "Shellitem.h"

typedef enum {
	reqEMPTY = 0, //
	reqCONSOLA, //
	reqSYS,
} ReqSrc;

class MdbUart: public TUart {
public:
	enum {
		FRAME_LEN = 240,
	};
private:
	void setTxEn(bool txON);
	struct {
		uint8_t rxData;
		int globCnt;
		int ptr;
		uint8_t buf[FRAME_LEN + 1];
		uint32_t tick;
	} rxRec;

protected:
	virtual void TxCpltCallback();
	virtual void RxCpltCallback();
public:
	MdbUart(int PortNr, int Priority);
	HAL_StatusTypeDef Init(int BaudRate, int parity);
	void writeBuf(const void *buf, int len);
	uint32_t getLastRxCharTick() {
		return rxRec.tick;
	}
	int getRxCharCnt() {
		return rxRec.ptr;
	}
	int getRxGlobCnt() {
		return rxRec.globCnt;
	}
	const uint8_t* getRxBuf() {
		return rxRec.buf;
	}
	void clearRxBuf();
};

typedef struct {
	bool err;
	bool info;
	bool dat;
} MsgV;

class MdbMaster {
private:
	enum {
		MAX_VAL_CNT = 10, // maksymalna ilość rejestrów dla funcji 16
		MAX_MDB_REPL_TM = 1000, // maksymalny czas na odpowiedz -> 1[sek]

	};

	MdbUart *mUart;
	uint8_t sendBuf[248];

protected:
	int mDbgLevel;
	void showState(OutStream *strm);
	void sendMdbFun(ReqSrc reqSrc, uint8_t *buf, int len);
	void SetWord(uint8_t *p, uint16_t w);
	uint16_t GetWord(const uint8_t *p);
	float GetIntAsFloat(const uint8_t *p);

	float GetFloat(const uint8_t *p);

	void sendMdbFun3(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt);
	void sendMdbFun4(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt);
	void sendMdbFun6(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regVal);
	void sendMdbFun16(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt, uint16_t *regTab);

	void sendConsolaReq();
	void proceedRecFrame();
	void buidMsgRec(MsgV *m);

protected:
	int mMdbNr;

	struct {
		uint8_t devNr;
		uint8_t fun;
		uint16_t regAdr;
		uint16_t regCnt;
		uint16_t regVal[MAX_VAL_CNT];
	} reqConsola;

	struct {
		struct {
			ReqSrc currReq; // przetwarzane żadanie modbus
			uint8_t devNr;
			uint8_t code;
			uint16_t regAdr;
			uint16_t regCnt;
			uint16_t regVal;
			uint32_t tick;
		} sent;
		uint32_t proceedFrameTick;
		int timeOutCnt;
		int crcErrCnt;

	} state;

	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
	}
	virtual void doOnTimeOut() {
	}
	struct {
		ShellItem *tab;
		int baseCnt;
	} menu;
	virtual const ShellItem* getMenu();
	virtual const char* getMenuName();
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	void buildMenu(const ShellItem *toAddMenu);
public:
	MdbMaster(int MdbNr, int PortNr, int Priority);
	HAL_StatusTypeDef Init(int BaudRate, int parity);

	void shell(OutStream *strm, const char *cmd);
	void tick();
};

class MdbPAPI: public MdbMaster {
private:
	enum {
		TM_AUTO_RD = 2000, // czas automatycznego odczytu czujnika hałasu
		MAX_TIME_REPL = 1500, // maksymalny czas odpowiedzi
		regPWM1 = 1, //poziom wysterowania kanału 1 (WHITE)
		regPWM2 = 2, //poziom wysterowania kanału 2 (IR1)
		regPWM3 = 3, //poziom wysterowania kanału 3 (RED)
		regPWM4 = 4, //poziom wysterowania kanału 4 (IR2)

		regLEVEL_CPX = 14, // ustawienia 4 kanałów
		regCTRL_EX = 15, // DipSwitch'e

		regDEV_STATUS2 = 9, // numer rejestru do odczytu statusu2 w S847
		regDEV_STATUS = 10, // numer rejestru do odczytu statusu w S847
		regSET_TILT = 16, // numer rejestru do ustawienia położenia bazowego
		SET_TILT_VAL = 0xAA11, //
	};

	struct {
		uint16_t powerDt;
		uint32_t rdPowerTick;
		uint32_t wrPowerTick;
	} papiState;

	struct {
		struct {
			bool startNow;
			bool setBasePosition;
			bool sendDipSwitchPos;
		} triger;
		int phase;
		uint32_t tick;
		uint32_t redTick;
		int redCnt; // licznik odpowiedzi o pomiary
		int reqCnt; // licznik zapytań o pomiary
		char statusTxt[20];
		uint32_t heaterOrderLastSendTick;
	} autoRd;

protected:
	virtual void doOnTimeOut();
	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt);
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	virtual const ShellItem* getMenu();
	virtual const char* getMenuName();
public:
	MdbPAPI(int PortNr, int Priority);
	void tick();
	void sendLightState();
	void sendSetBasePosition();
	void sendManipChg();

};

#endif /* MDBMASTER_H_ */
