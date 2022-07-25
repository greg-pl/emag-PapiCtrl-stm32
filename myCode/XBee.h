/*
 * XBee.h
 *
 *  Created on: 20 mar 2022
 *      Author: Grzegorz
 */

#ifndef XBEE_H_
#define XBEE_H_

#include <uart.h>

class XBee: public TUart {
private:
	enum {
		FRAME_LEN = 240,
	};

	int mDbgLevel;
	bool mHexMode;
	uint8_t mAtCmdIdx;

	struct {
		uint8_t rxData;
		int globCnt;
		int ptr;
		uint8_t buf[FRAME_LEN + 1];
		uint32_t tick;
	} rxRec;

	struct {
		int cpltCnt;
		char sndBuf[FRAME_LEN];
	} txRec;


	void setTxAsUART();
	void setTxAsGPIO();

	void setResetHd(bool res);
	void setSleepReqHd(bool sleep);

	void resetPulse();
	void resetPulseToCmMode();
	void longBrkToCmMode();

	bool rdStatus();
	bool rdAttn();
	bool rdRssi();

	void showState(OutStream *strm);
	void writeStr(const char *dt);
	void eneterCommandMode();
	void tickReciveBuf();
	void addEnvelope(uint8_t cmdID, const char *inp, int len);
	void senApiCmd(const char *inp, int len, uint8_t *cmdIdx);
	bool checkSumFrame(const uint8_t *inp, int len);
	void recModemStatus(uint8_t recSt);
	void recTransmitStatus(const uint8_t *dt, int len);
	void recAtCommand(const uint8_t *dt, int len);
	void recRemoteFrame(const uint8_t *dt, int len);
	bool cmp2B(const void *x1,const void *x2);
	const char* getStatusTxt(uint8_t st);
	uint16_t getWord(const uint8_t *dt);
	void sendAtCmd(OutStream *strm, const char *cmd);



protected:
	virtual void TxCpltCallback();
	virtual void RxCpltCallback();

public:
	XBee(int PortNr, int Priority);
	HAL_StatusTypeDef Init(int BaudRate);
	void tick();
	void shell(OutStream *strm, const char *cmd);

};

#endif /* XBEE_H_ */
