/*
 * shell.h
 *
 *  Created on: 21 lip 2021
 *      Author: Grzegorz
 */

#ifndef SRC_SHELL_H_
#define SRC_SHELL_H_

#include "stdint.h"

#include "RxTxBuf.h"
#include "EscTerminal.h"
#include "IOStream.h"
#include "uart.h"



class Shell: public OutStream, public TUart {
private:
	RxTxBuf *rxBuf;
	RxTxBuf *txBuf;
	struct {
		char rxChar;
		HAL_StatusTypeDef reciveSt;
		int ErrCnt;
	} rxRec;

	EscTerminal *term;
	bool flgSendAny;

	void trySend();
	void StartRecive();


	void execAltChar(char altChar);
	void execFunKey(FunKey funKey);
	void execCmdLine(const char *cmd);
	void showHdwState(OutStream *strm);
	void showState();
	void setLeds(const char *cmd);


protected:
	virtual void TxCpltCallback();
	virtual void RxCpltCallback();
	virtual void ErrorCallback();

protected:
	virtual void putOut(const void *mem, int len);
public:
	Shell();
	HAL_StatusTypeDef Init(int BaudRate);

	static Shell *Me;
	void addRecData(uint8_t *Buf, uint32_t Len);
	void tick();

};

#endif /* SRC_SHELL_H_ */
