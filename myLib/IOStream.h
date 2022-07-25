/*
 * MsgStream.h
 *
 *  Created on: Dec 11, 2020
 *      Author: Grzegorz
 */

#ifndef MSGSTREAM_H_
#define MSGSTREAM_H_

#include "stdarg.h"
#include "EscTerminal.h"

enum {
	MSG_ERR = 1, //
	MSG_WARN = 2, //
	MSG_INFO = 3, //
	MSG_DATA = 4, //
};

class OutStream: public OutHdStream {
private:
	char outBuf[200];  //dostęp do bufora tylko po otwarciu semafora

protected:
	//virtual void putOut(const void *mem, int len);
public:
	OutStream();
	virtual void closeTerm() {
	}
	void oMsgX(TermColor color, const char *pFormat, ...);
	void oBufX(TermColor color, const void *buf, int len);
	void oWrX(TermColor color, const char *buf);
	void oBinBufX(TermColor color, const void *buf, int len);
	void oBinBufHex(const void *buf, int len);
	void oBinBufHexX(TermColor color, const void *buf, int len);

	void oFormat(TermColor color, const char *pFormat, va_list ap);

	bool oOpen(TermColor color);
	void oClose();
	void oMsgNN(const char *pFormat, ...);
	void oMsg(const char *pFormat, ...);
	void oWr(const char *txt);
	void oBuf(const void *mem, int len);
	void oSetColor(TermColor color);
	void oBinBuf(const void *buf, int len);

};

extern "C" OutStream* getOutStream();

class SignaledClass {
public:
	virtual void setSignal()=0;
};

#endif /* MSGSTREAM_H_ */
