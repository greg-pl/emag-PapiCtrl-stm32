/*
 * utils.h
 *
 *  Created on: Dec 5, 2020
 *      Author: Grzegorz
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "_ansi.h"
#include "stdint.h"
#include "main.h"

#include "myDef.h"
#include "ErrorDef.h"


#define  HAL_CRC_ERR ((HAL_StatusTypeDef) 0x04)
#define  HAL_CMP_ERR ((HAL_StatusTypeDef) 0x05)
#define  HAL_NOWR_EXEC ((HAL_StatusTypeDef) 0x06)
#define  HAL_NO_VALUE  ((HAL_StatusTypeDef) 0x07)
#define  HAL_NO_SEMF   ((HAL_StatusTypeDef) 0x08)
#define  HAL_DATA_ERR  ((HAL_StatusTypeDef) 0x09)
#define  HAL_SUM_ERR  ((HAL_StatusTypeDef) 0x010)
#define  HAL_CFG_KOREKTED ((HAL_StatusTypeDef) 0x011)


#define USE_RTC 0

extern "C" const char* HAL_getErrStr(HAL_StatusTypeDef st);
extern "C" const char* ST3Str(ST3 val);
extern "C" const char* YN(bool q);
extern "C" const char* HL(bool q);
extern "C" const char* OnOff(bool q);
extern "C" const char* ErrOk(bool q);
extern "C" const char* OkErr(bool q);
extern "C" const char* FalseTrue(bool q);

extern "C" const char* getStatusStr(TStatus st);
extern "C" const char* getTmSrcName(uint8_t tmSrc);

extern "C" bool strbcmp(const char *buf, const char *wz);
extern "C" bool strbcmp2(const char *buf, const char *wz, const char **rest);
extern "C" bool loadSoftVer(VerInfo *ver, const char *mem);
extern "C" const char* binStr(char *buf, uint32_t v, int n);
extern "C" uint32_t swap32(uint32_t inp);
extern "C" void putWord(uint8_t *ptr, uint16_t w);
extern "C" uint32_t getDWord(uint8_t *ptr);
extern "C" float getFloat(uint8_t *ptr);


typedef struct {
	void *mem;
	int size;
} MemInfo;

class TimeTools {
public:
	enum {
		DT_TM_SIZE = 20, //
		DT_TM_ZZ_SIZE = 24, //
	};
	static bool CheckTime(const TDATE *tm);
	static bool CheckDate(const TDATE *tm);
	static bool CheckDtTm(const TDATE *tm);
	static const char* TimeStr(char *buf, const TDATE *tm);
	static const char* TimeStrZZ(char *buf, const TDATE *tm);
	static const char* DateStr(char *buf, const TDATE *tm);
	static const char* DtTmStr(char *buf, const TDATE *tm);
	static const char* DtTmStrK(char *buf, const TDATE *tm);
	static const char* DtTmStrZZ(char *buf, const TDATE *tm);
	static bool parseTime(const char **cmd, TDATE *Tm);
	static bool parseDate(const char **cmd, TDATE *Tm);
	static void copyDate(TDATE *dst, const TDATE *src);
	static void copyTime(TDATE *dst, const TDATE *src);
	static bool AddHour(TDATE *tm, int delHour);
	static const char* TimeLongStr(char *buf, int milisec);
	static uint16_t PackDate(const TDATE *unp);
	static void UnPackDate(TDATE *unp, uint16_t pack);
	static uint32_t PackTime(const TDATE *unp);
	static void UnPackTime(uint32_t pack, TDATE *unp);

};

#if (USE_RTC)

class Rtc {
private:
	static RTC_HandleTypeDef hrtc;
	static uint8_t getSetne(RTC_TimeTypeDef *sTime);
public:
	static HAL_StatusTypeDef mRtcStatus;
	static void Init();
	static bool ReadTime(TDATE *tm);
	static bool ReadOnlyTime(TDATE *tm);
	static bool SetDtTm(const TDATE *tm);
	static bool SetDate(const TDATE *tm);
	static bool SetTime(const TDATE *tm);
};

#endif

class GlobTime {
private:
	enum {
		USEK_PER_MIN = 1000 * 60, //
		USEK_PER_HOUR = 60 * USEK_PER_MIN, //
		USEK_PER_DAY = 24 * USEK_PER_HOUR, //
	};
	static uint32_t usek;
	static uint8_t day;
	static uint8_t month;
	static uint8_t year;

public:
	static void init();
	static void setTm(const TDATE *aTm);
	static void getTm(TDATE *aTm);
	static void incUSek();

};

class DtFilter {
private:
	float mFactor;
	float mState;
	float mEmpty;
	int mNanCnt;
public:
	void init(float afactor);
	void inp(float val);
	float get();
};

class DigiFiltrTm {
	uint32_t mFiltrTmUp;
	uint32_t mFiltrTmDn;
	uint32_t mTimer;
	uint32_t FLastTickCnt;

	bool mVal;
public:
	DigiFiltrTm(int aFiltrTm);
	DigiFiltrTm(int aFiltrTmUp, int aFiltrTmDn);
	void setTimes(int aFiltrTmUp, int aFiltrTmDn);
	void input(bool x);
	bool val() {
		return mVal;
	}
};

class TCrc {
public:
	static void Set(uint8_t *p, int cnt);
	static uint16_t Build(const uint8_t *p, int cnt);
	static uint16_t Proceed(uint16_t Crc, byte uint8_t);
	static bool Check(const uint8_t *p, int cnt);
};

#endif /* UTILS_H_ */
