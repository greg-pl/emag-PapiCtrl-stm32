/*
 * myDef.h
 *
 *  Created on: 25 lis 2019
 *      Author: Grzegorz
 */

#ifndef INC_MYDEF_H_
#define INC_MYDEF_H_

#include "stdint.h"
#include "ErrorDef.h"


#define PACKED   __attribute__((packed))

//typedef unsigned char bool;

typedef unsigned char byte;
typedef unsigned int dword;
typedef unsigned short word;
typedef char const  *CSTR;

#define SEC_LABEL    __attribute__ ((section (".label")))
#define SEC_NOINIT   __attribute__ ((section (".noinit")))
#define SEC_RAM_FUNC __attribute__ ((section (".ram_func")))

#define false 0
#define true 1

enum {
	tmSrcUNKNOWN = 0, tmSrcMANUAL, //wprowadzony ręcznie
	tmSrcNTP, //protokół NTP
	tmSrcGPS, //GPS
	tmFirmVer,
};

typedef struct {
	byte yr;
	byte mt;
	byte dy;
	byte hr;
	byte mn;
	byte sc;
} PACKED TTime;

typedef struct {
	word rev;
	word ver;
	byte hdw;
	TTime tm;
} TVersion;

typedef struct {
	uint8_t rk; //
	uint8_t ms; //
	uint8_t dz; //
	uint8_t gd; //
	uint8_t mn; //
	uint8_t sc; //
	uint8_t se; // setne części sekundy
	uint8_t timeSource; //
} TDATE;

typedef enum {
	posFREE = 0, //
	posGND, //
	posVCC, //
} ST3;

typedef struct {
	word ver;
	word rev;
	TDATE time;
} VerInfo;




typedef union{
	word w;
	struct{
		word bit0:1;
		word bit1:1;
		word bit2:1;
		word bit3:1;
		word bit4:1;
		word bit5:1;
		word bit6:1;
		word bit7:1;
		word bit8:1;
		word bit9:1;
		word bitA:1;
		word bitB:1;
		word bitC:1;
		word bitD:1;
		word bitE:1;
		word bitF:1;
	};
} wordBit;


#endif /* INC_MYDEF_H_ */
