/*
 * I2cDev.h
 *
 *  Created on: 30 gru 2020
 *      Author: Grzegorz
 */

#ifndef I2CDEV_H_
#define I2CDEV_H_

#include "stdint.h"

#include <main.h>
#include "IOStream.h"
#include "Shellitem.h"

#define I2C_SERV_SHT35   0
#define I2C_SERV_BMP338  0
#define I2C_SERV_TIDS    1
#define I2C_SERV_PAPI_MAN 1  // manpulator PAPI
#define I2C_SERV_VEML6030 1  // VISHAY: VEML6030



class I2cBus;

class I2c1Dev {
	friend class I2cBus;
protected:
	uint8_t mDevAdr;
	int mMenuOffset; // offset w menu zbiorczym
	bool mDevExist;
	I2cBus *iicBus;

	void showDevExist(OutStream *strm);
	virtual void tick() {
	}
	virtual void init(bool zeroVar) {
	}
	virtual const ShellItem* getShellMenu() {
		return NULL;
	}
protected:
	HAL_StatusTypeDef checkDev();


	HAL_StatusTypeDef reciveBytes(uint8_t *data, uint8_t length);

	HAL_StatusTypeDef readByte(uint8_t regAddr, uint8_t *data);
	HAL_StatusTypeDef readWord(uint8_t regAddr, uint16_t *data);
	HAL_StatusTypeDef readBytes(uint8_t regAddr, uint8_t *data, uint8_t length);
	HAL_StatusTypeDef readBytes16B(uint16_t regAddr, uint8_t *data, uint16_t length);

	HAL_StatusTypeDef writeByte(uint8_t regAddr, uint8_t data);
	HAL_StatusTypeDef writeWord(uint8_t regAddr, uint16_t data);
	HAL_StatusTypeDef writeBytes(uint8_t regAddr, const uint8_t *data, uint8_t length);
	HAL_StatusTypeDef writeBytes16B(uint16_t regAddr, const uint8_t *data, uint16_t length);
	HAL_StatusTypeDef masterTransmit(uint8_t *pData, uint16_t length);


public:

	I2c1Dev(I2cBus *bus, uint8_t devAdr);

	virtual void showState(OutStream *strm)=0;
	virtual void showMeas(OutStream *strm) {
	}
	virtual void execFun(OutStream *strm, int funNr) {
	}
	uint8_t getAdr() {
		return mDevAdr;
	}
	bool isDevExist() {
		return mDevExist;
	}
	virtual bool isError()=0;
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd) {
		return false;
	}
	virtual bool isRdAuto() {
		return false;
	}

};

#if I2C_SERV_SHT35

class SHT35DevPub: public I2c1Dev {
public:
	bool mReadAuto; // czy czujnik ma być czytany cyklicznie
	uint32_t serialNr;
	HAL_StatusTypeDef mMeasStart;
	virtual HAL_StatusTypeDef getData(float *temperature, float *humidity)=0;
	SHT35DevPub(I2cBus *bus, uint8_t devAdr);
	static SHT35DevPub* createDev(I2cBus *bus, uint8_t devAdr);

};

#endif

#if I2C_SERV_BMP338

class Bmp338DevPub: public I2c1Dev {
public:
	bool chipIdOk;
	bool coefOk;
	virtual HAL_StatusTypeDef getData(float *temperature, float *pressure)=0;
	Bmp338DevPub(I2cBus *bus, uint8_t devAdr);
	static Bmp338DevPub* createDev(I2cBus *bus, uint8_t devAdr);
};

#endif


#if I2C_SERV_TIDS

//WURTH  WSEN-TIDS, 2521020222501
class TidsDevPub: public I2c1Dev {
public:
	bool chipIdOk;
	bool mReadAuto; // czy czujnik ma być czytany cyklicznie
	static bool mManyDev; // czy jest więcej niż jeden czujnik
	virtual HAL_StatusTypeDef getData(float *temperature) {
		return HAL_ERROR;
	}
	TidsDevPub(I2cBus *bus, uint8_t devAdr);
	static TidsDevPub* createDev(I2cBus *bus, uint8_t devAdr);
};

#endif



#if I2C_SERV_PAPI_MAN

class PapiManipDevPub: public I2c1Dev {
public:
	enum {
		swVISUAL = 0x01,
		swPHOTO_ACTIV = 0x02,
		swNIGHT_LEV = 0x04,
		swTILT_ACTIV = 0x08,
		swHEATER = 0x10,
	};


	bool chipIdOk;
	bool mReadAuto; // czy czujnik ma być czytany cyklicznie
	virtual HAL_StatusTypeDef getData(uint8_t *encoder, uint8_t *dipSwitch) {
		return HAL_ERROR;
	}

	PapiManipDevPub(I2cBus *bus, uint8_t devAdr);
	static PapiManipDevPub* createDev(I2cBus *bus, uint8_t devAdr);
};

#endif


#if I2C_SERV_VEML6030

class Veml6030DevPub: public I2c1Dev {
public:
	bool chipIdOk;
	bool mReadAuto; // czy czujnik ma być czytany cyklicznie
	virtual HAL_StatusTypeDef getData(float *light) {
		return HAL_ERROR;
	}

	Veml6030DevPub(I2cBus *bus, uint8_t devAdr);
	static Veml6030DevPub* createDev(I2cBus *bus, uint8_t devAdr);
};

#endif



class I2cBus {
	friend class I2c1Dev;
private:
	enum {
		MENU_SIZE = 30,
	};
	int mBusNr;
	ShellItem mMenu[MENU_SIZE];
	struct{
		uint32_t lastWorkTick;
		uint32_t busRestartTick;
		int RestartCnt;
	}busState;
protected:
	enum {
		MAX_DEV_CNT = 4,
	};
	I2C_HandleTypeDef hi2c;
	int mDevCnt;
	I2c1Dev *devTab[MAX_DEV_CNT];
	int mBusRestartCnt;

	void swap(uint16_t *p);
	uint16_t swapD(uint16_t d);
	void putSwap(uint8_t *p, uint16_t d);

	HAL_StatusTypeDef checkDev(uint8_t dev_addr);

	HAL_StatusTypeDef reciveBytes(uint8_t dev_addr, uint8_t *data, uint8_t len);

	HAL_StatusTypeDef readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
	HAL_StatusTypeDef readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data);
	HAL_StatusTypeDef readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t length);
	HAL_StatusTypeDef readBytes16B(uint8_t devAddr, uint16_t regAddr, uint8_t *data, uint16_t length);

	HAL_StatusTypeDef writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
	HAL_StatusTypeDef writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
	HAL_StatusTypeDef writeBytes(uint8_t devAddr, uint8_t regAddr, const uint8_t *data, uint8_t length);
	HAL_StatusTypeDef writeBytes16B(uint8_t devAddr, uint16_t regAddr, const uint8_t *data, uint16_t length);

	HAL_StatusTypeDef masterTransmit(uint8_t dev_addr, uint8_t *pData, uint16_t length);

private:
	void ScanBus(OutStream *strm);
	void showState(OutStream *strm);
	void showMeas(OutStream *strm);
	void ShowBusRegisters(OutStream *strm);

	void execFun(OutStream *strm, int idx);
	HAL_StatusTypeDef InitHd();
	void setAsGpio();
	void setGpioSDA(GPIO_PinState PinState);
	void setGpioSCL(GPIO_PinState PinState);
	bool getGpioSDA();
	bool getGpioSCL();
	void gpioSCLWave();
	bool rdBusyFlag();

	bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	int addToMenu(const ShellItem *devMenu);
	bool isRdAuto();

public:
	enum {
		iicBUS1 = 0, //
		iicBUS2 = 1, //
	};
	I2cBus(int busNr);
	HAL_StatusTypeDef BusRestart();
	void BusUnlock();
	void BusUnlock2();

	void shell(OutStream *strm, const char *cmd);
	void tick();
	int getBusRestartCnt() {
		return mBusRestartCnt;
	}
	bool isError();
	bool addDev(I2c1Dev *dev);
};

#endif /* I2CDEV_H_ */
