/*
 * I2cDev.cpp
 *
 *  Created on: 30 gru 2020
 *      Author: Grzegorz
 */

#include "string.h"
#include "stdio.h"
#include "math.h"

#include <I2cDev.h>
#include <uMain.h>
#include <Utils.h>
#include <cpx.h>
#include <Token.h>
#include <Utils.h>
#include <Cfg.h>
#include <Hdw.h>

extern Cfg *cfg;

#define TIME_DT_RD      2000
#define TIME_DT_VALID   5000


#define FILTR_FACTOR  0.8

const ShellItem menuI2c[] = { //
		{ "s", "stan" }, //
				{ "m", "pomiary" }, //
				{ "scan", "przeszukanie magistrali" }, //
				{ "restart", "restart magistrali" }, //
				{ "reg", "show iic registers" }, //
				{ "busUnlock", "odblokowanie magistrali i2c" }, //
				{ "rdGpio", "czytaj linie SDA,SCL jako GPIO" }, //
				{ "sclWave", "wygenerowanie fali na lnii SCL" }, //

				{ NULL, NULL } };

I2cBus::I2cBus(int busNr) {
	mBusNr = busNr;
	mDevCnt = 0;
	mBusRestartCnt = 0;
	memset(&busState, 0, sizeof(busState));
	for (int i = 0; i < MAX_DEV_CNT; i++) {
		devTab[i] = NULL;
	}
	InitHd();
	memset(mMenu, 0, sizeof(mMenu));
	copyMenu(mMenu, MENU_SIZE, menuI2c);
}

HAL_StatusTypeDef I2cBus::InitHd() {
	memset(&hi2c, 0, sizeof(hi2c));
	switch (mBusNr) {
	case iicBUS1:
		hi2c.Instance = I2C1;
		break;
	case iicBUS2:
		hi2c.Instance = I2C2;
		break;
	default:
		return HAL_ERROR;
	}
	hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.ClockSpeed = 40000; //40kHz
	hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c.Init.OwnAddress1 = 0;
	hi2c.Init.OwnAddress2 = 0;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	return HAL_I2C_Init(&hi2c);
}

void I2cBus::BusUnlock() {
	SET_BIT(hi2c.Instance->CR1, I2C_CR1_SWRST);
	HAL_Delay(2);
	CLEAR_BIT(hi2c.Instance->CR1, I2C_CR1_SWRST);
}
void I2cBus::BusUnlock2() {
	gpioSCLWave();
	HAL_Delay(50);
	InitHd();
}

void I2cBus::setAsGpio() {
	GPIO_InitTypeDef R;
	memset(&R, 0, sizeof(R));
	R.Mode = GPIO_MODE_OUTPUT_OD;
	R.Speed = GPIO_SPEED_FREQ_HIGH;
	R.Pull = GPIO_PULLUP;

	switch (mBusNr) {
	case iicBUS1:
		//PB6     ------> I2C1_SCL
		//PB7     ------> I2C1_SDA
		R.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		HAL_GPIO_Init(GPIOB, &R);
		break;
	case iicBUS2:
		R.Pin = GPIO_PIN_11 | GPIO_PIN_10;
		HAL_GPIO_Init(GPIOB, &R);

		break;
	default:
		return;
	}

	setGpioSDA(GPIO_PIN_SET);
	setGpioSCL(GPIO_PIN_SET);
}

void I2cBus::setGpioSDA(GPIO_PinState PinState) {
	switch (mBusNr) {
	case iicBUS1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, PinState);
		break;
	case iicBUS2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, PinState);
		break;
	}
}
void I2cBus::setGpioSCL(GPIO_PinState PinState) {
	switch (mBusNr) {
	case iicBUS1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, PinState);
		break;
	case iicBUS2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, PinState);
		break;
	}
}

bool I2cBus::getGpioSDA() {
	switch (mBusNr) {
	case iicBUS1:
		return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET);
	case iicBUS2:
		return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET);
	}
	return false;
}

bool I2cBus::getGpioSCL() {
	switch (mBusNr) {
	case iicBUS1:
		return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET);
	case iicBUS2:
		return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET);
	}
	return false;
}

void I2cBus::gpioSCLWave() {
	setAsGpio();
	setGpioSDA(GPIO_PIN_SET);
	for (int i = 0; i < 16; i++) {
		setGpioSCL(GPIO_PIN_RESET);
		HAL_Delay(2);
		setGpioSCL(GPIO_PIN_SET);
		HAL_Delay(2);
	}
}

HAL_StatusTypeDef I2cBus::BusRestart() {
	HAL_I2C_DeInit(&hi2c);
	gpioSCLWave();
	HAL_StatusTypeDef st = InitHd();
	if (st == HAL_OK) {
		for (int i = 0; i < mDevCnt; i++) {
			devTab[i]->init(false);
		}
	}
	mBusRestartCnt++;
	return st;
}

void I2cBus::ShowBusRegisters(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("CR1:0x%08X", hi2c.Instance->CR1);
		strm->oMsg("CR2:0x%08X", hi2c.Instance->CR2);
		strm->oMsg("SR1:0x%08X", hi2c.Instance->SR1);
		strm->oMsg("SR2:0x%08X", hi2c.Instance->SR2);
		strm->oMsg("CCR  :0x%08X", hi2c.Instance->CCR);
		strm->oMsg("TRISE:0x%08X", hi2c.Instance->TRISE);
		strm->oClose();
	}

}

void I2cBus::swap(uint16_t *p) {
	uint8_t *pb = (uint8_t*) p;
	uint8_t b1 = pb[0];
	uint8_t b2 = pb[1];
	pb[0] = b2;
	pb[1] = b1;
}

uint16_t I2cBus::swapD(uint16_t d) {
	return (d >> 8) | (d << 8);
}

void I2cBus::putSwap(uint8_t *p, uint16_t d) {
	p[0] = d >> 8;
	p[1] = d & 0xff;
}

HAL_StatusTypeDef I2cBus::reciveBytes(uint8_t dev_addr, uint8_t *data, uint8_t len) {
	busState.lastWorkTick = HAL_GetTick();
	return HAL_I2C_Master_Receive(&hi2c, dev_addr, data, len, 100);
}

HAL_StatusTypeDef I2cBus::readBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
	busState.lastWorkTick = HAL_GetTick();
	return HAL_I2C_Mem_Read(&hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

HAL_StatusTypeDef I2cBus::readBytes16B(uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t len) {
	busState.lastWorkTick = HAL_GetTick();
	return HAL_I2C_Mem_Read(&hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_16BIT, data, len, 100);
}

HAL_StatusTypeDef I2cBus::readByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
	return readBytes(dev_addr, reg_addr, data, 1);
}

HAL_StatusTypeDef I2cBus::readWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t *data) {
	uint16_t tmp;
	HAL_StatusTypeDef st = readBytes(dev_addr, reg_addr, (uint8_t*) &tmp, 2);
	*data = swapD(tmp);
	return st;
}

HAL_StatusTypeDef I2cBus::writeBytes(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint8_t len) {
	busState.lastWorkTick = HAL_GetTick();
	return HAL_I2C_Mem_Write(&hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*) (int) data, len, 100);
}
HAL_StatusTypeDef I2cBus::writeBytes16B(uint8_t dev_addr, uint16_t reg_addr, const uint8_t *data, uint16_t len) {
	busState.lastWorkTick = HAL_GetTick();
	return HAL_I2C_Mem_Write(&hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*) (int) data, len, 100);
}

HAL_StatusTypeDef I2cBus::masterTransmit(uint8_t dev_addr, uint8_t *pData, uint16_t length) {
	busState.lastWorkTick = HAL_GetTick();
	return HAL_I2C_Master_Transmit(&hi2c, dev_addr, pData, length, 100);
}

HAL_StatusTypeDef I2cBus::writeByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
	return writeBytes(dev_addr, reg_addr, &data, 1);
}

HAL_StatusTypeDef I2cBus::writeWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t data) {
	data = swapD(data);
	return writeBytes(dev_addr, reg_addr, (const uint8_t*) &data, 2);
}

bool I2cBus::addDev(I2c1Dev *dev) {
	if (mDevCnt < MAX_DEV_CNT) {
		devTab[mDevCnt] = dev;
		mDevCnt++;
		dev->mMenuOffset = addToMenu(dev->getShellMenu());
		return true;
	}
	return false;
}

int I2cBus::addToMenu(const ShellItem *devMenu) {
	return copyMenu(mMenu, MENU_SIZE, devMenu);
}

HAL_StatusTypeDef I2cBus::checkDev(uint8_t dev_addr) {
	busState.lastWorkTick = HAL_GetTick();
	return HAL_I2C_IsDeviceReady(&hi2c, dev_addr, 3, 100);
}

void I2cBus::ScanBus(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		int devCnt = 0;
		for (int i = 0; i < 128; i++) {
			uint16_t dev_addr = 2 * i;
			busState.lastWorkTick = HAL_GetTick();
			if (HAL_I2C_IsDeviceReady(&hi2c, dev_addr, 2, 50) == HAL_OK) {
				strm->oMsg("Found adr=0x%02X", dev_addr);
				devCnt++;
			}
		}
		strm->oMsg("Found %u devices", devCnt);
		strm->oClose();
	}
}

void I2cBus::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("BusRestartCnt=%u", mBusRestartCnt);

		for (int i = 0; i < mDevCnt; i++) {
			devTab[i]->showState(strm);
		}
		strm->oClose();
	}
}
void I2cBus::showMeas(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		for (int i = 0; i < mDevCnt; i++) {
			devTab[i]->showMeas(strm);
		}
		strm->oClose();
	}
}

bool I2cBus::rdBusyFlag() {
	return ((hi2c.Instance->SR2 & I2C_SR2_BUSY) != 0);
}

bool I2cBus::isRdAuto() {
	bool rd = false;
	for (int i = 0; i < mDevCnt; i++) {
		rd |= devTab[i]->isRdAuto();
	}
	return rd;
}

void I2cBus::tick() {
	if (HAL_GetTick() - busState.lastWorkTick > 5) {

		if (rdBusyFlag()) {
			if (isRdAuto()) {
				//zablokowanie magistrali- próba odblokowania
				if (HAL_GetTick() - busState.busRestartTick > 800) {
					busState.busRestartTick = HAL_GetTick();
					busState.RestartCnt++;
					setAsGpio();
					bool sdaBf = getGpioSDA();
					gpioSCLWave();
					bool sdaAf = getGpioSDA();
					bool busyAf = rdBusyFlag();
					BusRestart();

					bool q = (busState.RestartCnt < 5) || ((busState.RestartCnt % 5) == 0);
					if (q) {
						getOutStream()->oMsgX(colRED, "I2CBus RESTART: sdaBf=%u, sdaAf=%u, busyAf=%u", sdaBf, sdaAf, busyAf);
					}
				}
			}

		} else {
			busState.RestartCnt = 0;
			for (int i = 0; i < mDevCnt; i++) {
				devTab[i]->tick();
			}
		}
	}
}

bool I2cBus::isError() {
	bool q = 0;
	for (int i = 0; i < mDevCnt; i++) {
		q |= devTab[i]->isError();
	}
	return q;
}

void I2cBus::execFun(OutStream *strm, int idx) {
	if (strm->oOpen(colWHITE)) {
		for (int i = 0; i < mDevCnt; i++) {
			devTab[i]->execFun(strm, idx);
		}
		strm->oClose();
	}
}

void I2cBus::shell(OutStream *strm, const char *cmd) {

	char tok[20];
	int idx = -1;

	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(mMenu, tok);

	if (!execMenuItem(strm, idx, cmd)) {
		bool fnd = false;
		for (int i = 0; i < mDevCnt; i++) {
			fnd = devTab[i]->execMenuItem(strm, idx, cmd);
			if (fnd)
				break;
		}
		if (!fnd) {
			char mnName[20];
			snprintf(mnName, sizeof(mnName), "I2C_%u Menu", mBusNr);
			showHelp(strm, mnName, mMenu);
		}
	}
}

bool I2cBus::execMenuItem(OutStream *strm, int idx, const char *cmd) {
	switch (idx) {
	case 0: //s
		showState(strm);
		break;
	case 1: //m
		showMeas(strm);
		break;
	case 2: //scan
		ScanBus(strm);
		break;
	case 3: //restart
		BusRestart();
		break;
	case 4: //reg
		ShowBusRegisters(strm);
		break;
	case 5: //busUnlock
		BusUnlock();
		strm->oMsgX(colWHITE, "Ok. Wykonaj restart i2c.");
		break;
	case 6: //rdGpio
		if (strm->oOpen(colWHITE)) {
			setAsGpio();
			strm->oMsg("SDA:%u", getGpioSDA());
			strm->oMsg("SCL:%u", getGpioSCL());
			strm->oClose();
		}
		break;
	case 7: //sclWave
		gpioSCLWave();
		strm->oMsgX(colWHITE, "SCLWave. Wykonaj restart i2c.");
		break;
	default:
		return false;
	}
	return true;
}

//-------------------------------------------------------------------------------------------------------------------------
// I2c1Dev
//-------------------------------------------------------------------------------------------------------------------------
I2c1Dev::I2c1Dev(I2cBus *bus, uint8_t devAdr) {
	iicBus = bus;
	mDevAdr = devAdr;
	mMenuOffset = 0;
}

HAL_StatusTypeDef I2c1Dev::checkDev() {
	return iicBus->checkDev(mDevAdr);
}

HAL_StatusTypeDef I2c1Dev::reciveBytes(uint8_t *data, uint8_t length) {
	return iicBus->reciveBytes(mDevAdr, data, length);
}

HAL_StatusTypeDef I2c1Dev::readByte(uint8_t regAddr, uint8_t *data) {
	return iicBus->readByte(mDevAdr, regAddr, data);
}

HAL_StatusTypeDef I2c1Dev::readWord(uint8_t regAddr, uint16_t *data) {
	return iicBus->readWord(mDevAdr, regAddr, data);
}
HAL_StatusTypeDef I2c1Dev::readBytes(uint8_t regAddr, uint8_t *data, uint8_t length) {
	return iicBus->readBytes(mDevAdr, regAddr, data, length);
}
HAL_StatusTypeDef I2c1Dev::readBytes16B(uint16_t regAddr, uint8_t *data, uint16_t length) {
	return iicBus->readBytes16B(mDevAdr, regAddr, data, length);
}

HAL_StatusTypeDef I2c1Dev::writeByte(uint8_t regAddr, uint8_t data) {
	return iicBus->writeByte(mDevAdr, regAddr, data);
}
HAL_StatusTypeDef I2c1Dev::writeWord(uint8_t regAddr, uint16_t data) {
	return iicBus->writeWord(mDevAdr, regAddr, data);
}
HAL_StatusTypeDef I2c1Dev::writeBytes(uint8_t regAddr, const uint8_t *data, uint8_t length) {
	return iicBus->writeBytes(mDevAdr, regAddr, data, length);
}
HAL_StatusTypeDef I2c1Dev::writeBytes16B(uint16_t regAddr, const uint8_t *data, uint16_t length) {
	return iicBus->writeBytes16B(mDevAdr, regAddr, data, length);
}

HAL_StatusTypeDef I2c1Dev::masterTransmit(uint8_t *pData, uint16_t length) {
	return iicBus->masterTransmit(mDevAdr, pData, length);
}

void I2c1Dev::showDevExist(OutStream *strm) {
	HAL_StatusTypeDef st = checkDev();
	strm->oMsg("DevExist=%s", HAL_getErrStr(st));
}

//-------------------------------------------------------------------------------------------------------------------------
// SHT35Dev
//-------------------------------------------------------------------------------------------------------------------------
#if I2C_SERV_SHT35

typedef enum {
	CMD_READ_SERIALNBR = 0x3780, // read serial number
	CMD_READ_STATUS = 0xF32D, // read status register
	CMD_CLEAR_STATUS = 0x3041, // clear status register
	CMD_HEATER_ENABLE = 0x306D, // enabled heater
	CMD_HEATER_DISABLE = 0x3066, // disable heater
	CMD_SOFT_RESET = 0x30A2, // soft reset
	CMD_BREAK = 0x3093, //break measure
	CMD_MEAS_CLOCKSTR_H = 0x2C06, // measurement: clock stretching, high repeatability
	CMD_MEAS_CLOCKSTR_M = 0x2C0D, // measurement: clock stretching, medium repeatability
	CMD_MEAS_CLOCKSTR_L = 0x2C10, // measurement: clock stretching, low repeatability
	CMD_MEAS_POLLING_H = 0x2400, // measurement: polling, high repeatability
	CMD_MEAS_POLLING_M = 0x240B, // measurement: polling, medium repeatability
	CMD_MEAS_POLLING_L = 0x2416, // measurement: polling, low repeatability
	CMD_MEAS_PERI_05_H = 0x2032, // measurement: periodic 0.5 mps, high repeatability
	CMD_MEAS_PERI_05_M = 0x2024, // measurement: periodic 0.5 mps, medium repeatability
	CMD_MEAS_PERI_05_L = 0x202F, // measurement: periodic 0.5 mps, low repeatability
	CMD_MEAS_PERI_1_H = 0x2130, // measurement: periodic 1 mps, high repeatability
	CMD_MEAS_PERI_1_M = 0x2126, // measurement: periodic 1 mps, medium repeatability
	CMD_MEAS_PERI_1_L = 0x212D, // measurement: periodic 1 mps, low repeatability
	CMD_MEAS_PERI_2_H = 0x2236, // measurement: periodic 2 mps, high repeatability
	CMD_MEAS_PERI_2_M = 0x2220, // measurement: periodic 2 mps, medium repeatability
	CMD_MEAS_PERI_2_L = 0x222B, // measurement: periodic 2 mps, low repeatability
	CMD_MEAS_PERI_4_H = 0x2334, // measurement: periodic 4 mps, high repeatability
	CMD_MEAS_PERI_4_M = 0x2322, // measurement: periodic 4 mps, medium repeatability
	CMD_MEAS_PERI_4_L = 0x2329, // measurement: periodic 4 mps, low repeatability
	CMD_MEAS_PERI_10_H = 0x2737, // measurement: periodic 10 mps, high repeatability
	CMD_MEAS_PERI_10_M = 0x2721, // measurement: periodic 10 mps, medium repeatability
	CMD_MEAS_PERI_10_L = 0x272A, // measurement: periodic 10 mps, low repeatability
	CMD_FETCH_DATA = 0xE000, // readout measurements for periodic mode
	CMD_R_AL_LIM_LS = 0xE102, // read alert limits, low set
	CMD_R_AL_LIM_LC = 0xE109, // read alert limits, low clear
	CMD_R_AL_LIM_HS = 0xE11F, // read alert limits, high set
	CMD_R_AL_LIM_HC = 0xE114, // read alert limits, high clear
	CMD_W_AL_LIM_HS = 0x611D, // write alert limits, high set
	CMD_W_AL_LIM_HC = 0x6116, // write alert limits, high clear
	CMD_W_AL_LIM_LC = 0x610B, // write alert limits, low clear
	CMD_W_AL_LIM_LS = 0x6100, // write alert limits, low set
	CMD_NO_SLEEP = 0x303E,
} etCommands;

typedef enum {
	REPEATAB_HIGH,   // high repeatability
	REPEATAB_MEDIUM, // medium repeatability
	REPEATAB_LOW,    // low repeatability
} etRepeatability;

// Measurement Mode
typedef enum {
	MODE_CLKSTRETCH, // clock stretching
	MODE_POLLING,    // polling
} etMode;

typedef enum {
	FREQUENCY_HZ5,  //  0.5 measurements per seconds
	FREQUENCY_1HZ,  //  1.0 measurements per seconds
	FREQUENCY_2HZ,  //  2.0 measurements per seconds
	FREQUENCY_4HZ,  //  4.0 measurements per seconds
	FREQUENCY_10HZ, // 10.0 measurements per seconds
} etFrequency;

typedef struct {
	float temperatureHighSet;
	float temperatureHighClear;
	float temperatureLowClear;
	float temperatureLowSet;
	float humidityHighSet;
	float humidityHighClear;
	float humidityLowClear;
	float humidityLowSet;
} AlertRec;

class SHT35Dev: public SHT35DevPub {
private:
	enum {
		POLYNOMIAL = 0x131, // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
	};
	uint16_t rdSwap(uint8_t *p);
	HAL_StatusTypeDef ReadWordWithCrc(uint16_t cmdSHT, uint16_t *val);

	HAL_StatusTypeDef ReadSerialNumber(uint32_t *serialNumber);
	HAL_StatusTypeDef ReadStatus(uint16_t *status);
	uint8_t CalcCrc(uint8_t *data, uint8_t cnt);

	float CalcTemperature(uint16_t rawValue);
	float CalcHumidity(uint16_t rawValue);
	uint16_t CalcRawTemperature(float temperature);
	uint16_t CalcRawHumidity(float humidity);

	HAL_StatusTypeDef writeCmd(uint16_t cmd);
	HAL_StatusTypeDef EnableHeater(void);
	HAL_StatusTypeDef DisableHeater(void);
	HAL_StatusTypeDef ClearAllAlertFlags(void);
	HAL_StatusTypeDef SoftReset(void);
	HAL_StatusTypeDef SoftBreak(void);

	HAL_StatusTypeDef SendAlertData(uint16_t cmdSHT, float humidity, float temperature);
	HAL_StatusTypeDef ReadAlertData(uint16_t cmdSHT, float *humidity, float *temperature);
	HAL_StatusTypeDef setAlerts(AlertRec *rec);
	HAL_StatusTypeDef readAlerts(AlertRec *rec);
	HAL_StatusTypeDef StartPeriodicMeasurment(etRepeatability repeatability, etFrequency frequency);

	void showSerialNumer(OutStream *strm);
	void showStatus(OutStream *strm);
	void showMeasData(OutStream *strm);
	struct {
		float temp;
		float humi;
	} meas;
	enum {
		MEAS_VALID = 1000, //1 sekunda
	};
	DtFilter filterTemp;
	DtFilter filterHumidity;
	uint32_t mLastRdDataTick;
	uint32_t mLastTryRdTick;
	HAL_StatusTypeDef readData();
protected:
	virtual void tick();
	virtual void init(bool zeroVar);
	virtual const ShellItem* getShellMenu();
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
public:
	SHT35Dev(I2cBus *bus, uint8_t adr);
	virtual HAL_StatusTypeDef getData(float *temperature, float *humidity);
	virtual void showState(OutStream *strm);
	virtual void showMeas(OutStream *strm);
	virtual bool isError();
	virtual bool isRdAuto();
};

SHT35DevPub::SHT35DevPub(I2cBus *bus, uint8_t devAdr) :
		I2c1Dev::I2c1Dev(bus, devAdr) {
	mReadAuto = false;

}

SHT35DevPub* SHT35DevPub::createDev(I2cBus *bus, uint8_t devAdr) {
	SHT35DevPub *dev = new SHT35Dev(bus, devAdr);
	bus->addDev(dev);
	dev->init(true);
	return dev;
}

SHT35Dev::SHT35Dev(I2cBus *bus, uint8_t adr) :
		SHT35DevPub::SHT35DevPub(bus, adr) {
}

void SHT35Dev::init(bool zeroVar) {
	mDevExist = (checkDev() == HAL_OK);

//SoftReset();
	ReadSerialNumber(&serialNr);
	mMeasStart = StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_2HZ);
	DisableHeater();
	if (zeroVar) {
		filterTemp.init(FILTR_FACTOR);
		filterHumidity.init(FILTR_FACTOR);
		mLastRdDataTick = HAL_GetTick();
		mLastTryRdTick = HAL_GetTick();
	}
}

uint16_t SHT35Dev::rdSwap(uint8_t *p) {
	uint16_t w;
	w = p[0] << 8;
	w |= p[1];
	return w;
}

uint8_t SHT35Dev::CalcCrc(uint8_t *data, uint8_t cnt) {
	uint8_t crc = 0xFF;

	for (int j = 0; j < cnt; j++) {
		crc ^= data[j];
		for (int ii = 8; ii > 0; --ii) {
			if (crc & 0x80)
				crc = (crc << 1) ^ POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}
	return crc;
}

float SHT35Dev::CalcTemperature(uint16_t rawValue) {
// calculate temperature [°C]
// T = -45 + 175 * rawValue / (2^16-1)
	return 175.0f * (float) rawValue / 65535.0f - 45.0f;
}

float SHT35Dev::CalcHumidity(uint16_t rawValue) {
// calculate relative humidity [%RH]
// RH = rawValue / (2^16-1) * 100
	return 100.0f * (float) rawValue / 65535.0f;
}

uint16_t SHT35Dev::CalcRawTemperature(float temperature) {
// calculate raw temperature [ticks]
// rawT = (temperature + 45) / 175 * (2^16-1)
	return (temperature + 45.0f) / 175.0f * 65535.0f;
}

uint16_t SHT35Dev::CalcRawHumidity(float humidity) {
// calculate raw relative humidity [ticks]
// rawRH = humidity / 100 * (2^16-1)
	return humidity / 100.0f * 65535.0f;
}

HAL_StatusTypeDef SHT35Dev::ReadSerialNumber(uint32_t *serialNumber) {
	uint8_t data[6];

	HAL_StatusTypeDef st = readBytes16B(CMD_READ_SERIALNBR, data, 6);
	if (st == HAL_OK) {
		uint8_t crc1 = CalcCrc(&data[0], 2);
		uint8_t crc2 = CalcCrc(&data[3], 2);
		if (crc1 == data[2] && crc2 == data[5]) {
			uint32_t sn;
			sn = data[0] << 24;
			sn |= data[1] << 16;
			sn |= data[3] << 8;
			sn |= data[4];
			*serialNumber = sn;
		} else {
			*serialNumber = 0;
			st = HAL_CRC_ERR;
		}
	}

	return st;
}

HAL_StatusTypeDef SHT35Dev::ReadWordWithCrc(uint16_t cmdSHT, uint16_t *val) {
	uint8_t data[3];

	HAL_StatusTypeDef st = readBytes16B(cmdSHT, data, 3);
	if (st == HAL_OK) {
		uint8_t crc1 = CalcCrc(&data[0], 2);
		if (crc1 == data[2]) {
			*val = rdSwap(data);
		} else {
			*val = 0;
			st = HAL_CRC_ERR;
		}
	}
	return st;
}

HAL_StatusTypeDef SHT35Dev::ReadStatus(uint16_t *status) {
	return ReadWordWithCrc(CMD_READ_STATUS, status);
}

HAL_StatusTypeDef SHT35Dev::writeCmd(uint16_t cmd) {
	uint8_t data[2];
	data[0] = cmd >> 8;
	data[1] = cmd;
	return masterTransmit(data, 2);
}

HAL_StatusTypeDef SHT35Dev::EnableHeater(void) {
	HAL_StatusTypeDef st = writeCmd(CMD_HEATER_ENABLE);
	HAL_Delay(5);
	return st;
}

HAL_StatusTypeDef SHT35Dev::DisableHeater(void) {
	HAL_StatusTypeDef st = writeCmd(CMD_HEATER_DISABLE);
	HAL_Delay(5);
	return st;

}

HAL_StatusTypeDef SHT35Dev::ClearAllAlertFlags(void) {
	return writeCmd(CMD_CLEAR_STATUS);
}

HAL_StatusTypeDef SHT35Dev::SoftReset(void) {
	return writeCmd(CMD_SOFT_RESET);
}

HAL_StatusTypeDef SHT35Dev::SoftBreak(void) {
	return writeCmd(CMD_BREAK);
}

HAL_StatusTypeDef SHT35Dev::SendAlertData(uint16_t cmdSHT, float humidity, float temperature) {
	if ((humidity < 0.0f) || (humidity > 100.0f) || (temperature < -45.0f) || (temperature > 130.0f)) {
		return HAL_DATA_ERR;
	} else {
		uint16_t rawHumidity = CalcRawHumidity(humidity);
		uint16_t rawTemperature = CalcRawTemperature(temperature);
		uint16_t w = (rawHumidity & 0xFE00) | ((rawTemperature >> 7) & 0x001FF);
		uint8_t tab[3];
		tab[0] = w >> 8;
		tab[1] = w & 0xff;
		tab[2] = CalcCrc(tab, 2);
		return writeBytes16B(cmdSHT, tab, 3);
	}
}

HAL_StatusTypeDef SHT35Dev::ReadAlertData(uint16_t cmdSHT, float *humidity, float *temperature) {
	uint16_t data;
	HAL_StatusTypeDef st = ReadWordWithCrc(cmdSHT, &data);
	if (st == HAL_OK) {

		*humidity = CalcHumidity(data & 0xFE00);
		*temperature = CalcTemperature(data << 7);
	}
	return st;

}

HAL_StatusTypeDef SHT35Dev::setAlerts(AlertRec *rec) {
	HAL_StatusTypeDef st = HAL_NO_SEMF;

	st = SendAlertData(CMD_W_AL_LIM_HS, rec->humidityHighSet, rec->temperatureHighSet);
	if (st == HAL_OK)
		st = SendAlertData(CMD_W_AL_LIM_HC, rec->humidityHighClear, rec->temperatureHighClear);
	if (st == HAL_OK)
		st = SendAlertData(CMD_W_AL_LIM_LC, rec->humidityLowClear, rec->temperatureLowClear);
	if (st == HAL_OK)
		st = SendAlertData(CMD_W_AL_LIM_LS, rec->humidityLowSet, rec->temperatureLowSet);
	return st;

}

HAL_StatusTypeDef SHT35Dev::readAlerts(AlertRec *rec) {
	HAL_StatusTypeDef st;
	st = ReadAlertData(CMD_W_AL_LIM_HS, &rec->humidityHighSet, &rec->temperatureHighSet);
	if (st == HAL_OK)
		st = ReadAlertData(CMD_W_AL_LIM_HC, &rec->humidityHighClear, &rec->temperatureHighClear);
	if (st == HAL_OK)
		st = ReadAlertData(CMD_W_AL_LIM_LC, &rec->humidityLowClear, &rec->temperatureLowClear);
	if (st == HAL_OK)
		st = ReadAlertData(CMD_W_AL_LIM_LS, &rec->humidityLowSet, &rec->temperatureLowSet);
	return st;

}

const uint16_t TabStartCmd[3][5] = { //
		{ CMD_MEAS_PERI_05_H, CMD_MEAS_PERI_1_H, CMD_MEAS_PERI_2_H, CMD_MEAS_PERI_4_H, CMD_MEAS_PERI_10_H }, //
				{ CMD_MEAS_PERI_05_M, CMD_MEAS_PERI_1_M, CMD_MEAS_PERI_2_M, CMD_MEAS_PERI_4_M, CMD_MEAS_PERI_10_M }, //
				{ CMD_MEAS_PERI_05_L, CMD_MEAS_PERI_1_L, CMD_MEAS_PERI_2_L, CMD_MEAS_PERI_4_L, CMD_MEAS_PERI_10_L } //
		};

HAL_StatusTypeDef SHT35Dev::StartPeriodicMeasurment(etRepeatability repeatability, etFrequency frequency) {
	HAL_StatusTypeDef st;

	if (repeatability >= 0 && repeatability < 3 && frequency >= 0 && frequency < 5) {
		uint16_t cmdSHT = TabStartCmd[repeatability][frequency];
		st = writeCmd(cmdSHT);
	} else
		st = HAL_DATA_ERR;
	return st;
}

bool SHT35Dev::isError() {
	return (HAL_GetTick() - mLastRdDataTick > TIME_DT_VALID);
}

HAL_StatusTypeDef SHT35Dev::getData(float *temperature, float *humidity) {
	if (!isError()) {
		*temperature = filterTemp.get();
		*humidity = filterHumidity.get();
		return HAL_OK;
	} else {
		*temperature = NAN;
		*humidity = NAN;
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef SHT35Dev::readData() {

	meas.temp = NAN;
	meas.humi = NAN;

	uint8_t data[6];
	HAL_StatusTypeDef st = readBytes16B(CMD_FETCH_DATA, data, 6);

	if (st == HAL_OK) {
		uint8_t crc1 = CalcCrc(&data[0], 2);
		uint8_t crc2 = CalcCrc(&data[3], 2);
		if (crc1 == data[2] && crc2 == data[5]) {
			uint16_t rawTemp = rdSwap(&data[0]);
			uint16_t rawHumi = rdSwap(&data[3]);
			meas.temp = CalcTemperature(rawTemp);
			meas.humi = CalcHumidity(rawHumi);
		} else {
			st = HAL_CRC_ERR;
		}
	} else {
		iicBus->BusRestart();
		mMeasStart = StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_2HZ);
	}
	filterTemp.inp(meas.temp);
	filterHumidity.inp(meas.humi);
	return st;
}

bool SHT35Dev::isRdAuto(){
	return mReadAuto;
}


void SHT35Dev::tick() {
	if (isRdAuto()) {
		uint32_t tt = HAL_GetTick();
		if (tt - mLastRdDataTick > TIME_DT_RD) {
			if (tt - mLastTryRdTick > 500) {
				mLastTryRdTick = tt;
				if (readData() == HAL_OK) {
					mLastRdDataTick = tt;
				}
			}
		}
	}
}

void SHT35Dev::showMeasData(OutStream *strm) {
	float temperature, humidity;
	HAL_StatusTypeDef st = getData(&temperature, &humidity);
	if (st == HAL_OK)
		strm->oMsg("Temper=%.2f[st]  Humi=%.1f[%%]", temperature, humidity);
	else
		strm->oMsg("Data error:%s", HAL_getErrStr(st));
}

void SHT35Dev::showStatus(OutStream *strm) {
	uint16_t status;
	HAL_StatusTypeDef st = ReadStatus(&status);
	if (st == HAL_OK) {
		status &= 0xAC13; // maskowanie pól reserved
		strm->oMsg("Status=0x%04X", status);
	} else
		strm->oMsg("status error:%s", HAL_getErrStr(st));
}

void SHT35Dev::showSerialNumer(OutStream *strm) {
	uint32_t sn;
	HAL_StatusTypeDef st = ReadSerialNumber(&sn);
	if (st == HAL_OK)
		strm->oMsg("SerialNb=0x%08X", sn);
	else
		strm->oMsg("SerialNb error:%s", HAL_getErrStr(st));
}

void SHT35Dev::showMeas(OutStream *strm) {
	float temperature, humidity;
	HAL_StatusTypeDef st = getData(&temperature, &humidity);
	if (st == HAL_OK)
		strm->oMsg("SHT35 : Temper=%.2f[st]  Humi=%.1f[%%]", temperature, humidity);
	else
		strm->oMsg("SHT35 : Data error:%s", HAL_getErrStr(st));
}

void SHT35Dev::showState(OutStream *strm) {
	strm->oMsg("__SHT35Dev__");
	strm->oMsg("chipExist: %s", YN(mDevExist));
	if (mDevExist) {
		strm->oMsg("AutoRd   =%u", isRdAuto());
		strm->oMsg("SerialNb =0x%08X", serialNr);
		strm->oMsg("MeasStart=%s", HAL_getErrStr(mMeasStart));
		showDevExist(strm);
		showStatus(strm);
		showMeasData(strm);

	}
}

const ShellItem menuSHT35[] = { //

		{ "SHT_show_alert", "SHT35 pokaż allerty " }, //
				{ "SHT_set_alert", "SHT35 ustaw zestaw alertów, parametr: 1|2. " }, //
				{ "SHT_status", "SHT35 pokaż rejestr statusu" }, //
				{ "SHT_clear", "SHT35 czyść flagi alertów" }, //
				{ "SHT_reset", "SHT35 software reset" }, //
				{ "SHT_start", "SHT35 start pomiarów" }, //
				{ "SHT_sn", "SHT35 czytaj serial number" }, //
				{ "SHT_meas", "SHT35 pokaz pomiary" }, //

				{ NULL, NULL } };

const ShellItem* SHT35Dev::getShellMenu() {
	return menuSHT35;
}
bool SHT35Dev::execMenuItem(OutStream *strm, int idx, const char *cmd) {
	AlertRec alert;
	HAL_StatusTypeDef st;

	idx -= mMenuOffset;
	switch (idx) {
	case 0:
		//SHT_show_alert
		st = readAlerts(&alert);
		if (st == HAL_OK) {
			strm->oMsg("TempHigh: %.0f-%.0f", alert.temperatureHighClear, alert.temperatureHighSet);
			strm->oMsg("TempLow : %.0f-%.0f", alert.temperatureLowClear, alert.temperatureLowSet);
			strm->oMsg("HumiHigh: %.0f-%.0f", alert.humidityHighClear, alert.humidityHighSet);
			strm->oMsg("HumiLow : %.0f-%.0f", alert.humidityLowClear, alert.humidityLowSet);
		} else
			strm->oMsg("readAlerts error:%s", HAL_getErrStr(st));
		break;
	case 1: { //SHT_set_alert
		int nr;
		if (Token::getAsInt(&cmd, &nr)) {
			bool doIt = false;
			switch (nr) {
			case 1:

				alert.temperatureHighSet = 30;
				alert.temperatureHighClear = 25;
				alert.temperatureLowClear = 10;
				alert.temperatureLowSet = 5;
				alert.humidityHighSet = 80;
				alert.humidityHighClear = 75;
				alert.humidityLowClear = 60;
				alert.humidityLowSet = 55;
				doIt = true;
				break;
			case 2:

				alert.temperatureHighSet = 33;
				alert.temperatureHighClear = 28;
				alert.temperatureLowClear = 13;
				alert.temperatureLowSet = 8;
				alert.humidityHighSet = 82;
				alert.humidityHighClear = 77;
				alert.humidityLowClear = 62;
				alert.humidityLowSet = 57;
				doIt = true;
				break;
			}
			if (doIt) {

				strm->oMsg("setAlerts st=%s", HAL_getErrStr(setAlerts(&alert)));
			}
		}
	}
		break;
	case 2: //SHT_status
		showStatus(strm);
		break;
	case 3: //SHT_clear
		strm->oMsg("ClearAllAlertFlags st=%s", HAL_getErrStr(ClearAllAlertFlags()));
		break;
	case 4: //SHT_reset
		strm->oMsg("SoftReset st=%s", HAL_getErrStr(SoftReset()));
		break;
	case 5: //SHT_start
		st = StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_4HZ);
		strm->oMsg("StartPeriodicMeasurment st=%s", HAL_getErrStr(st));
		break;
	case 6: //SHT_sn
		showSerialNumer(strm);
		break;

	case 7: //SHT_sn
		showMeasData(strm);
		break;
	case 8: //SHT_meas
		showDevExist(strm);
		break;
	default:
		return false;

	}
	return true;
}

#endif

//-------------------------------------------------------------------------------------------------------------------------
// Bmp338Dev
//-------------------------------------------------------------------------------------------------------------------------
#if I2C_SERV_BMP338

typedef struct {
	uint16_t T1, T2, P5, P6;
	int16_t P1, P2, P9;
	int8_t T3, P3, P4, P7, P8, P10, P11;
	double T1f, T2f, T3f, P1f, P2f, P3f, P4f, P5f, P6f, P7f, P8f, P9f, P10f, P11f;
} Calibration;

#define AC_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

class Bmp338Dev: public Bmp338DevPub {
private:
	enum {
		REG_CHIPID = 0x00, //
		REG_CAL_DATA = 0x31, //
		REG_CMD = 0x7E, //
		REG_CONTROL = 0x1B, //
		REG_STATUS = 0x03, //
		REG_PRESS = 0x04, //
	};

	enum {
		CHIPID = 0x50, // wartość w rejestrze REG_CHIP_ID
		CMD_RESET = 0xB6, //
		CMD_MEASURE = 0x13, //  rozkaz wykonania pomiaru
		STATUS = 0x60, //
	};

	Calibration _c;
	HAL_StatusTypeDef read_coefficients();
	uint32_t mLastRdDataTick;
	uint32_t mLastTryRdTick;

	double ac_pow(double base, uint8_t power);

	double comp_temperature(uint32_t T);
	double comp_pressure(uint32_t P, double T);
	HAL_StatusTypeDef reset();
	HAL_StatusTypeDef measure(double *temp, double *pressure);
	void showMeasData(OutStream *strm);
	HAL_StatusTypeDef Start();
	DtFilter filterTemper;
	DtFilter filterPressure;
	HAL_StatusTypeDef readData();
protected:
	virtual void tick();
	virtual void init(bool zeroVar);
	virtual const ShellItem* getShellMenu();
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
public:
	Bmp338Dev(I2cBus *bus, uint8_t adr);
	virtual HAL_StatusTypeDef getData(float *temperature, float *pressure);
	virtual void showState(OutStream *strm);
	virtual void execFun(OutStream *strm, int funNr);
	virtual void showMeas(OutStream *strm);
	virtual bool isError();

};
Bmp338DevPub::Bmp338DevPub(I2cBus *bus, uint8_t devAdr) :
		I2c1Dev::I2c1Dev(bus, devAdr) {

}

Bmp338DevPub* Bmp338DevPub::createDev(I2cBus *bus, uint8_t devAdr) {
	Bmp338DevPub *dev = new Bmp338Dev(bus, devAdr);
	bus->addDev(dev);
	dev->init(true);
	return dev;

}

Bmp338Dev::Bmp338Dev(I2cBus *bus, uint8_t adr) :
		Bmp338DevPub::Bmp338DevPub(bus, adr) {
}

void Bmp338Dev::init(bool zeroVar) {
	chipIdOk = false;
	coefOk = false;
	Start();
	if (zeroVar) {
		filterTemper.init(FILTR_FACTOR);
		filterPressure.init(FILTR_FACTOR);
		mLastRdDataTick = HAL_GetTick();
		mLastTryRdTick = HAL_GetTick();
	}
}

HAL_StatusTypeDef Bmp338Dev::Start() {
	HAL_StatusTypeDef st = HAL_NO_SEMF;
	mDevExist = (checkDev() == HAL_OK);
	if (mDevExist) {
		uint8_t b;
		st = readByte(REG_CHIPID, &b);
		if (st == HAL_OK) {
			chipIdOk = (b == CHIPID);
			reset();
			coefOk = (read_coefficients() == HAL_OK);
		}
	}
	return st;
}

HAL_StatusTypeDef Bmp338Dev::reset() {
	return writeByte(REG_CMD, CMD_RESET);
}

HAL_StatusTypeDef Bmp338Dev::read_coefficients() {
	const int size = 21;
	uint8_t b[size];

	HAL_StatusTypeDef st = readBytes(REG_CAL_DATA, b, size);

	_c.T1 = (uint16_t) AC_CONCAT_BYTES(b[1], b[0]);
	_c.T2 = (uint16_t) AC_CONCAT_BYTES(b[3], b[2]);
	_c.T3 = (int8_t) b[4];
	_c.P1 = (int16_t) AC_CONCAT_BYTES(b[6], b[5]);
	_c.P2 = (int16_t) AC_CONCAT_BYTES(b[8], b[7]);
	_c.P3 = (int8_t) b[9];
	_c.P4 = (int8_t) b[10];
	_c.P5 = (uint16_t) AC_CONCAT_BYTES(b[12], b[11]);
	_c.P6 = (uint16_t) AC_CONCAT_BYTES(b[14], b[13]);
	_c.P7 = (int8_t) b[15];
	_c.P8 = (int8_t) b[16];
	_c.P9 = (int16_t) AC_CONCAT_BYTES(b[18], b[17]);
	_c.P10 = (int8_t) b[19];
	_c.P11 = (int8_t) b[20];

	_c.T1f = (double) _c.T1 / 0.00390625f;
	_c.T2f = (double) _c.T2 / 1073741824.0f;
	_c.T3f = (double) _c.T3 / 281474976710656.0f;
	_c.P1f = ((double) _c.P1 - 16384) / 1048576.0f;
	_c.P2f = ((double) _c.P2 - 16384) / 536870912.0f;
	_c.P3f = (double) _c.P3 / 4294967296.0f;
	_c.P4f = (double) _c.P4 / 137438953472.0f;
	_c.P5f = (double) _c.P5 / 0.125f;
	_c.P6f = (double) _c.P6 / 64.0f;
	_c.P7f = (double) _c.P7 / 256.0f;
	_c.P8f = (double) _c.P8 / 32768.0f;
	_c.P9f = (double) _c.P9 / 281474976710656.0f;
	_c.P10f = (double) _c.P10 / 281474976710656.0f;
	_c.P11f = (double) _c.P11 / 36893488147419103232.0f;

	return st;
}

double Bmp338Dev::comp_temperature(uint32_t T) {
	const double TP1 = (double) (T - _c.T1f);
	const double TP2 = (double) (TP1 * _c.T2f);
	return TP2 + (TP1 * TP1) * _c.T3f;
}

double Bmp338Dev::ac_pow(double base, uint8_t power) {
	double pow_output = 1;
	while (power != 0) {
		pow_output = base * pow_output;
		power--;
	}
	return pow_output;
}

/* Calibrate pressure reading */
double Bmp338Dev::comp_pressure(uint32_t P, double T) {
	double partial_data1;
	double partial_data2;
	double partial_data3;
	double partial_data4;
	double partial_out1;
	double partial_out2;

	partial_data1 = _c.P6f * T;
	partial_data2 = _c.P7f * ac_pow(T, 2);
	partial_data3 = _c.P8f * ac_pow(T, 3);
	partial_out1 = _c.P5f + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = _c.P2f * T;
	partial_data2 = _c.P3f * ac_pow(T, 2);
	partial_data3 = _c.P4f * ac_pow(T, 3);
	partial_out2 = P * (_c.P1f + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = ac_pow((double) P, 2);
	partial_data2 = _c.P9f + _c.P10f * T;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ac_pow((double) P, 3) * _c.P11f;

	return partial_out1 + partial_out2 + partial_data4;
}

HAL_StatusTypeDef Bmp338Dev::measure(double *temp, double *press) {
	HAL_StatusTypeDef st = HAL_NO_SEMF;

	st = writeByte(REG_CONTROL, CMD_MEASURE);
	if (st == HAL_OK) {

		int loop_count = 0;
		while (1) {
			loop_count++;
			HAL_Delay(200);
			uint8_t v = 0;
			st = readByte(REG_STATUS, &v);
			if (st != HAL_OK)
				break;
			if (v == 0x70) {
				break;
			}
			if (loop_count > 10) {
				st = HAL_TIMEOUT;
				break;
			}
		}

		if (st == HAL_OK) {
			uint8_t p[6];
			st = readBytes(REG_PRESS, p, 6);
			if (st == HAL_OK) {

				uint32_t P = (p[2] << 16) | (p[1] << 8) | p[0];
				uint32_t T = (p[5] << 16) | (p[4] << 8) | p[3];

				double temperature = comp_temperature(T);
				double pressure = comp_pressure(P, temperature);

				*temp = temperature;
				*press = pressure / 100;  //hPa
			}
		}
	}
	if (st != HAL_OK) {
		*temp = NAN;
		*press = NAN;
	}
	return st;
}

bool Bmp338Dev::isError() {
	return (HAL_GetTick() - mLastRdDataTick > TIME_DT_VALID);
}

HAL_StatusTypeDef Bmp338Dev::getData(float *temperature, float *pressure) {
	if (!isError()) {
		*temperature = filterTemper.get();
		*pressure = filterPressure.get();
		return HAL_OK;
	} else {
		*temperature = NAN;
		*pressure = NAN;
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef Bmp338Dev::readData() {
	double d_temperature, d_pressure;
	HAL_StatusTypeDef st = measure(&d_temperature, &d_pressure);
	if (st == HAL_OK) {
		filterTemper.inp(d_temperature);
		filterPressure.inp(d_pressure);
	}
	return st;
}

void Bmp338Dev::tick() {
	uint32_t tt = HAL_GetTick();
	if (tt - mLastRdDataTick > TIME_DT_RD) {
		if (tt - mLastTryRdTick > 200) {
			mLastTryRdTick = tt;
			if (readData() == HAL_OK) {
				mLastRdDataTick = tt;
			}
		}
	}
}

void Bmp338Dev::execFun(OutStream *strm, int funNr) {
	switch (funNr) {
	case 20:
		strm->oMsg("BMP338 Start st=%s", HAL_getErrStr(Start()));
		strm->oMsg("chipIdOk: %s", OkErr(chipIdOk));
		strm->oMsg("coefOk: %s", OkErr(coefOk));
		break;
	}
}

void Bmp338Dev::showMeasData(OutStream *strm) {
	double temperature, pressure;
	HAL_StatusTypeDef st = measure(&temperature, &pressure);
	if (st == HAL_OK)
		strm->oMsg("Temper=%.2f[st]  Pressure=%.2f[hPa]", temperature, pressure);
	else
		strm->oMsg("Data error:%s", HAL_getErrStr(st));
}

void Bmp338Dev::showMeas(OutStream *strm) {
	double temperature, pressure;
	HAL_StatusTypeDef st = measure(&temperature, &pressure);
	if (st == HAL_OK)
		strm->oMsg("BMP338: Temper=%.2f[st]  Pressure=%.2f[hPa]", temperature, pressure);
	else
		strm->oMsg("BMP338: Data error:%s", HAL_getErrStr(st));

}

void Bmp338Dev::showState(OutStream *strm) {
	strm->oMsg("__BMP338__");
	strm->oMsg("chipExist: %s", YN(mDevExist));
	if (mDevExist) {
		strm->oMsg("chipIdOk: %s", OkErr(chipIdOk));
		strm->oMsg("coefOk: %s", OkErr(coefOk));
		showMeasData(strm);
	}
}

const ShellItem menuBmp338[] = { //
		{ "BMP_init", "inicjuj BMP338" }, //
				{ NULL, NULL } };

const ShellItem* Bmp338Dev::getShellMenu() {
	return menuBmp338;
}

bool Bmp338Dev::execMenuItem(OutStream *strm, int idx, const char *cmd) {
	idx -= mMenuOffset;
	switch (idx) {
	case 0:
		strm->oMsg("BMP338 Start st=%s", HAL_getErrStr(Start()));
		strm->oMsg("chipIdOk: %s", OkErr(chipIdOk));
		strm->oMsg("coefOk: %s", OkErr(coefOk));
		break;
	default:
		return false;
	}

	return true;
}

#endif

//-------------------------------------------------------------------------------------------------------------------------
// TidsDevPub
//-------------------------------------------------------------------------------------------------------------------------
#if I2C_SERV_TIDS

class TidsDev: public TidsDevPub {
private:
	enum {
		REG_DEVICEID = 0x01, //
		REG_T_H_LIMIT = 0x02, //
		REG_T_L_LIMIT = 0x03, //
		REG_CTRL = 0x04, //
		REG_STATUS = 0x05, //
		REG_DATA_L = 0x06, //
		REG_DATA_H = 0x07, //
		REG_SOFT_RESET = 0x0C, //

		TIDS_DEVICEID = 0xA0, //
		REG_CTRL_RUN_VAL = 0x7C, //
	};
	enum {
		MEAS_VALID = 1000, //1 sekunda
	};
	DtFilter filterTemp;
	uint32_t mLastRdDataTick;
	uint32_t mLastTryRdTick;
	float makeTemp(uint8_t l, uint8_t h);
	HAL_StatusTypeDef readData();
protected:

	virtual void tick();
	virtual void init(bool zeroVar);
	virtual const ShellItem* getShellMenu();
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	virtual void showState(OutStream *strm);

public:
	TidsDev(I2cBus *bus, uint8_t adr);
	virtual HAL_StatusTypeDef getData(float *temperature);
	virtual bool isError();
	virtual bool isRdAuto();
};

bool TidsDevPub::mManyDev;

TidsDevPub::TidsDevPub(I2cBus *bus, uint8_t devAdr) :
		I2c1Dev::I2c1Dev(bus, devAdr) {
	mReadAuto = false;
	mManyDev = false;
}
TidsDevPub* TidsDevPub::createDev(I2cBus *bus, uint8_t devAdr) {
	//UWAGA: w konstructorze nie sa jeszcze podłaczone metody virtualne typów potomnych
	TidsDevPub *dev = new TidsDev(bus, devAdr);
	bus->addDev(dev);
	dev->init(true);
	return dev;
}
//---------

TidsDev::TidsDev(I2cBus *bus, uint8_t devAdr) :
		TidsDevPub::TidsDevPub(bus, devAdr) {
	chipIdOk = false;
}

bool TidsDev::isError() {
	return (HAL_GetTick() - mLastRdDataTick > TIME_DT_VALID);
}

void TidsDev::showState(OutStream *strm) {
	strm->oMsg("__TIDS_0x%02X_", mDevAdr);
	strm->oMsg("AutoRd   =%u", isRdAuto());
	strm->oMsg("chipExist: %s", YN(mDevExist));
	if (mDevExist) {
		uint8_t ctrl;
		readByte(REG_CTRL, &ctrl);
		strm->oMsg("CTRL   : 0x%02X", ctrl);
	}
}

HAL_StatusTypeDef TidsDev::getData(float *temperature) {
	if (!isError()) {
		*temperature = filterTemp.get();
		return HAL_OK;
	} else {
		*temperature = NAN;
		return HAL_ERROR;
	}
}

void TidsDev::init(bool zeroVar) {
	mDevExist = (checkDev() == HAL_OK);
	uint8_t mId;
	HAL_StatusTypeDef st = readByte(REG_DEVICEID, &mId);
	chipIdOk = (st == HAL_OK && mId == TIDS_DEVICEID);
	if (chipIdOk) {
		writeByte(REG_CTRL, 0x1C); // BDU, AVG=50[Hz] IF_ADC_INC FREE_RUN
		writeByte(REG_CTRL, REG_CTRL_RUN_VAL); // BDU, AVG=50[Hz] IF_ADC_INC FREE_RUN
	}
	if (zeroVar) {
		mLastRdDataTick = HAL_GetTick();
		mLastTryRdTick = HAL_GetTick();
		filterTemp.init(FILTR_FACTOR);
	}
}

HAL_StatusTypeDef TidsDev::readData() {

	float temp = NAN;

	uint8_t data[2];
	HAL_StatusTypeDef st = readBytes(REG_DATA_L, data, 2);
	if (st == HAL_OK) {
		if (data[0] == 0 && data[1] == 0) {
			st = readByte(REG_CTRL, data);
			if (st == HAL_OK) {
				if (data[0] != REG_CTRL_RUN_VAL) {
					st = HAL_ERROR;
				}
			}
		}
	}
	if (st == HAL_OK) {
		temp = makeTemp(data[0], data[1]);
	} else {
		init(false);
	}
	filterTemp.inp(temp);
	return st;
}

bool TidsDev::isRdAuto() {
	return mReadAuto;
}

void TidsDev::tick() {
	if (isRdAuto()) {
		uint32_t tt = HAL_GetTick();
		if (tt - mLastRdDataTick > TIME_DT_RD) {
			if (tt - mLastTryRdTick > 500) {
				mLastTryRdTick = tt;
				if (readData() == HAL_OK) {
					mLastRdDataTick = tt;
				}
			}
		}
	}

}

const ShellItem menuTids[] = { //
		{ "TIDS_status", "TIDS status" }, //
				{ "TIDS_start", "TIDS start measure" }, //
				{ "TIDS_reset", "TIDS reset" }, //
				{ "TIDS_meas", "TIDS measure" }, //
				{ "TIDS_set_limit_L", "" }, //
				{ "TIDS_set_limit_H", "" }, //
				{ NULL, NULL } };

const ShellItem menuTids1[] = { //
		{ "TIDS_status_1", "TIDS status 0x70" }, //
				{ "TIDS_start_1", "TIDS start measure" }, //
				{ "TIDS_reset_1", "TIDS reset" }, //
				{ "TIDS_meas_1", "TIDS measure" }, //
				{ "TIDS_set_limit_L_1", "" }, //
				{ "TIDS_set_limit_H_1", "" }, //
				{ NULL, NULL } };

const ShellItem menuTids2[] = { //
		{ "TIDS_status_2", "TIDS status 0x7E" }, //
				{ "TIDS_start_2", "TIDS start measure" }, //
				{ "TIDS_reset_2", "TIDS reset" }, //
				{ "TIDS_meas_2", "TIDS measure" }, //
				{ "TIDS_set_limit_L_2", "" }, //
				{ "TIDS_set_limit_H_2", "" }, //
				{ NULL, NULL } };

const ShellItem* TidsDev::getShellMenu() {
	if (!mManyDev) {
		return menuTids;
	} else {
		if (mDevAdr == 0x70)
			return menuTids1;
		else
			return menuTids2;
	}
}

float TidsDev::makeTemp(uint8_t L, uint8_t H) {
	int16_t t;
	((uint8_t*) &t)[0] = L;
	((uint8_t*) &t)[1] = H;
	float temp = t;
	temp /= 100.0;
	return temp;
}

bool TidsDev::execMenuItem(OutStream *strm, int idx, const char *cmd) {
	idx -= mMenuOffset;
	switch (idx) {
	case 0: //TIDS_status
		if (strm->oOpen(colWHITE)) {
			strm->oMsg("Exist=%s", YN(mDevExist));
			if (mDevExist) {
				strm->oMsg("ChipIdOK=%s", OkErr(chipIdOk));
#if 0
				uint8_t tab[2];
				uint8_t v;
				HAL_StatusTypeDef st = readByte(REG_DEVICEID, &v);
				if (st == HAL_OK)
					strm->oMsg("Reg_DeviceID=0x%02X", v);

				st = readByte(REG_T_L_LIMIT, &v);
				if (st == HAL_OK)
					strm->oMsg("Reg_Limit_L =%u (%.1f[*C])", v, (v - 63.0) * 0.64);

				st = readByte(REG_T_H_LIMIT, &v);
				if (st == HAL_OK)
					strm->oMsg("Reg_Limit_H =%u (%.1f[*C])", v, (v - 63.0) * 0.64);

				st = readByte(REG_CTRL, &v);
				if (st == HAL_OK)
					strm->oMsg("Reg_Ctrl    =0x%02X", v);
				st = readByte(REG_STATUS, &v);
				if (st == HAL_OK)
					strm->oMsg("Reg_Status  =0x%02X", v);

				st = readBytes(REG_STATUS, tab, 2);
				if (st == HAL_OK)
					strm->oMsg("RegData     =%u, %u %.2f[*C])", tab[0], tab[1], (double) (256 * tab[1] + tab[0]));
#else
				uint8_t tab[7];
				HAL_StatusTypeDef st = readBytes(REG_DEVICEID, tab, 7);
				if (st == HAL_OK) {
					strm->oMsg("Reg_DeviceID=0x%02X", tab[0]);
					strm->oMsg("Reg_Limit_H =%u (%.1f[*C])", tab[1], (tab[1] - 63.0) * 0.64);
					strm->oMsg("Reg_Limit_L =%u (%.1f[*C])", tab[2], (tab[2] - 63.0) * 0.64);
					strm->oMsg("Reg_Ctrl    =0x%02X", tab[3]);
					strm->oMsg("Reg_Status  =0x%02X", tab[4]);
					strm->oMsg("RegData     =%u, %u (%.2f[*C])", tab[5], tab[6], makeTemp(tab[5], tab[6]));
				}
#endif
			} else {
				strm->oMsg("Dev no exists!");
			}

			strm->oClose();
		}
		break;
	case 1:
		//TIDS_start
		init(false);
		strm->oMsgX(colWHITE, "Init, ChipIdOK=%s", OkErr(chipIdOk));
		break;
	case 2: { //TIDS_reset
		HAL_StatusTypeDef st = writeByte(REG_SOFT_RESET, 0x02);
		if (st == HAL_OK) {
			HAL_Delay(5);
			st = writeByte(REG_SOFT_RESET, 0x00);
		}
		strm->oMsgX(colWHITE, "TIDS Reset st=%s", HAL_getErrStr(st));
	}
		break;
	case 3:
		//TIDS_meas
	{
		uint8_t tab[2];
		HAL_StatusTypeDef st = readBytes(REG_DATA_L, tab, 2);
		if (st == HAL_OK) {
			strm->oMsgX(colWHITE, "Temp=%.2f[*C] (%u, %u))", makeTemp(tab[0], tab[1]), tab[0], tab[1]);
		}
	}
		break;
	case 4:
		//TIDS_set_limit_L
	case 5:
		//TIDS_set_limit_H
	{
		uint8_t reg;
		if (idx == 4)
			reg = REG_T_L_LIMIT;
		else
			reg = REG_T_H_LIMIT;

		float f;
		if (Token::getAsFloat(&cmd, &f)) {
			uint8_t v = ((int) (f / 0.64)) + 63;
			HAL_StatusTypeDef st = writeByte(reg, v);
			strm->oMsgX(colWHITE, "Write limit st=%s", HAL_getErrStr(st));
		}

	}

		break;

	default:
		return false;
	}
	return true;
}

#endif

//-------------------------------------------------------------------------------------------------------------------------
// PAPI_Manipulator
//-------------------------------------------------------------------------------------------------------------------------
#if I2C_SERV_PAPI_MAN

class PapiManipDev: public PapiManipDevPub {
private:
	enum {
		regENCODER = 0, //
		regDIP_SW = 1, //
	};
	void rstDev();
	void init(bool zeroVar);
protected:
	virtual const ShellItem* getShellMenu();
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	virtual void tick();
public:
	bool rdNow;
	PapiManipDev(I2cBus *bus, uint8_t devAdr);
	virtual HAL_StatusTypeDef getData(uint8_t *encoder, uint8_t *dipSwitch);
	virtual bool isError();
	virtual void showState(OutStream *strm);

};

PapiManipDevPub::PapiManipDevPub(I2cBus *bus, uint8_t devAdr) :
		I2c1Dev::I2c1Dev(bus, devAdr) {

}

PapiManipDevPub* PapiManipDevPub::createDev(I2cBus *bus, uint8_t devAdr) {
	//UWAGA: w konstructorze nie sa jeszcze podłaczone metody virtualne typów potomnych
	PapiManipDevPub *dev = new PapiManipDev(bus, devAdr);
	bus->addDev(dev);
	dev->init(true);
	return dev;
}
//---------
PapiManipDev::PapiManipDev(I2cBus *bus, uint8_t devAdr) :
		PapiManipDevPub::PapiManipDevPub(bus, devAdr) {
	chipIdOk = false;
}

void PapiManipDev::rstDev() {
	Hdw::setExtRst(false);
	HAL_Delay(5);
	Hdw::setExtRst(true);
}

void PapiManipDev::init(bool zeroVar) {
	rstDev();
	HAL_Delay(1000);

	mDevExist = (checkDev() == HAL_OK);
	uint8_t encoder;
	uint8_t dipSwitch;
	HAL_StatusTypeDef st = getData(&encoder, &dipSwitch);
	chipIdOk = (st == HAL_OK && ((dipSwitch & 0xE0) == 0xA0));
	if (zeroVar) {
		rdNow = true;
	}
}

HAL_StatusTypeDef PapiManipDev::getData(uint8_t *encoder, uint8_t *dipSwitch) {
	uint8_t dt[2];
	HAL_StatusTypeDef st = reciveBytes(dt, sizeof(dt));
	*encoder = dt[0];
	*dipSwitch = dt[1];
	return st;
}

bool PapiManipDev::isError() {
	return !chipIdOk;
}
void PapiManipDev::showState(OutStream *strm) {
	strm->oMsg("__PAPI_MANIP__");
	strm->oMsg("chipExist: %s", YN(mDevExist));
	if (mDevExist) {
		strm->oMsg("chipIdOk: %s", OkErr(chipIdOk));
	}
}

const ShellItem menuPapiManip[] = { //

		{ "rd", "show manip data" }, //
				{ "rst", "restart manip " }, //

				{ NULL, NULL } };

const ShellItem* PapiManipDev::getShellMenu() {
	return menuPapiManip;
}

bool PapiManipDev::execMenuItem(OutStream *strm, int idx, const char *cmd) {

	idx -= mMenuOffset;
	switch (idx) {
	case 0: { //rd

		uint8_t encoder;
		uint8_t dipSwitch;
		HAL_StatusTypeDef st = getData(&encoder, &dipSwitch);
		if (st == HAL_OK) {
			strm->oMsgX(colWHITE, "Encoder=%u", encoder);
			strm->oMsgX(colWHITE, "Switch =0x%02X", dipSwitch & 0x1F);

		} else {
			strm->oMsgX(colRED, "Read Error, st=%d", st);
		}

	}
		break;
	case 1: //rst
		strm->oMsgX(colWHITE, "EXT_RST Pulse.");
		rstDev();
		break;

	default:
		return false;

	}
	return true;
}

void PapiManipDev::tick() {

	if ((Hdw::rdExpInt1() == 0) || (Hdw::rdExpInt2() == 0) || rdNow) {
		rdNow = false;
		uint8_t encoder;
		uint8_t dipSwitch;
		HAL_StatusTypeDef st = getData(&encoder, &dipSwitch);
		globalVar.manipOk = (st == HAL_OK);
		if (st == HAL_OK) {
			globalVar.setEncoder(encoder);
			globalVar.setDipSw(dipSwitch);
		} else {
			globalVar.setNoPulpit();
		}

	}
}

#endif

//-------------------------------------------------------------------------------------------------------------------------
// Veml6030Dev
//-------------------------------------------------------------------------------------------------------------------------
#if I2C_SERV_VEML6030

/*
 0.00001 lx Light from Sirius, the brightest star in the night sky
 0.0001 lx Total starlight, overcast sky
 0.002 lx Moonless clear night sky with airglow
 0.01 lx Quarter moon,
 0.27 lx; full moon on a clear night
 ----
 1 lx Full moon overhead at tropical latitudes
 3.4 lx Dark limit of civil twilight under a clear sky
 50 lx Family living room
 80 lx Hallway / bathroom
 100 lx Very dark overcast day
 320 lx to 500 lx Office lighting
 400 lx Sunrise or sunset on a clear day
 1000 lx Overcast day; typical TV studio lighting
 --
 10 000 lx to 25 000 lx Full daylight (not direct sun)
 32 000 lx to 130 000 lx Direct sunlight
 */

#define VEML_FILTR_FACTOR  0.0  //filtr wyłączony

class Veml6030Dev: public Veml6030DevPub {
private:


	enum {
		cmdALS_CONF = 0, //W
		cmdALS_WH = 1, //W
		cmdALS_WL = 2, //W
		cmdPOWER_SAVE = 3, //
		cmdALS = 4, //R
		cmdWHITE = 5, //R
		cmdALS_INT = 6, //R

	};

	enum REGISTER_BIT_POSITIONS {
		//ALS_CONF
		NO_SHIFT = 0x00, //
		INT_EN_POS = 0x01, //
		PSM_POS = 1, //
		PERS_PROT_POS = 4, //
		INTEG_POS = 6, //
		GAIN_POS = 11, //
		INT_POS = 14, //
		//POWER_DN_CFG
		POWEWR_DN_MODE_POS = 1, //

	};

	enum {
		GAIN_x1 = 0, // x 1.0
		GAIN_x2 = 1, // x 2.0
		GAIN_x1_8 = 2, // x 0.125
		GAIN_x1_4 = 3, // x 0.25
	};

	enum {
		PERS_1 = 0, // persistence=1
		PERS_2 = 1, // persistence=1
		PERS_4 = 2, // persistence=1
		PERS_8 = 3, // persistence=1
	};

	enum {
		INT_TM_100 = 0, // integeration time = 100ms
		INT_TM_200 = 1, // integeration time = 200ms
		INT_TM_400 = 2, // integeration time = 400ms
		INT_TM_800 = 3, // integeration time = 800ms
		INT_TM_025 = 12, // integeration time = 25ms
		INT_TM_050 = 8, // integeration time = 50ms
	};

	enum {
		PW_DN_MODE1 = 0, //mode 1
		PW_DN_MODE2 = 1, //mode 2
		PW_DN_MODE3 = 2, //mode 3
		PW_DN_MODE4 = 3, //mode 4
	};

	struct {
		uint8_t gain;
		uint8_t persistence;
		uint8_t integrTm;
		uint8_t powerDnMode;
	} mCurrent;

	DtFilter filterLight;
	uint32_t mLastRdDataTick;
	bool mDataValid;
	uint32_t mLastTryRdTick;
	bool mRdTestCount;
	uint32_t mLastTestRdTick;

	void rstDev();
	void init(bool zeroVar);
	uint16_t getConfRegVal();
	void showHdMeas(OutStream *strm);
	HAL_StatusTypeDef readData();
	HAL_StatusTypeDef rdAlsReg(uint16_t *w);
	HAL_StatusTypeDef rdWhiteReg(uint16_t *w);

	bool getLuxCoeff(float *coeff);
	float luxCompensation(float val);

	uint16_t calculateBits(float luxVal);
	float calculateLux(uint16_t lightBits);
	uint16_t Swap(uint16_t w);

protected:
	virtual const ShellItem* getShellMenu();
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	virtual void tick();
public:
	bool rdNow;
	Veml6030Dev(I2cBus *bus, uint8_t devAdr);
	bool isRdAuto();
	virtual HAL_StatusTypeDef getData(float *light);
	virtual bool isError();
	virtual void showState(OutStream *strm);

};

Veml6030DevPub::Veml6030DevPub(I2cBus *bus, uint8_t devAdr) :
		I2c1Dev::I2c1Dev(bus, devAdr) {
	mReadAuto = false;
}

Veml6030DevPub* Veml6030DevPub::createDev(I2cBus *bus, uint8_t devAdr) {
	//UWAGA: w konstructorze nie sa jeszcze podłaczone metody virtualne typów potomnych
	Veml6030DevPub *dev = new Veml6030Dev(bus, devAdr);
	bus->addDev(dev);
	dev->init(true);
	return dev;
}

//--------------------------------------------------------------
Veml6030Dev::Veml6030Dev(I2cBus *bus, uint8_t devAdr) :
		Veml6030DevPub::Veml6030DevPub(bus, devAdr) {
	chipIdOk = false;
	mLastRdDataTick = 0;
	mLastTryRdTick = 0;
	//test
	mRdTestCount = false;
	mLastTestRdTick = 0;
}

bool Veml6030Dev::isError() {
	return ((HAL_GetTick() - mLastRdDataTick > TIME_DT_VALID) || !mDataValid);
}

void Veml6030Dev::showState(OutStream *strm) {
	strm->oMsg("__VEML_0x%02X_", mDevAdr);
	strm->oMsg("AutoRd   =%u", isRdAuto());
	strm->oMsg("chipExist: %s", YN(mDevExist));
	if (mDevExist) {
		uint16_t als;
		uint16_t white;
		rdAlsReg(&als);
		rdWhiteReg(&white);

		strm->oMsg("ALS   : 0x%04X(%u)", als, als);
		strm->oMsg("WHITE : 0x%04X(%u)", white, white);

	}
}

HAL_StatusTypeDef Veml6030Dev::getData(float *light) {
	if (!isError()) {
		*light = filterLight.get();
		return HAL_OK;
	} else {
		*light = NAN;
		return HAL_ERROR;
	}
}

uint16_t Veml6030Dev::getConfRegVal() {
	uint16_t w = 0;
	w |= mCurrent.gain << GAIN_POS;
	w |= mCurrent.persistence << PERS_PROT_POS;
	w |= mCurrent.integrTm << INTEG_POS;
	return w;
}

void Veml6030Dev::init(bool zeroVar) {
	mDevExist = (checkDev() == HAL_OK);

	uint16_t v;
	HAL_StatusTypeDef st1 = readWord(cmdALS_INT, &v);
	chipIdOk = (st1 == HAL_OK) && ((v & 0x3fff) == 0x133F);

	mCurrent.gain = GAIN_x2;
	mCurrent.persistence = PERS_1;
	mCurrent.integrTm = INT_TM_100;
	mCurrent.powerDnMode = PW_DN_MODE1;

	writeWord(cmdALS_CONF, 0);  // wyłączenie SLEEP'a
	HAL_Delay(5);

	uint16_t w = getConfRegVal();
	writeWord(cmdALS_CONF, w);

	uint16_t pdm = 0x01;
	pdm |= mCurrent.powerDnMode << POWEWR_DN_MODE_POS;
	writeWord(cmdPOWER_SAVE, pdm);

	if (zeroVar) {
		mLastRdDataTick = HAL_GetTick();
		mLastTryRdTick = HAL_GetTick();
		filterLight.init(VEML_FILTR_FACTOR);
		mDataValid = false;

	}
}

uint16_t Veml6030Dev::Swap(uint16_t w) {
	uint16_t w1 = (w >> 8) | (w << 8);
	return w1;
}

HAL_StatusTypeDef Veml6030Dev::rdAlsReg(uint16_t *w) {
	uint16_t v;
	HAL_StatusTypeDef st = readWord(cmdALS, &v);
	if (st == HAL_OK) {
		*w = Swap(v);
	}
	return st;
}

HAL_StatusTypeDef Veml6030Dev::rdWhiteReg(uint16_t *w) {
	uint16_t v;
	HAL_StatusTypeDef st = readWord(cmdWHITE, &v);
	if (st == HAL_OK) {
		*w = Swap(v);
	}
	return st;

}

HAL_StatusTypeDef Veml6030Dev::readData() {

	uint16_t als;
	HAL_StatusTypeDef st1 = rdAlsReg(&als);
	float lux = calculateLux(als);
	filterLight.inp(lux);
	return st1;
}

void Veml6030Dev::showHdMeas(OutStream *strm) {

	uint16_t als, white;
	HAL_StatusTypeDef st = rdAlsReg(&als);

	if (st == HAL_OK)
		strm->oMsg("Reg_ALS  =0x%04X (%.1f%%) %.2f[lux]", als, 100.0 * als / 65536.0, calculateLux(als));

	st = rdWhiteReg(&white);
	if (st == HAL_OK)
		strm->oMsg("Reg_WHITE=0x%04X (%.1f%%) %.2f[lux]", white, 100.0 * white / 65536.0, calculateLux(white));
}

const float TabCoeffTm800[] = { .0036, .0072, .0288, .0576 };
const float TabCoeffTm400[] = { .0072, .0144, .0576, .1152 };
const float TabCoeffTm200[] = { .0144, .0288, .1152, .2304 };
const float TabCoeffTm100[] = { .0288, .0576, .2304, .4608 };
const float TabCoeffTm050[] = { .0576, .1152, .4608, .9216 };
const float TabCoeffTm025[] = { .1152, .2304, .9216, 1.8432 };

bool Veml6030Dev::getLuxCoeff(float *coeff) {

	uint8_t idx;
	switch (mCurrent.gain) {
	case GAIN_x1:
		idx = 1;
		break;
	case GAIN_x2:
		idx = 0;
		break;
	case GAIN_x1_8:
		idx = 2;
		break;
	case GAIN_x1_4:
		idx = 3;
		break;
	default:
		return false;
	}

	float luxCoeff;
	switch (mCurrent.integrTm) {
	case INT_TM_100:
		luxCoeff = TabCoeffTm100[idx];
		break;
	case INT_TM_200:
		luxCoeff = TabCoeffTm200[idx];
		break;
	case INT_TM_400:
		luxCoeff = TabCoeffTm400[idx];
		break;
	case INT_TM_800:
		luxCoeff = TabCoeffTm800[idx];
		break;
	case INT_TM_025:
		luxCoeff = TabCoeffTm025[idx];
		break;
	case INT_TM_050:
		luxCoeff = TabCoeffTm050[idx];
		break;
	default:
		return false;
	}
	*coeff = luxCoeff;
	return true;

}

float Veml6030Dev::luxCompensation(float val) {

	// Polynomial is pulled from pg 10 of the datasheet.
	float luxC = (.00000000000060135 * (pow(val, 4)));
	luxC -= (.0000000093924 * (pow(val, 3)));
	luxC += (.000081488 * (pow(val, 2)));
	luxC += (1.0023 * val);
	return luxC;

}

float Veml6030Dev::calculateLux(uint16_t lightBits) {
	float coeff;
	if (getLuxCoeff(&coeff)) {
		float luxVal = lightBits * coeff;

		if (luxVal > 1000) {
			float compLux = luxCompensation(luxVal);
			return compLux;
		} else
			return luxVal;

	} else
		return NAN;
}

uint16_t Veml6030Dev::calculateBits(float luxVal) {
	float coeff;
	if (getLuxCoeff(&coeff)) {
		float w = luxVal / coeff;
		if (w > 65535)
			w = 65535;
		return w;
	} else
		return 0;
}

bool Veml6030Dev::isRdAuto() {
	return mReadAuto;
}

void Veml6030Dev::tick() {
	static int measCnt = 0;
	if (isRdAuto()) {
		uint32_t tt = HAL_GetTick();

		if (tt - mLastRdDataTick > TIME_DT_VALID) {
			// gdy brak odczytów to powtórna inicjalizacja
			init(true);
		}
		else if (tt - mLastRdDataTick > TIME_DT_RD) {
			if (tt - mLastTryRdTick > 500) {
				mLastTryRdTick = tt;
				if (readData() == HAL_OK) {
					mLastRdDataTick = tt;
					mDataValid = true;
				}
			}
		}

	}
	if (mRdTestCount) {
		uint32_t tt = HAL_GetTick();
		if (tt - mLastTryRdTick > 1000) {
			mLastTryRdTick = tt;
			OutStream *strm = getOutStream();
			if (strm->oOpen(colYELLOW)) {
				strm->oMsg("------ %u ---", measCnt++);
				strm->oSetColor(colWHITE);
				showHdMeas(strm);
				strm->oClose();
			}

		}

	}

}

const ShellItem menuVeml[] = { //
		{ "VEML_status", "VEML status" }, //
				{ "VEML_start", "VEML start measure" }, //
				{ "VEML_stop", "VEML stop" }, //
				{ "VEML_meas", "VEML measure" }, //
				{ "VEML_cont", "VEML cotinuos meas" }, //
				{ "VEML_set_limit_L", "" }, //
				{ "VEML_set_limit_H", "" }, //
				{ NULL, NULL } };

const ShellItem* Veml6030Dev::getShellMenu() {
	return menuVeml;
}

bool Veml6030Dev::execMenuItem(OutStream *strm, int idx, const char *cmd) {
	idx -= mMenuOffset;
	switch (idx) {
	case 0: //VEML_status
		if (strm->oOpen(colWHITE)) {
			strm->oMsg("Exist=%s", YN(mDevExist));
			if (mDevExist) {
				strm->oMsg("ChipIdOK=%s", OkErr(chipIdOk));

				uint16_t als, white, als_int, als_conf, als_wl, als_wh, pwr;
				HAL_StatusTypeDef st;

				st = readWord(cmdALS_CONF, &als_conf);
				if (st == HAL_OK)
					strm->oMsg("Reg_ALS_CONF=0x%04X", als_conf);

				st = readWord(cmdPOWER_SAVE, &pwr);
				if (st == HAL_OK)
					strm->oMsg("Reg_PWR_SAVE=0x%04X", pwr);

				st = readWord(cmdALS_WL, &als_wl);
				if (st == HAL_OK)
					strm->oMsg("Reg_ALS_WL  =%u", als_wl, als_wl);
				st = readWord(cmdALS_WH, &als_wh);
				if (st == HAL_OK)
					strm->oMsg("Reg_ALS_WH  =%u", als_wh, als_wh);

				st = readWord(cmdALS_INT, &als_int);
				if (st == HAL_OK)
					strm->oMsg("Reg_ALS_INT =0x%04X", als_int);

				st = rdAlsReg(&als);
				if (st == HAL_OK)
					strm->oMsg("Reg_ALS     =0x%04X (%.1f%%)", als, 100.0 * als / 65536.0);
				st = rdWhiteReg(&white);
				if (st == HAL_OK)
					strm->oMsg("Reg_WHITE   =0x%04X (%.1f%%)", white, 100.0 * white / 65536.0);

			} else {
				strm->oMsg("Dev no exists!");
			}

			strm->oClose();
		}
		break;
	case 1: { //VEML_start
		uint16_t w = getConfRegVal();
		HAL_StatusTypeDef st = writeWord(cmdALS_CONF, w);
		strm->oMsgX(colWHITE, "write ALS_CONF, st=%d", st);
		st = writeWord(cmdPOWER_SAVE, 0x03);

	}
		break;

	case 2: { //VEML_stop
		uint16_t w = getConfRegVal();
		w |= 0x0001;  // wyłaczenie sensora

		HAL_StatusTypeDef st = writeWord(cmdALS_CONF, w);
		strm->oMsgX(colWHITE, "write ALS_CONF, st=%d", st);
	}
		break;

	case 3: //VEML_meas
		showHdMeas(strm);
		break;

	case 4: //VEML_cont
		bool q;
		if (Token::getAsBool(&cmd, &q)) {
			mRdTestCount = q;
		}
		break;

	case 5:
		//VEML_set_limit_L
	case 6:
		//VEML_set_limit_H
	{
		uint8_t reg;
		if (idx == 5)
			reg = cmdALS_WL;
		else
			reg = cmdALS_WH;

		//	float f;
		int vv;
		//if (Token::getAsFloat(&cmd, &f)) {
		if (Token::getAsInt(&cmd, &vv)) {
			uint16_t v = vv;
			HAL_StatusTypeDef st = writeWord(reg, v);
			strm->oMsgX(colWHITE, "Write limit st=%s", HAL_getErrStr(st));
		}

	}

		break;

	default:
		return false;
	}
	return true;
}

#endif
