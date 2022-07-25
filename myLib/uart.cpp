#include "_ansi.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdarg.h"

#include "main.h"
#include "uart.h"

//-----------------------------------------------------------------------------
// TUart
//-----------------------------------------------------------------------------

//PortNr: 1...N
const USART_TypeDef *const USART_TAB[TUart::PORT_CNT] = {
USART1, USART2, USART3, UART4, UART5 };
const IRQn_Type IRQ_NR_TAB[TUart::PORT_CNT] = { USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn };

TUart *TUart::UartVec[TUart::PORT_CNT] = {
NULL, NULL, NULL, NULL, NULL, NULL };

TUart::TUart(int PortNr, int Priority) {

	mPortNr = PortNr;
	if (mPortNr >= 0 && mPortNr < PORT_CNT) {
		UartVec[PortNr] = this;
		memset(&mHuart, 0, sizeof(mHuart));

		mPriority = Priority;
		mFramingErrCnt = 0;
		mIrqCnt = 0;
		txSending = false;
		mUseRts = false;
		mHdwCtrl = false;
		mHalfDuplex = false;

	} else {
#ifdef  USE_FULL_ASSERT
		assert_failed((byte*) 0, __LINE__);
#endif
	}
}

TUart::~TUart(void) {
	UartVec[mPortNr] = NULL;
	return;
}

void TUart::TxCpltCallback() {
	txSending = false;
}

void TUart::writeBuf(const void *dt, int len) {
	if (len > 0) {
		txSending = true;
		HAL_UART_Transmit_IT(&mHuart, (uint8_t*) dt, len);
	}
}

void TUart::readBuf(void *dt, int len) {
	HAL_UART_Receive_IT(&mHuart, (uint8_t*) dt, len);
}

void TUart::SetBaudRate(uint32_t baudRate) {
	HAL_UART_DeInit(&mHuart);
	Init(baudRate);
}

HAL_StatusTypeDef TUart::Init(int BaudRate) {
	return Init(BaudRate, parityNONE);
}

HAL_StatusTypeDef TUart::Init(int BaudRate, int parity) {

	mHuart.Instance = (USART_TypeDef*) USART_TAB[mPortNr];
	mHuart.Init.BaudRate = BaudRate;
	mHuart.Init.StopBits = UART_STOPBITS_1;
	switch (parity) {
	default:
	case parityNONE:
		mHuart.Init.Parity = UART_PARITY_NONE;
		mHuart.Init.WordLength = UART_WORDLENGTH_8B;
		break;
	case parityEVEN:
		mHuart.Init.Parity = UART_PARITY_EVEN;
		mHuart.Init.WordLength = UART_WORDLENGTH_9B;
		break;
	case parityODD:
		mHuart.Init.Parity = UART_PARITY_ODD;
		mHuart.Init.WordLength = UART_WORDLENGTH_9B;
		break;
	}
	mHuart.Init.Mode = UART_MODE_TX_RX;
	mHuart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	mHuart.Init.OverSampling = UART_OVERSAMPLING_16;

	mHuart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	if (mUseRts)
		mHuart.Init.HwFlowCtl = UART_HWCONTROL_RTS;
	if (mHdwCtrl)
		mHuart.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;

	HAL_StatusTypeDef st;
	if (!mHalfDuplex)
		st = HAL_UART_Init(&mHuart);
	else
		st = HAL_HalfDuplex_Init(&mHuart);

	// interrupt Init
	if (st == HAL_OK) {
		IRQn_Type irqNr = IRQ_NR_TAB[mPortNr];
		HAL_NVIC_SetPriority(irqNr, mPriority, 0);
		HAL_NVIC_EnableIRQ(irqNr);
	}
	return st;

//	TxQuee->Clear();
//	RxQuee->Clear();
//HAL_UART_Receive_IT(&mHuart, &rxRec.mRecByte, 1);
}

void TUart::Tick1msEntry() {
	for (int i = 0; i < PORT_CNT; i++) {
		if (UartVec[i] != NULL) {
			UartVec[i]->Tick1ms();
		}
	}
}

void TUart::TxCpltCallback(UART_HandleTypeDef *huart) {
	for (int i = 0; i < PORT_CNT; i++) {
		if (UartVec[i] != NULL) {
			if (&(UartVec[i]->mHuart) == huart) {
				UartVec[i]->TxCpltCallback();
				break;
			}
		}
	}
}

void TUart::RxCpltCallback(UART_HandleTypeDef *huart) {
	for (int i = 0; i < PORT_CNT; i++) {
		if (UartVec[i] != NULL) {
			if (&(UartVec[i]->mHuart) == huart) {
				UartVec[i]->RxCpltCallback();
				break;
			}
		}
	}
}

void TUart::ErrorCallback(UART_HandleTypeDef *huart) {
	for (int i = 0; i < PORT_CNT; i++) {
		if (UartVec[i] != NULL) {
			if (&(UartVec[i]->mHuart) == huart) {
				UartVec[i]->ErrorCallback();
				break;
			}
		}
	}
}

void TUart::ISR(int uartNr) {
	TUart *uart = UartVec[uartNr];
	if (uart != NULL) {
		uart->mIrqCnt++;
		HAL_UART_IRQHandler(&uart->mHuart);
	}
}

extern "C" void uartsTick1ms() {
	TUart::Tick1msEntry();
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	TUart::TxCpltCallback(huart);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	TUart::RxCpltCallback(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	TUart::ErrorCallback(huart);
}

extern "C" void USART1_IRQHandler(void) {
	TUart::ISR(TUart::myUART1);
}
extern "C" void USART2_IRQHandler(void) {
	TUart::ISR(TUart::myUART2);
}
extern "C" void USART3_IRQHandler(void) {
	TUart::ISR(TUart::myUART3);
}

extern "C" void UART4_IRQHandler(void) {
	TUart::ISR(TUart::myUART4);
}
extern "C" void UART5_IRQHandler(void) {
	TUart::ISR(TUart::myUART5);
}

