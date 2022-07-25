/*
 * Hdw.cpp
 *
 *  Created on: 10 lut 2022
 *      Author: Grzegorz
 */

#include "Hdw.h"
#include "main.h"

void Hdw::ledOK(bool q) {
	q = !q;
	HAL_GPIO_WritePin(LED_OK_GPIO_Port, LED_OK_Pin, (GPIO_PinState) q);
}

void Hdw::ledR(bool q) {
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, (GPIO_PinState) q);
}
void Hdw::ledG(bool q) {
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, (GPIO_PinState) q);
}

void Hdw::ledKey1(bool q) {
	HAL_GPIO_WritePin(LED_KEY1_GPIO_Port, LED_KEY1_Pin, (GPIO_PinState) q);
}

void Hdw::ledKey2(bool q) {
	HAL_GPIO_WritePin(LED_KEY2_GPIO_Port, LED_KEY2_Pin, (GPIO_PinState) q);
}

bool Hdw::rdKey1() {
	return (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET);
}
bool Hdw::rdKey2() {
	return (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET);
}

void Hdw::setRts2(bool q) {
	HAL_GPIO_WritePin(ZAS_UART2_RTS_GPIO_Port, ZAS_UART2_RTS_Pin, (GPIO_PinState) q);
}

void Hdw::setBleRst(bool q) {
	HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, (GPIO_PinState) q);
}
void Hdw::setGpsRst(bool q) {
	HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, (GPIO_PinState) q);
}

void Hdw::setExtRst(bool q) {
	HAL_GPIO_WritePin(Z1_EXP_RST_GPIO_Port, Z1_EXP_RST_Pin, (GPIO_PinState) q);
}

bool Hdw::rdExpInt1() {
	return (HAL_GPIO_ReadPin(EXP_INT1_GPIO_Port, EXP_INT1_Pin) == GPIO_PIN_SET);
}
bool Hdw::rdExpInt2() {
	return (HAL_GPIO_ReadPin(EXP_INT2_GPIO_Port, EXP_INT2_Pin) == GPIO_PIN_SET);
}

bool Hdw::rdGpsPPS() {
	return (HAL_GPIO_ReadPin(GPS_PPS_GPIO_Port, GPS_PPS_Pin) == GPIO_PIN_SET);
}
