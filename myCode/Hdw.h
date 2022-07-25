/*
 * Hdw.h
 *
 *  Created on: 10 lut 2022
 *      Author: Grzegorz
 */

#ifndef HDW_H_
#define HDW_H_

class Hdw {
public:
	static void ledOK(bool q);
	static void ledR(bool q);
	static void ledG(bool q);
	static void ledKey1(bool q);
	static void ledKey2(bool q);
	static bool rdKey1();
	static bool rdKey2();
	static void setRts2(bool q);
	static void setBleRst(bool q);
	static void setGpsRst(bool q);
	static void setExtRst(bool q);
	static bool rdExpInt1();
	static bool rdExpInt2();
	static bool rdGpsPPS();


};

#endif /* HDW_H_ */
