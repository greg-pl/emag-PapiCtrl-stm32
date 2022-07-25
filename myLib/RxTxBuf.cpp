/*
 * RxTxBuf.cpp
 *
 *  Created on: Jun 14, 2021
 *      Author: Grzegorz
 */

#include <stdlib.h>
#include <string.h>

#include <RxTxBuf.h>

RxTxBuf::RxTxBuf(int size) {
	mSize = size;
	mBuf = (char*) malloc(mSize);
	clear();
}

void RxTxBuf::clear() {
	mHead = 0;
	mTail = 0;
	mLockRegionTail = 0;
	mLockRegionFlag =false;

	memset(mBuf, 0, mSize);
}

int RxTxBuf::getFree() {
	int ocu = mHead - mTail;
	if (ocu < 0)
		ocu += mSize;
	return mSize - 1 - ocu;
}

bool RxTxBuf::add(char ch) {
	int h = mHead;

	if (++h >= mSize)
		h = 0;
	if (h == mTail)
		return false;
	mBuf[mHead] = ch;
	mHead = h;
	return true;
}

bool RxTxBuf::pop(char *ch) {
	if (mHead != mTail) {
		*ch = mBuf[mTail];
		if (++mTail == mSize) {
			mTail = 0;
		}
		return true;
	}
	return false;
}

//funkcja przesuwa wskaźnik: mHead
bool RxTxBuf::addBuf(Portion *portion) {
	int fr = getFree();
	if (fr == 0) {
		//nic nie udało sie odłożyć, porcja bez zmian
		return false;
	}

	int lenP = portion->len;
	if (lenP > fr)
		lenP = fr;

	int len = lenP;
	int n1 = mSize - mHead;
	int len1 = len;
	if (len1 > n1) {
		len1 = n1;
	}
	const char *dt = portion->dt;
	int h = mHead;
	memcpy(&mBuf[h], dt, len1);
	h += len1;
	if (h >= mSize)
		h = 0;
	len -= len1;
	if (len != 0) {
		dt += len1;
		memcpy(&mBuf[h], dt, len);
		h += len;
	}
	mHead = h;
	portion->len -= lenP;
	(portion->dt) += lenP;

	return (portion->len == 0);
}

bool RxTxBuf::lockLinearRegion(const char **pptr, int *cnt, int max) {
	if (mTail == mHead)
		return false;
	int hh = mHead;
	int tt = mTail;

	int n;
	if (hh > tt) {
		n = hh - tt;
	} else {
		n = mSize - tt;
	}
	if (n > max)
		n = max;
	int nTT = tt + n;
	if (nTT == mSize)
		nTT = 0;

	*pptr = &mBuf[tt];
	*cnt = n;
	mLockRegionTail = nTT;
	mLockRegionFlag = true;
	return true;
}
void RxTxBuf::unlockRegion() {
	mLockRegionFlag = false;
	mTail = mLockRegionTail;

}

bool RxTxBuf::readLn(char *buf, int max) {
	int t = mTail;
	int h = mHead;
	int k = 0;
	while (t != h) {
		char ch = mBuf[t];
		if (++t >= mSize)
			t = 0;

		if (ch == '\n') {
			mTail = t;
			buf[k++] = ch;
			buf[k++] = 0;
			return true;
		}
		if (k < max - 1) {
			buf[k++] = ch;
		}
	}
	return false;
}
