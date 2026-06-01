/*
	Copyright (c) 2018 Cyberon Corp.  All right reserved.
	File: Convert2TransferBuffer.c
	Author: Ming Wu
	Date: 2018/05/03
*/
#include "Convert2TransferBuffer.h"


int Convert2TransferBuffer(const uint8_t *lpbyInput, int nInputSize, uint8_t *lpbyOutput, int nOutputSize, int eChecksumMode)
{
	int nIndex = 0;

	if (eChecksumMode == eFourByteDataOneChecksum && nOutputSize >= nInputSize * 5 / 4)
	{
		for (int i = 0; i <= nInputSize - 4; i += 4)
		{
			if (nIndex > nOutputSize - 5)
				break;

			lpbyOutput[nIndex]   = lpbyInput[i];
			lpbyOutput[nIndex+1] = lpbyInput[i+1];
			lpbyOutput[nIndex+2] = lpbyInput[i+2];
			lpbyOutput[nIndex+3] = lpbyInput[i+3];
			lpbyOutput[nIndex+4] = lpbyInput[i] ^ lpbyInput[i+1] ^ lpbyInput[i+2] ^ lpbyInput[i+3] ^ 0xFF;
			nIndex += 5;
		}
	}
	else if (eChecksumMode == eTwoByteDataOneChecksum && nOutputSize >= nInputSize * 3 / 2)
	{
		for (int i = 0; i <= nInputSize - 2; i += 2)
		{
			if (nIndex > nOutputSize - 3)
				break;

			lpbyOutput[nIndex]   = lpbyInput[i];
			lpbyOutput[nIndex+1] = lpbyInput[i+1];
			lpbyOutput[nIndex+2] = lpbyInput[i] ^ lpbyInput[i+1] ^ 0xFF;
			nIndex += 3;
		}
	}
	
	return (int)nIndex;
}
