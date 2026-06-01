/*
	Copyright (c) 2018 Cyberon Corp.  All right reserved.
	File: Convert2TransferBuffer.h
	Author: Ming Wu
	Date: 2018/05/03
*/

#ifndef _CONVERT_TO_TRANSFER_BUFFER_H_
#define _CONVERT_TO_TRANSFER_BUFFER_H_

#include <stdint.h>

#define	eTwoByteDataOneChecksum   0
#define	eFourByteDataOneChecksum  1

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Convert recording data array to UART transfer data array.
 *
 * lpbyInput [in]
 * 		The recording data array.
 * nInputSize [in]
 * 		The size of recording data array.
 * lpbyOutput [out]
 * 		The output data array for UART transfer.
 * nOutputSize [in]
 * 		The size of output data array. It shall be great then (nInputSize * 3 / 2) for eTwoByteDataOneChecksum,
 * 		or (nInputSize * 5 / 4) for eFourByteDateOneChecksum.
 * Return
 * 		The valid converted data size in lpbyOutput, or negative value for error.
 */
int Convert2TransferBuffer(const uint8_t *lpbyInput, int nInputSize, uint8_t *lpbyOutput, int nOutputSize, int eChecksumMode);

#ifdef __cplusplus
}
#endif

#endif
