#ifndef __DSPOTTER_HL_H__
#define __DSPOTTER_HL_H__

#include <stdint.h>
#include <stdbool.h>

#define DSPOTTER_INIT_DATA_VER      1

#define DSPOTTER_HL_INIT_STAGE      0
#define DSPOTTER_HL_TRIGGER_STAGE   1
#define DSPOTTER_HL_COMMAND_STAGE   2

#define COMMAND_STAGE_SINGLE_COMMAND  0
#define COMMAND_STAGE_MULTI_COMMAND   1


typedef struct _DSpotterInitData
{
	uint8_t nInitDataVer;
	uint8_t nInitDataSize;
	int16_t nMaxCommandTime;			// Command must be spoke within this time, the unit is 10 ms.
	int16_t nCommandStageTimeout;		// The minimum recording time(unit is ms) when there is no result at command stage.
										// The reasonable range is nMaxCommandTime to 10000 ms.
	uint8_t nCommandStageFlowControl;	// If COMMAND_STAGE_SINGLE_COMMAND, the recognition flow will switch to trigger stage immediately after
										// command recognized. If COMMAND_STAGE_MULTI_COMMAND, it will recognize repeatedly at command stage
										// till to timeout.
	uint8_t byAGC_Gain;					// The scale times of AGC volume, 0 ~ 128.
	bool bOneShotMode;					// If true, there is no need to pause between wake word and command word, but it need more memory.
	bool bExternalFlashModel;			// If true, DSpotter will FlashReadData() API to read model to RAM. It will use more memory for cahce buffer.
	bool byaReservedSetting[6];			// The reserved setting data for future use.
}	DSpotterInitData;

const char* DSpotterHL_GetVer();

/* Get the memory usage of DSpotter.
 *   lpdwModel(in): The voice model.
 *   pDSpotterInitData(in): DSpotter initial data.
 * Return the memory usage. */
int DSpotterHL_GetMemoryUsage(const uint32_t *lpdwModel, const DSpotterInitData *pDSpotterInitData);

/* Initialize high level DSpotter interface. The high level API only support the voice model with 1 or 2 group.
 * It treats the first group as trigger(wake) words and the second group as command words.
 *   lpdwModel(in): The voice model.
 *   pDSpotterInitData(in): DSpotter initial data.
 *   lpbyMemPool(in): The memory buffer that will be used by DSpotter.
 *   nMemSize(in): The memory size.
 * Return DSPOTTER_SUCCESS: Initial OK.
 *        DSPOTTER_ERR_IllegalParam: If pointer parameter is NULL.
 *        DSPOTTER_ERR_LeaveNoMemory: If nMemSize less than the return value of DSpotterHL_GetMemoryUsage().
 *        DSPOTTER_ERR_LoadModelFailed: If lpdwModel is not a valid voice model. */
int DSpotterHL_Init(const uint32_t *lpdwModel, const DSpotterInitData *pDSpotterInitData, unsigned char *lpbyMemPool, int nMemSize);

/* To release DSpotter, then the memory buffer(lpbyMemPool) can be reused by others.
 * Return Success or negative value for error. */
int DSpotterHL_Release();

/* Put the record data to the cached record buffer. Please call this API in record data callback function, or after pre-process output.
 *   lpsSample(in): The record data buffer, it must be 16KHz, 16 bits, mono, PCM format data.
 *   nNumSample(in): The number of samples in the record data buffer. Please do not exceed 480.
 * Return DSPOTTER_SUCCESS: Put OK.
 *        DSPOTTER_ERR_IllegalParam: If DSpotterHL_Init() is not called.
 *        DSPOTTER_ERR_LeaveNoMemory: If the cached record buffer has no enough free space. The lost count will
 *                       increment by 1. If it only appears temporarily, please enlarge the cache 
 *                       length of record data. If it keeps appearing, it is usually because the MCU
 *                       computing resources are exhausted. */
int DSpotterHL_PutRecordData(const short *lpsSample, int nNumSample);

/* Get the lost count when putting record data.
 * Return the lost count. */
int DSpotterHL_GetRecordLostCount();

/* Try to get 30 millisecond data from internal cached record buffer and process it. 
 * This API will switch recognition stage automatically acording to the DSpotterInitData.nCommandStageTimeout and nCommandStageFlowControl.
 * Please call it in the while loop of main function(bare metal) or reognition task(RTOS).
 *   pnCurrentStage(out): The current stage.
 * Return DSPOTTER_SUCCESS: Get the recognition result, please use DSpotterHL_GetRecogResult() to get detail information.
 *        DSPOTTER_ERR_IllegalHandle: If call it before DSpotterHL_Init().
 *        DSPOTTER_ERR_NeedMoreSample: There is no enough data in the cached record buffer, or no recognition result. DSpotter need more sample to process. */
int DSpotterHL_DoRecognition(int *pnCurrentStage);

/* Recognize the record data. This API = DSpotterHL_PutRecordData() + DSpotterHL_DoRecognition().
 * This API will switch recognition stage automatically acording to the DSpotterInitData.nCommandStageTimeout and nCommandStageFlowControl.
 * Please call it in the while loop of main function(bare metal) or reognition task(RTOS).
 *   lpsSample(in): The record data buffer, it must be 16KHz, 16 bits, mono, PCM format data.
 *   nNumSample(in): The number of samples in the record data buffer. Please do not exceed 480.
 *   pnCurrentStage(out): The current stage.
 * Return DSPOTTER_SUCCESS: Get the recognition result, please use DSpotterHL_GetRecogResult() to get detail information.
 *        DSPOTTER_ERR_IllegalHandle: If call it before DSpotterHL_Init().
 *        DSPOTTER_ERR_NeedMoreSample: There is no enough data in the cached record buffer, or no recognition result. DSpotter need more sample to process. */
int DSpotterHL_AddSample(const short *lpsSample, int nNumSample, int *pnCurrentStage);

/* Recognize the record data without switching recognition stage.
 * Please call it in the while loop of main function(bare metal) or reognition task(RTOS).
 *   lpsSample(in): The record data buffer, it must be 16KHz, 16 bits, mono, PCM format data.
 *   nNumSample(in): The number of samples in the record data buffer. Please do not exceed 480.
 * Return DSPOTTER_SUCCESS: Get the recognition result, please use DSpotterHL_GetRecogResult() to get detail information.
 *        DSPOTTER_ERR_IllegalHandle: If call it before DSpotterHL_Init().
 *        DSPOTTER_ERR_NeedMoreSample: There is no enough data in the cached record buffer, or no recognition result. DSpotter need more sample to process. */
int DSpotterHL_AddSampleNoFlow(const short *lpsSample, int nNumSample);

/* Set stage for recognizing the .
 *   nStage(in): DSPOTTER_HL_TRIGGER_STAGE for recognize trigger words only.
                 DSPOTTER_HL_COMMAND_STAGE for recognize command words only.
                 (DSPOTTER_HL_TRIGGER_STAGE + DSPOTTER_HL_COMMAND_STAGE) for recognize both trigger words and command words.
 * Return DSPOTTER_SUCCESS: Get the recognition result, please use DSpotterHL_GetRecogResult() to get detail information.
 *        DSPOTTER_ERR_IllegalHandle: If call it before DSpotterHL_Init().
 *        DSPOTTER_ERR_IllegalParam: . */
int DSpotterHL_SetRecognitionStage(int nStage);

/* Get the information of recognition result.
 *   pnCmdID(out): The ID of command. The ID is assigned by DSMT.
 *   pnCmdIndex(out): The index of command. It is same as the index of DSMT command group list.
 *   lpszCommand(out): The command buffer.
 *   nCmdLength(in): The length of command buffer.
 *   pnCmdScore(out): The confidence score of command.
 *   pnCmdSG(out): The human voice similarity of command.
 *   pnCmdEnergy(out): The energy of command.
 *   pnCurrentStage(out): The current stage.
 * Return Success or negative value for error. */
int DSpotterHL_GetRecogResult(int *pnCmdID, int *pnCmdIndex, char *lpszCommand, int nCmdLength, int *pnCmdScore, int *pnCmdSG, int *pnCmdEnergy, int *pnCurrentStage);

/* Get the count of command at different stage for display. 
 * The display commands don't include multi-pronunciation commands and garbage commands.
 *   nStage(in): DSPOTTER_HL_TRIGGER_STAGE or DSPOTTER_HL_COMMAND_STAGE.
 * Return the command count or display.  */
int DSpotterHL_GetDisplayCommandCount(int nStage);

/* Get the information of command for display.
 *   nStage(in): DSPOTTER_HL_TRIGGER_STAGE or DSPOTTER_HL_COMMAND_STAGE.
 *   nDisplayIndex(in): From 0 to DSpotterHL_GetDisplayCommandCount(nStage) - 1.
 *   lpszCommand(out): The command buffer.
 *   nCmdLength(in): The length of command buffer.
 *   pnID(out): The ID of command.
 * Return Success or negative value for error. */
int DSpotterHL_GetDisplayCommand(int nStage, int nDisplayIndex, char *lpszCommand, int nCmdLength, int *pnID);

#endif