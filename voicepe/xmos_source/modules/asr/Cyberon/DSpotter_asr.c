// Copyright (c) 2022-2023 XMOS LIMITED. This Software is subject to the terms of the
// XMOS Public License: Version 1
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xcore/assert.h>
#include <platform.h>
#include "platform/driver_instances.h"

#define DBG_TRACE                 // Output log by DbgTrace(), if not defined only output by rtos_printf.
//#define UART_DUMP_RECORD        // Send the audio input data and chechsum to UART.
//#define SKIP_DSPOTTER_RECOG

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"

#include "app_conf.h"
#include "asr.h"
#include "rtos_swmem.h"

#include "DSpotterHL.h"
#include "DSpotterSDKApi_Const.h"
#include "DbgTrace.h"
#include "Convert2TransferBuffer.h"
#include "FlashReadData.h"

#define MAX_COMMAND_TIME         (5000/10)               // Trigger and command must be spoke in 5000 ms (500 frames).
#define DSPOTTER_FRAME_SAMPLE    480                     // DSpotter compute every 30 ms, it is 480 samples or 16 KHz.
#define COMMAND_STAGE_TIMEOUT    appconfINTENT_RESET_DELAY_MS  // When no result at command recognition stage, the minimum recording time in ms.
#define VOLUME_SCALE_RECONG      800                     // The AGC volume scale percentage for recognition. It depends on original microphone data.

static uint8_t *g_lpbyDSpotterMem = NULL;
static size_t g_nRecordFrameCount = 0;
devmem_manager_t *devmem_ctx = NULL;


//https://www.xmos.ai/documentation/XM-014363-PC-4/html/prog-guide/prog-ref/xcc-pragma-directives/pragmas.html
//#pragma stackfunction n. This pragma allocates n words ( int s) of stack space for the next function declaration in the current translation unit.
//pragma stackfunction 1500 => Stack size is 1500*sizeof(int) = 6000
#pragma stackfunction 1500
asr_port_t asr_init(int32_t *model, int32_t *grammar, devmem_manager_t *devmem)
{
    DSpotterInitData oDSpotterInitData;
    char szCommand[64];
    int nCmdID;
    int nCount;
    int nMemSize;
    int nRet;

    DBG_TRACE("App build at %s, %s\r\n", __DATE__, __TIME__);
    DBG_TRACE("DSpotter version %s\r\n", DSpotterHL_GetVer());

#ifdef UART_DUMP_RECORD
    DBG_TRACE("It will send record data to UART. The format is four bytes audio data and one byte checksum.\r\n");
    DBG_TRACE("\r\n");
#endif

    devmem_ctx = devmem;

    oDSpotterInitData.nInitDataVer = DSPOTTER_INIT_DATA_VER;
    oDSpotterInitData.nInitDataSize = (uint8_t)sizeof(DSpotterInitData);
    oDSpotterInitData.nMaxCommandTime = MAX_COMMAND_TIME;
    oDSpotterInitData.nCommandStageTimeout = COMMAND_STAGE_TIMEOUT;
    oDSpotterInitData.nCommandStageFlowControl = COMMAND_STAGE_MULTI_COMMAND;
    oDSpotterInitData.byAGC_Gain = (uint8_t)(VOLUME_SCALE_RECONG / 100);
    oDSpotterInitData.bOneShotMode = true;
    oDSpotterInitData.bExternalFlashModel = true;

    nMemSize = DSpotterHL_GetMemoryUsage((const uint32_t *)model, &oDSpotterInitData);
    DBG_TRACE("The DSpotter memory usage is %d.\r\n", nMemSize);
    g_lpbyDSpotterMem = devmem_malloc(devmem_ctx, nMemSize);
    if (g_lpbyDSpotterMem == NULL)
    {
        DBG_TRACE("devmem_malloc() fail!\r\n");
        return NULL;
    }

    DBG_TRACE("DSpotterHL_Init\r\n");
    nRet = DSpotterHL_Init((const uint32_t *)model, &oDSpotterInitData, g_lpbyDSpotterMem, nMemSize);
    if (nRet != DSPOTTER_SUCCESS)
    {
        DBG_TRACE("DSpotterHL_Init() fail, error = %d!\r\n", nRet);
        devmem_free(devmem_ctx, g_lpbyDSpotterMem);
        g_lpbyDSpotterMem = NULL;
        return NULL;
    }

    DBG_TRACE("The list of trigger word: \r\n");
    nCount = DSpotterHL_GetDisplayCommandCount(DSPOTTER_HL_TRIGGER_STAGE);
    for (int i = 0; i < nCount; i++)
    {
        DSpotterHL_GetDisplayCommand(DSPOTTER_HL_TRIGGER_STAGE, i, szCommand, sizeof(szCommand), &nCmdID);
        DBG_TRACE("    %s, ID = %d\r\n", szCommand, nCmdID);
    }

    nCount = DSpotterHL_GetDisplayCommandCount(DSPOTTER_HL_COMMAND_STAGE);
    if (nCount > 0)
    {
        DBG_TRACE("The list of command word: \r\n");
        for (int i = 0; i < nCount; i++)
        {
            DSpotterHL_GetDisplayCommand(DSPOTTER_HL_COMMAND_STAGE, i, szCommand, sizeof(szCommand), &nCmdID);
            DBG_TRACE("    %s, ID = %d\r\n", szCommand, nCmdID);
        }
    }
    DBG_TRACE("\r\n");

    return (asr_port_t)100;
}

asr_error_t asr_get_attributes(asr_port_t *ctx, asr_attributes_t *attributes)
{
    return ASR_NOT_SUPPORTED;
}

asr_error_t asr_process(asr_port_t *ctx, int16_t *audio_buf, size_t buf_len)
{
#ifdef UART_DUMP_RECORD
    uint8_t byaTxBuffer[DSPOTTER_FRAME_SAMPLE*sizeof(int16_t)*3/2];
    int nTransferSize;

    if (buf_len < DSPOTTER_FRAME_SAMPLE)
    {
        nTransferSize = Convert2TransferBuffer((const uint8_t*)audio_buf, buf_len*sizeof(int16_t), byaTxBuffer, sizeof(byaTxBuffer), eFourByteDataOneChecksum);
        rtos_uart_tx_write(uart_tx_ctx, byaTxBuffer, (uint32_t)nTransferSize);
    }
#else
    if (++g_nRecordFrameCount % 100 == 0) {
        DBG_TRACE(".");
    }
#endif

#ifdef SKIP_DSPOTTER_RECOG
    return ASR_ERROR;
#endif

    // Uncomment the line below to compute MIPS usage.
    // uint32_t timer_start = get_reference_time();

    int nRet = DSpotterHL_AddSampleNoFlow(audio_buf, buf_len);

    // Uncomment the two lines below to compute MIPS usage.
    // uint32_t timer_end = get_reference_time();
    // asr_printf("DSpotter processing time: %lu (us)\n", (timer_end - timer_start) / 100);

    if (nRet == DSPOTTER_SUCCESS)
    {
        return ASR_OK;
    }
    else if (nRet == DSPOTTER_ERR_Expired)
    {
        DBG_TRACE("DSpotter evaluation expired!\r\n");
        return ASR_EVALUATION_EXPIRED;
    }
    else
    {
        return ASR_ERROR;
    }
}

asr_error_t asr_get_result(asr_port_t *ctx, asr_result_t *result)
{
    char szCommand[64];
    int nCmdID, nCmdScore, nCmdSG, nCmdEnergy;

    if (DSpotterHL_GetRecogResult(&nCmdID, NULL, szCommand, sizeof(szCommand), &nCmdScore, &nCmdSG, &nCmdEnergy, NULL) == DSPOTTER_SUCCESS)
    {
        DBG_TRACE("\r\nGet %s, ID=%d, Score=%d, SG_Diff=%d, Energy=%d\r\n", szCommand, nCmdID, nCmdScore, nCmdSG, nCmdEnergy);
        result->id = nCmdID;

        result->score = nCmdScore;
        result->gscore = nCmdSG;
        // The following result fields are not implemented
        result->start_index = -1;
        result->end_index = -1;
        result->duration = -1;
        return ASR_OK;
    }
    else
    {
        return ASR_ERROR;
    }
}

asr_error_t asr_reset(asr_port_t *ctx)
{
    DSpotterHL_SetRecognitionStage(DSPOTTER_HL_TRIGGER_STAGE + DSPOTTER_HL_COMMAND_STAGE);
    return ASR_OK;
}

asr_error_t asr_release(asr_port_t *ctx)
{
    DSpotterHL_Release();

    if (g_lpbyDSpotterMem != NULL)
    {
        devmem_free(devmem_ctx, g_lpbyDSpotterMem);
        g_lpbyDSpotterMem = NULL;
    }

    return ASR_OK;
}
