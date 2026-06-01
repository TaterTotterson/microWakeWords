/*
 * File      : appAudio.c
 * Purpose   : App level audio buffering routines
 * Project   : T2SI "time slice" architecture 
 * Compiler  : PC/embedded platforms
 *
 * Copyright (C) 1995-2022 Sensory Inc., All Rights Reserved                             
 *
 * ******* SENSORY CONFIDENTIAL *************
 *
 *****************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sensorylib.h"

#include "asr.h"

BOOL AppAudioFrameReady(appStruct_T *ap)
{
    AUDIOINDEX scnt = ap->audioPutIndex - ap->audioGetIndex;
    if (scnt < 0) scnt += ap->audioBufferLen;
    return scnt >= FRAME_LEN ? 1 : 0;
}

AUDIOINDEX AppAudioNumUnprocessedSamples(appStruct_T *ap)
{
    AUDIOINDEX scnt = ap->audioPutIndex - ap->audioGetIndex;
    if (scnt < 0) scnt += ap->audioBufferLen;
    return scnt;
}

__attribute__((weak))
BOOL AppAudioLPSDDecreasePowerMode(void)
{
    //Not running recognizer. Can lower clock frequency.
    asr_printf("ASR: LP\n");
    return 0;
}

__attribute__((weak))
BOOL AppAudioLPSDIncreasePowerMode(void)
{
    //Will start to run recognizer. Need to increase clock frequency.
    asr_printf("ASR: FP\n");
    return 0;
    //0=will not skip any processing
    //1=will skip extra processing for this frame (need an extra frame to start frontend processing)
}

// NB: this code currently assumes the audio buffer is an integral multiple of FRAME_LEN.
// If audioBufferLen is zero,  then audioBufferStart will point to the start of the brick.

BOOL AppAudioGetFrame(appStruct_T *ap, s16 startOffset, s16 count, SAMPLE *dst)
{
    if (count) {
        AUDIOINDEX scnt; //, gindex;
        
        scnt = ap->audioPutIndex - ap->audioGetIndex;
        if (scnt < 0) scnt += ap->audioBufferLen;

        if (scnt < FRAME_LEN)  
            return 0;  

        memcpy(dst, ap->audioBufferStart+ap->audioGetIndex+startOffset, count*sizeof(s16));

	}

    ap->audioGetIndex += FRAME_LEN;
    if (ap->audioGetIndex >= ap->audioBufferLen)
        ap->audioGetIndex = 0;

    ap->audioGetFrameCounter++;  
    return 1;
}

// Convert input audio of type BRICK_SAMPLE to brickSize x samples of type SAMPLE (s16).

// Appropriate activity at this point may include:
// DC block, scaling or agc, resizing to s16 from s24 or larger, downsampling to 16KHz etc.

// NB: this code currently assumes that the audio buffer is an integral multiple of brickSize samples.

void AppAudioStoreBrick(appStruct_T *ap, BRICK_SAMPLE *brick, s16 brickSize) 
{
#ifdef USE_SENSORY_DCBLOCK
    if (ap->audioBufferLen == 0) {
        SensoryDcBlock(ap, (SAMPLE *)brick, (SAMPLE *)brick, brickSize);
        ap->audioBufferStart = brick;
        ap->audioGetIndex = 0;
        ap->audioPutIndex = ap->audioFilledCount = brickSize;
        return;
    }

    SensoryDcBlock(ap, (SAMPLE *)brick, ap->audioBufferStart+ap->audioPutIndex, brickSize);
#else
    if (ap->audioBufferLen == 0) {
        ap->audioBufferStart = brick;
        ap->audioGetIndex = 0;
        ap->audioPutIndex = ap->audioFilledCount = brickSize;
        return;
    }
    else
		memcpy(ap->audioBufferStart+ap->audioPutIndex, brick, brickSize*sizeof(s16));
#endif
    ap->audioPutIndex += brickSize;
    if (ap->audioPutIndex >= ap->audioBufferLen)  
        ap->audioPutIndex = 0; 

    ap->audioFilledCount += brickSize;
    if (ap->audioFilledCount > ap->audioBufferLen)
        ap->audioFilledCount = ap->audioBufferLen;
}

// Sensory logging feature
#ifdef SENSORY_LOGGING
/******************************************************************************
* Function Name : SensoryLogInit
* Parameters    : 
* log:			: Pass the logging buffer of data type of SENSORY_LOG_OBJ 
* n				: Size of the logging buffer
* Comment       : Called once to initialize logging variables
******************************************************************************/
void SensoryLogInit(appStruct_T *app, SENSORY_LOG_OBJ *log, int n)
{
    t2siStruct *t = &app->_t;
    t->sensoryLogIndex = 0;
    t->sensoryNumLogItems = n;
    t->sensoryLogStart = log;
    t->sensoryLogFull = 0;
}

/******************************************************************************
* Function Name : SensoryLog
* Parameters    : 
* event:		: Underlying logging event.
* a, b			: Underlying logging event's two arguments
* Comment       : Save the underlying logging function calls into the logging buffer
******************************************************************************/
void SensoryLog(t2siStruct *t, int event, int a, int b)
{
    if (!t->sensoryLogStart) return;

    if (t->sensoryLogIndex < t->sensoryNumLogItems) 
    {
        int sensoryLogIndex = t->sensoryLogIndex;
        SENSORY_LOG_OBJ *sensoryLogStart = t->sensoryLogStart;
        
        sensoryLogStart[sensoryLogIndex].event = event;
        sensoryLogStart[sensoryLogIndex].frameCount = 0; //sensoryFrameCounter;
        sensoryLogStart[sensoryLogIndex].a = a;
        sensoryLogStart[sensoryLogIndex].b = b;
        t->sensoryLogIndex++;
    }
    if (t->sensoryLogIndex >= t->sensoryNumLogItems) 
        t->sensoryLogFull = 1;
    
}

#define NO_LOGGING_PRINT
#ifdef NO_LOGGING_PRINT
void SensoryPrintLog(appStruct_T *app){};
#else
void SensoryPrintLog(appStruct_T *app)
{
    t2siStruct *t = &app->_t;
    int sensoryLogIndex = t->sensoryLogIndex;
    int i;
    SENSORY_LOG_OBJ *sensoryLogStart = t->sensoryLogStart;
    
    printf("\nSensory Log\n");
    if (t->sensoryLogFull) printf("**full**\n");
    for (i=0; i < sensoryLogIndex; i++) {
        //printf("frame %6d %3d 0x%06x 0x%06x ", 
        printf("%6d %3d %6d %6d ",
                sensoryLogStart[i].frameCount, \
                sensoryLogStart[i].event,\
                sensoryLogStart[i].a, sensoryLogStart[i].b);

        switch (sensoryLogStart[i].event) {
			case SENSORY_LOG_RESET:  printf("RESET"); break;
			case SENSORY_LOG_INIT: printf("INIT result = %d", sensoryLogStart[i].a); break;
			case SENSORY_LOG_AUDIO_OVERFLOW: printf("AUDIO OVERFLOW"); break;
			case SENSORY_LOG_OUT_OF_MEMORY: printf("OUT OF MEMORY"); break;
			case SENSORY_LOG_LPSD_ABOVE_T:  printf("LPSD +++ getting data from frame %d",
												sensoryLogStart[i].frameCount - sensoryLogStart[i].a);
			break;
			case SENSORY_LOG_LPSD_BELOW_T:  printf("LPSD --- getting data from frame %d",
												sensoryLogStart[i].frameCount - sensoryLogStart[i].a);
			break;
			case SENSORY_LOG_WS_PASS: printf("  WS pass word %d score %d", sensoryLogStart[i].a, sensoryLogStart[i].b); 
			break;
			case SENSORY_LOG_DPP: printf("  DPP dur score = %d adjusted score = %d", sensoryLogStart[i].a, sensoryLogStart[i].b); 
			break;
			case SENSORY_LOG_EPQ: printf("  EPQ %s minSNR = %d ", sensoryLogStart[i].a ? "PASS" : "FAIL", sensoryLogStart[i].b); 
			break;
			case SENSORY_LOG_RECOG: printf("*** RECOG word %d  Samsung word %d", sensoryLogStart[i].a, sensoryLogStart[i].b); 
			break;
			case SENSORY_LOG_SCORES: printf("    SCORES finalScore %d  svScore %d", sensoryLogStart[i].a, sensoryLogStart[i].b); 
            break;
		}

		printf("\n");
    }
}
#endif //NO_LOGGING_PRINT
#endif
