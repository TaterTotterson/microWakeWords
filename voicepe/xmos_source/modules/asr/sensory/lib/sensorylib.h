 /*
 * File      : sensorylib.h
 * Purpose   : Generic functional and structural interface template
 * Project   : T2SI/THF on deeply-embedded DSPs
 * Compiler  : 
 *
 * Copyright (C) 1995-2022 Sensory Inc., All Rights Reserved
 *
 * ******* SENSORY CONFIDENTIAL *************
 *
 *****************************************************************************
 *
 * This is the standard API header for Sensory's tslice technology running
 * in a typical deeply-embedded DSP. It may be used as an implementation,
 * or as the starting point for further discussion.
 * This represents the face of Sensory's code exposed to the firmware (FW).
 *
 * Adapt this as required for the target platform
 *
*/

#ifndef SENSORYLIB_H_INCLUDED
#define SENSORYLIB_H_INCLUDED

#include <stdlib.h>
#include <stdio.h>

/*------------------  overview  ----------------*
 *
 * The intended target is a DSP that makes frequent calls to a "decoder" 
 * with small "bricks" of audio samples. A "brick" may contain as 
 * few as 16 samples (1 msec). 
 *
 *-------------- definitions -------------------*/

// platform specific items follow. Enable by defining one and only one  
// platform (such as PC, VPC) -- preferably by a compiler option in the .IDE
// This file should include typedefs for u32, s16, etc used in the structure definitions.
#include "sensorytypes.h"

/* there needs to be a documented error enumeration. Sensory code will return
 * errors in the format needed, assumed here "unsigned long". Also see ERROR CODES below */
typedef unsigned short errors_t;

/*---------- application configuration  -------------------------------------------------*/

// t->maxResults and t->maxTokens affect memory allocation, so they
// must be explicitly defined in the t2siStruct before calling SensoryAlloc().

#define MAX_RESULTS           6  // rare to need more for triggers or commands
#define MAX_TOKENS          300  // adjust as needed during development (monitor outOfMemory return)

/*
AUDIO_BUFFER_MS sets the size of the buffer. This can be changed at the app level without recompiling 
the Sensory library.

The audio buffer must an integral multiple of 30 msec, which is an integral multiple of all supported
brick sizes.   

The minimum size depends on the application requirements.  The maximum size is determined by the maximum
positive value of integer type AUDIOINDEX.  For example, if AUDIOINDEX is defined as a short integer
(16-bits signed), the maximum size of the audio buffer is 2040 msec or 32640 samples.  This is the
largest positive short integer value for a buffer that is an integral multiple of 30 msec (480 samples).

Without trigger endpoint detection:

    For non-LPSD, the minimum size is generally 30 msec. It can be 0 msec for the case of a 15 msec
    brick size (240 samples), which means that no audio buffer exists.  Warning: In this case the brick
    contents may be modified in AppAudioStoreBrick in some applications (e.g. if USE_SENSORY_DCBLOCK
    is enabled).

    For LPSD, the minimum size should match BACKOFF_MS which is defined in this file as well.   A typical 
    value for BACKOFF_MS (and thus a minimum value for AUDIO_BUFFER_MS) is 270 msec.   Note that the 
    application developer cannot change BACKOFF_MS, it is integrated in the Sensory library.

With trigger endpoint detection:

    For non-LPSD, the size depends on the delay value (from the grammar or t2siStruct delay field)
    and the value of ADDITIONAL_ENDPOINT_BACKOFF_FRAMES in sensorylib.h.   The application developer
    cannot change the latter value because it is integrated into the Sensory library.
    Typically for endpoint detection it is recommended that the delay value of the grammar be
    overridden by setting the delay value of the t2siStruct to be 240 msec.  A typical value for
    the AUDIO_BUFFER_MS in this scenario is 360 msec.  This is suitable for a delay value of 240 msec 
    and ADDITIONAL_ENDPOINT_BACKOFF_FRAMES up to 6.

    For LPSD,  the size of the audio buffer should be the sum of the requirements of LPSD and endpoint
    detection.   A typical value in this scenario is 630 msec which is the sum of 270 msec for LPSD
    and 360 msec for endpoint detection.
 
*/

#define AUDIO_BUFFER_MS   630

/*****************************************************************************************/
/*****************************************************************************************/
/*----- system definitions - customer SHOULD NOT change ANY of the parameters below -----*/
/*****************************************************************************************/
/*****************************************************************************************/

// The "brick" is the chunk of speech provided with each SensoryProcessData call
// Currently supported values: 15 (best, desired), 10, 5, 3, 2, 1 
// NOTE: BRICK_SIZE_MS must be consistent with the value in sensorylib.h used with the Sensory Library 

#define BRICK_SIZE_MS		(15) // 15 milliseconds = 240 samples

#define FRAME_LEN           240  // frame length in samples
#define FRAME_LEN_MS         15  // frame length in msec 
#define BRICK_SIZE_SAMPLES (BRICK_SIZE_MS*16)

#define BACKOFF_MS          270 // backoff in msec -- must be integral multiple of 30
#define NUM_BACKOFF_FRAMES  (((BACKOFF_MS+29)/30)*2) // 360msec = 24 frames 

#define NUM_AUDIO_BUFFER_FRAMES   (((AUDIO_BUFFER_MS+29)/30)*2) 
#define AUDIO_BUFFER_LEN (NUM_AUDIO_BUFFER_FRAMES*FRAME_LEN)

#define ADDITIONAL_ENDPOINT_BACKOFF_FRAMES 5    // 0 seems to work best to find the endpoint that does not include
                                                // tail end of the trigger (may require longer delay value).
                                                // 5 may be better if the command set includes initial phoneme(s) 
                                                // that are similar to the final phoneme of the trigger
                                                // used in SensoryFindEndpoin()

/* size definitions of non-malloc'd, non-SensoryPersistent objects */
#define STACK_SIZE          400 // for example (not needed by all platforms) 

/*---------------- default t2siStruct parameters (usually suitable) ---------------------*/
#define T2SI_DEFAULT_SEP_MS     400 
#define T2SI_MAX_SEP_MS         900      
#define T2SI_MIN_SEP_MS         100     
#define T2SI_DEFAULT_TIMEOUT_COMMAND 3    // in seconds. NOTE: default for trigger = 0

#define T2SI_MIN_KNOB           1
#define T2SI_DEFAULT_KNOB       3         
#define T2SI_MAX_KNOB           5

enum {SDET_NONE, SDET_SDET, SDET_LPSD, SDET_USE_GRAMMAR, SDET_EXTERNAL_LPSD};       // types of speech detectors to use
enum {SDET_LPSD_SILENCE, SDET_MAKING_BLOCKS, SDET_RECOGNIZING}; // states for the speech detectors 

/*-------------- structure definitions --------------*/

/* -----Sensory standard results structure ----------*/
// result_t must be defined, but may be UNused in some deeply-embedded DSPs
typedef struct {
  u16 wordID;           // word recognized (0 = SIL for T2SI)
  u16 inVocab;          // 1 if not NOTA or SIL 
  u16 isNOTA;
  s16 score;            // segmental score, may be negative for WS
  u16 gscore;           // garbage score, can be used to compute duration 
} result_t;

#ifdef SENSORY_LOGGING
typedef struct {
int event;
int frameCount;
int a;
int b;
} SENSORY_LOG_OBJ;
#endif

/*----- Sensory standard application structure ------*/
/* This public structure is required by Sensory technology
 * Many inputs (I) can be set to 0, which will force the use of default values,
 * or the application can configure as required
 */

typedef struct {
  intptr_t net;         // (I) address of the net
  intptr_t gram;        // (I) address of the grammar
  intptr_t reserved;    // (I) reserved for use by Sensory. Do not modify.
  u16 sdet_type;        // (I) type of speech detector: see enum above
  u16 timeout;          // (I) time in seconds to listen for speech to start (0=forever for trigger, 3s for commands)
  u16 maxTalkTime;      // (I) maximum talk time in seconds (0=use value in grammar)
  u16 separator;        // (I) trailing silence in milliseconds
  u16 maxResults;       // (I) call with max number of results, at least 1
  u16 maxTokens;        // (I) controls allocation for part of technology memory
  u16 knob;             // (I) 1..5, affects non-spotted command confidence
  s16 paramAOffset;     // (I) matches SDK paramAOffset, centered at 0
						//	   0: use the gram model default setting; otherwise use the new setting
  u16 enableLogging;    // (I) enable runtime logging feature
  u16 keepGoing;        // (I) keepGoing after trigger recognition
  u16 noVerify;         // (I) ignore SG_VERIFY in grammar (treat as UDT)
  u16 lpsdFixedThresh;  // (I) 0: use default, otherwise sets fixed part of threshold
  u16 delay;            // (I) 0: use default delay from grammar, otherwise selects delay in msec
  u16 initFromLast;     // (I) 1: initialize with state at end of last (previous) recognition.
  u16 initLateStart;    // (I) 1: late start initialization  
  u16 epqMinSNR;        // (I) 0: use default grammar value, 0xFFFF means disable, else
                        //     -24..24 db encoded as 1..0xFB30, (u16)(pow(10,minSNR/10)*256)
  s16 SvThreshold;		// (I) 0: will be overwritten with gram model default setting
						//	   Otherwise, keep new setting unchanged
  s16 LPSDLatencyCounter; // (I) reduced LPSD latency counter
  MemcpyFunction devMemCpy;   // (I) A special memory copy function for XCore, can copy from ROM or RAM
  IsFlashFunction devIsFlash; // (I) A special bool function for XCore, detects if address is ROM
  size_t romCacheSize;          // (I) ROM cache size for keeping large ROM elements in RAM persistently
  size_t romCacheUsed;
  u16 wordID;           // (O) wordID of final word spoken
  u16 error;            // (O) see error defs below for possible values
  u16 numResults;       // (O) returns actual number of results
  u16 finalScore;       // (O) these 5 used for debug printout: finalScore is WS score
  u16 garbageScore;     // (O) comparison score used for calculation of finalScore
  u16 maxPowerEnv;      // (O) measure of energy of utterance
  u16 silLevel;         // (O) measure of silence level before utterance
  u16 duration;         // (O) length in 15 msec frames of utterance
  result_t* results;    // (O) array of results, size = maxResults * maxBestResults
  u16 sdet_state;       // (O) state of speech detector: see enums above
  u16 outOfMemory;      // (O) error counter for out of tokens (indicates MAX_TOKENS too small)
  u16 tokensPruned;     // (O) indicates that pruning took place
  u16 maxTokensUsed;    // (O) max tokens used so far
  u32 idleFrames;       // (O) LPSD frames not-in-sound
  u32 streamFrames;     // (O) LPSD frames in-sound
  s16 svScore;          // (O) SV score
  void *spp;            // private technology memory pointer
  u32 size;             // persistent memory size
  u16 recogPending;     // (O) LPSD recog pending
  u16 lpsdHoldoff;      // (O) LPSD holdoff frames
  s16 nnpqPass;			// (O) report nnpq pass
  u16 nnpqThreshold;	// (O) nnpq threshold from model
  int nnpqScore;		// (O) report NNPQ score

#ifdef SENSORY_LOGGING
    SENSORY_LOG_OBJ *sensoryLogStart;
    int sensoryLogIndex;
    int sensoryLogFull;
    int sensoryNumLogItems;
#endif                     
  } t2siStruct;

typedef s16 BRICK_SAMPLE; // type of samples passed to SensoryProcessData by app
typedef s16 SAMPLE;       // type of samples used by sensory. we can only support s16 (short) currently
typedef s24 AUDIOINDEX;   // can be s24 or s32 for larger buffers. must be signed!

typedef struct 
{
u32 reserved0;   // place holder for private variables
s16 reserved1;
s16 reserved2;
s16 reserved3;
s16 reserved4;
s16 reserved5;
s16 reserved6;
s16 reserved7;
s16 reserved8;
s16 reserved9;
s16 reserved10;
s16 reserved11;
s16 reserved12;
s16 reserved13;
s16 reserved14;
s16 reserved15;
} lpsdStruct;

/*---- sample application structures --------*/
/* This structure acts as a wrapper for t2siStruct, allowing additional
 * contents that may be required by the FW.
 * This assumes the net and grammar are determined by the app and supplied
 * inside t2siStruct.
 */
typedef struct appStruct_s appStruct_T;

struct appStruct_s
{  
     /* optional app-specific/FW stuff (buffers, etc) goes here 
     *
     ... (as required by application/FW)
     *
     */

    // audio buffering:
    u32 audioGetFrameCounter;          // number of frames processed by lpsd or SensoryProcessData(), updated
                                       // by AppAudioGetFrame()
    u32 lpsdGetFrameCounter;           
    u16 brickCounter;                  // used by SensoryProcessData() to synchronize between bricks and frames 
    SAMPLE *audioBufferStart;          // pointer to start of audio buffer
    AUDIOINDEX audioBufferLen;         // size of the audio buffer in samples
    AUDIOINDEX volatile audioPutIndex; // buffer put index: volatile in case bricks are added in an ISR.
                                       // updated by AppAudioStoreBrick().
                                       // note: audioPutIndex is always modulus audioBufferLen, ie, 0..audioBufferLen-1.
    AUDIOINDEX audioGetIndex;          // buffer get index: lpsd uses this offset, SensoryProcessData uses
                                       // this offset minus audioBackoffFrames*FRAME_LEN.
                                       // updated by AppAudioGetFrame().
                                       // note: audioGetIndex is always modulus audioBufferLen, ie, 0..audioBufferLen-1.
    AUDIOINDEX audioFilledCount;       // number of samples in audioBuffer (limited to audioBufferLen)

    // Low Power Sound Detect: 
    AUDIOINDEX lpsdGetIndex;           // LPSD pointer into audioBuffer
    u16 doProcessingFlag;              // 1 means processing bricks
    lpsdStruct lpsd;                   // structure of LPSD variables

    s32 dcBlockZ;                      // dc block filter state 
    void *extras;       // (I,O) optional undefined structure to handle any extra application requirements
    t2siStruct _t;      // Sensory structure with application-visible values and pointer to Sensory Persistent Memory
    appStruct_T* featureSource; // Will be set if this recognizer gets features from another
};

/* This optional structure can be used for application needs */
typedef struct infoStruct_S
{
    u32 version;    // for example, version of Sensory technology
}infoStruct_T;


/*-------------- function prototypes: adapt for platform -------------------*/

/******************************************************************************
* Function Name     : SensoryInfo
* Parameters        : isp = pointer to structure in app
* Comment           : Optional for identification, etc. 
*                   : Fills in isp->version
******************************************************************************/
errors_t SensoryInfo(infoStruct_T *isp);

/******************************************************************************
* Function Name     : SensoryAlloc
* Parameters        : asp = pointer to persistent structure common to app and Sensory
*                   : (assumes grammar/net supplied in asp._t t2siStruct as above.)
*                   : size = pointer to int to receive persistent memory size (in malloc units)
* Comment           : Best mechanism for Sensory to obtain its persistent memory
*                   : The FW could call this function, allocate "size" bytes for
*                   : Sensory's persistent memory, then put a pointer to that
*                   : in appStruct._t.spp. 
******************************************************************************/
errors_t SensoryAlloc (appStruct_T *asp, unsigned int *size);

/******************************************************************************
* Function Name     : SensoryProcessInit
* Parameters        : assumes grammar/net supplied in asp._t t2siStruct as above.
* Comment           : Mechanism for Sensory to prepare structures for recognition.
*                   : t2siStruct contains various app-controllable parameters.
******************************************************************************/
errors_t SensoryProcessInit(appStruct_T *asp);

/******************************************************************************
* Function Name     : SensoryProcessRestart
* Parameters        : assumes grammar/net supplied in asp._t t2siStruct as above.
* Comment           : Mechanism for Sensory to restart recognition after success 
*                   : or error.  Faster than full initialization done by 
*                   : SensoryProcessInit()
******************************************************************************/
errors_t SensoryProcessRestart(appStruct_T *asp);

/******************************************************************************
* Function Name : SensoryProcessData
* Parameters    : Expects BRICK_SIZE_MS 16-bit samples at 16kHz in buffer pointed by brick.
* Comment       : The workhorse function, called repeatedly.
*               : Expects to be called every BRICK_SIZE_MS milliseconds.
*               : Should execute (on average) in the time associated with BRICK_SIZE_MS.
*               : Typically returns ERR_NOT_FINISHED, or ERR_OK upon recognition
******************************************************************************/

errors_t SensoryProcessData(BRICK_SAMPLE *brick, appStruct_T *asp);

/******************************************************************************
* Function Name : SensoryFeatureCompatible
* Parameters    : Expects two initialized appStruct_T which are checked 
*               : for feature-compatibility
* Comment       : Returns TRUE if the two apps are feature-compatible.
*               : Used in case of multiple recognizers on same audio stream.
******************************************************************************/

BOOL SensoryFeatureCompatible(appStruct_T* src, appStruct_T* dst);

/******************************************************************************
* Function Name : SensoryConnectFeatures
* Parameters    : Expects two initialized appStruct_T which will get connected for 
*               : transmitting audio features, src to dst.  This will save feature processing time.
* Comment       : Will return ERR_OK if connected successfully or ERR_T2SI_FEATURE_MISMATCH.
*               : if the features are not compatible.
*               : Used in case of multiple recognizers on same audio stream.
******************************************************************************/

errors_t SensoryConnectFeatures(appStruct_T* src, appStruct_T* dst);

/******************************************************************************
* Function Name : SensoryProcessFeatures
* Parameters    : An app-struct which will take features from the source and process them
* Comment       : Typically returns ERR_NOT_FINISHED, or ERR_OK upon recognition.
*               : This works just like SensoryProcessData, but faster.
*               : Used in case of multiple recognizers on same audio stream.
******************************************************************************/

errors_t SensoryProcessFeatures(appStruct_T *app);

/******************************************************************************
* Function Name     : SensoryLPSDInit
* Parameters        : appStruct_T pointer
* Comment           : Initializes Low Power Sound Detector.
*                   : Generally called once after audio buffer is set up.
*                   : note: zeros t2siStruct values streamFrames and idleFrames.
******************************************************************************/
void SensoryLPSDInit(appStruct_T *ap); 
BOOL SensoryLPSDFrameReady(appStruct_T *ap);
void SensoryLPSDResetGetPointer(appStruct_T *ap);

/******************************************************************************
* Function Name     : SensoryLPSDProcessFrame
* Parameters        : appStruct_T pointer
* Comment           : called if there is a least one frame's worth of data in
*                   : the audio buffer (between audioGetIndex and audioPutIndex).
*                   : returns zero if LPSD believes recognition should not be done,
*                   : returns one if LPSD believes recognition should be done.
******************************************************************************/
BOOL SensoryLPSDProcessFrame(appStruct_T *ap);

/******************************************************************************
* Function Name     : SensoryFindEndpoint
* Parameters        : appStruct_T pointer, pointers to values to receive outputs
* Comment           : After successful recognition return from SensoryProcessData,
*                   : use this function to find the point (audio buffer index)
*                   : in the buffer that corresponds to the best recognition score.
*                   :
*                   : It directly returns a count of frames to backoff from the
*                   : current frame (audioGetIndex).  This can be used to set audioBackOffFrames
*                   : to continue recognition from the endpoint w/o a speech detector.
*                   :
*                   : If tailStart and tailCount are non-null, fills them with
*                   : the audio buffer index of the start of the end point and the
*                   : number of samples in the audio buffer from this point to audioPutIndex
******************************************************************************/
AUDIOINDEX SensoryFindEndpoint(appStruct_T *ap, AUDIOINDEX *tailStart, AUDIOINDEX *tailCount);

/******************************************************************************
* Function Name     : SensoryFindStartpoint
* Parameters        : appStruct_T pointer, pointers to values to receive outputs
* Comment           : After successful recognition return from SensoryProcessData,
*                   : use this function to find the point (audio buffer index)
*                   : in the buffer that corresponds to the best recognition score.
*                   :
*                   : It directly returns a count of frames to backoff from the
*                   : current frame (audioGetIndex).  
*                   :
*                   : If stIndex is non-null, fills it with the audio buffer
*                   : index of the starting point of the phrase.
*                   : If the start point is not in the audio buffer, stIndex returns -1. 
*                   : If there is a larger external buffer holding the whole phrase,
*                   : the return count can be used to backoff from present position  
*                   : to find the start point in the external buffer.
******************************************************************************/
AUDIOINDEX SensoryFindStartpoint(appStruct_T *ap, AUDIOINDEX *stIndex);

/******************************************************************************
* Function Name     : AppAudioStoreBrick
* Parameters        : appStruct_T pointer, brick pointer, brick size in samples
* Comment           : copies a brick of samples into the audio buffer, updates
*                   : audioPutIndex.
******************************************************************************/
void AppAudioStoreBrick(appStruct_T *ap, BRICK_SAMPLE *brick, s16 brickSize);

/******************************************************************************
* Function Name     : AppAudioGetFrame
* Parameters        : appStruct_T pointer, frame start offset and sample count
*                   : and destination pointer
* Comment           : copies a full or partial frame of data to the specified
*                   : destination. The source of the frame is audioGetIndex 
*                   : minus audioBackoffFrames.
*
*                   : returns 0 if not enough samples between audioGetIndex and
*                   : audioPutIndex to make a frame, in which case transfer is 
*                   : not done.  
*                   : otherwise returns 1 after transferring samples and updating
*                   : audioGetIndex and incrementing audioGetFrameCounter.
*
*                   : note: if count = zero, just update audioGetIndex and 
*                   : audioGetFrameCounter
******************************************************************************/
BOOL AppAudioGetFrame(appStruct_T *ap, s16 startOffset, s16 count, SAMPLE *dst);

/******************************************************************************
* Function Name     : SensoryDcBlock
* Parameters        : appStruct_T pointer, source & dest pointers, and count
* Comment           : if audioGetFrameCounter is 0, the filter initializes itself.
******************************************************************************/
void SensoryDcBlock(appStruct_T *asp, SAMPLE *src, SAMPLE *dst, s16 cnt);

/******************************************************************************
* Function Name     : AppAudioLPSDIncreasePowerMode
* Parameters        : returns 0 will not skip any processing in the transition frame
*                   : returns 1 will skip the rest of processing for this frame
* Comment           : When running LPSD, this function is called during the transition
*                   : from NO processing to DO processing so that user can increase power/
*                   : clock frequency before frontend processing.
******************************************************************************/
BOOL AppAudioLPSDIncreasePowerMode(void);

/******************************************************************************
* Function Name     : AppAudioLPSDDecreasePowerMode
* Comment           : When running LPSD, this function is called during the transition
*                   : from DO processing to NO processing so that user can decrease power/
*                   : clock frequency to conserve power.
******************************************************************************/
BOOL AppAudioLPSDDecreasePowerMode(void);


AUDIOINDEX AppAudioNumUnprocessedSamples(appStruct_T *ap);
BOOL AppAudioFrameReady(appStruct_T *ap);

/*
 * Sensory Logging feature related
 */

#ifdef SENSORY_LOGGING
/******************************************************************************
* Function Name : SensoryLogInit
* Parameters    : 
* log:			: Pass the logging buffer of data type of SENSORY_LOG_OBJ 
* n				: Size of the logging buffer
* Comment       : Called once to initialize logging variables
******************************************************************************/
void SensoryLogInit(appStruct_T *app, SENSORY_LOG_OBJ *log, int n);

/******************************************************************************
* Function Name : SensoryLog
* Parameters    : 
* event:		: Underlying logging event.
* a, b			: Underlying logging event's two arguments
* Comment       : Save the underlying logging function calls into the logging buffer
******************************************************************************/
void SensoryLog(t2siStruct *app, int event, int a, int b);

void SensoryPrintLog(appStruct_T *app);

/*
 * Sensory Logging event definition
 */
#define SENSORY_LOG_RESET          0
#define SENSORY_LOG_CUSTOMER1      1
#define SENSORY_LOG_CUSTOMER2      2 
#define SENSORY_LOG_INIT           3
#define SENSORY_LOG_AUDIO_OVERFLOW 4
#define SENSORY_LOG_OUT_OF_MEMORY  5
#define SENSORY_LOG_LPSD_ABOVE_T   6
#define SENSORY_LOG_LPSD_BELOW_T   7
#define SENSORY_LOG_WS_PASS        8
#define SENSORY_LOG_DPP            9
#define SENSORY_LOG_EPQ            10
#define SENSORY_LOG_RECOG          11
#define SENSORY_LOG_SCORES         12

#define SENSORY_LOG(p,a,b,c) SensoryLog((p),(a),(b),(c))
#else
#define SENSORY_LOG(p,a,b,c)
#endif

/*-------------- SENSORY ERROR CODES for technology routines  -------------------*/
// Sensory error codes are u08 format, will be mapped/cast into app-required values

//-- 0x, FX: Generic errors 
#define ERR_OK									0x00    // generic pass return
#define ERR_NOT_OK								0x01    // generic error return
#define ERR_SEARCH_PRUNED                       0xFA    // warning that search is being limited by maxTokens
#define ERR_MEMORY_CORRUPT						0xFB    // RAM apparently changed -- RESERVED FOR NXP COMPATIBILITY
#define ERR_NULL_POINTER						0xFC    // fatal null pointer
#define ERR_NOT_FINISHED						0xFD    // non-fatal, need more data 
#define ERR_NO_FREE_TOKENS						0xFE    // no free tokens left
#define ERR_LICENSE 0xFF // license isn't valid or event limit reached

//-- 1x, 2x: Data collection errors 
#define ERR_DATACOL_BASE						0x10    
#define ERR_DATACOL_TIMEOUT						ERR_DATACOL_BASE+0x01 // no utterance detected
#define ERR_DATACOL_TOO_SHORT					ERR_DATACOL_BASE+0x03 // utterance was too short
#define ERR_DATACOL_TOO_SOFT					ERR_DATACOL_BASE+0x04 // utterance was too soft

//-- 3x,4x: Recognition errors 
#define ERR_RECOG_BASE							0x30    
#define ERR_RECOG_FAIL							ERR_RECOG_BASE+0x01 // recognition failed
#define ERR_RECOG_LOW_CONF						ERR_RECOG_BASE+0x02 // recognition result doubtful
#define ERR_RECOG_MID_CONF						ERR_RECOG_BASE+0x03 // recognition result maybe

//-- 5x: T2SI errors       
#define ERR_T2SI_BASE							0x50    
#define ERR_T2SI_PSTORE							ERR_T2SI_BASE       // null persistent storage pointer                                                 
#define ERR_T2SI_BAD_VERSION					ERR_T2SI_BASE+0x01  // grammar version not supported
#define ERR_T2SI_NN_BAD_VERSION					ERR_T2SI_BASE+0x02  // net version not supported
#define ERR_T2SI_BAD_SETUP						ERR_T2SI_BASE+0x03  // gram or net not specified 
#define ERR_T2SI_TRIG_NOTA						ERR_T2SI_BASE+0x04  // trigger NOTA - continues
#define ERR_T2SI_NN_MISMATCH					ERR_T2SI_BASE+0x05  // gram/net mismatch(# of models)
#define ERR_T2SI_TOO_MANY_RESULTS				ERR_T2SI_BASE+0x06  // MAX_RESULTS is too small
#define ERR_T2SI_UNEXPECTED_NUM_EXTRA_MODELS	ERR_T2SI_BASE+0x07	// too many extra number of models
#define ERR_T2SI_UNEXPECTED_NUM_EXTRA_GRAMS		ERR_T2SI_BASE+0x08	// too many extra number of grammars
#define ERR_T2SI_UNEXPECTED_NUM_EXTRA_NETS		ERR_T2SI_BASE+0x09	// too many extra number of nets
#define ERR_T2SI_UNEXPECTED_MFCC_TYPE			ERR_T2SI_BASE+0x0A	// The MFCC type is not supported
#define ERR_T2SI_GRAMMAR_UNALIGNED			    ERR_T2SI_BASE+0x0B	// Grammar memory must be 4-byte aligned
#define ERR_T2SI_FEATURE_MISMATCH			    ERR_T2SI_BASE+0x0C	// SensoryProcessFeatures must have matching features

//-- 6x: DNN errors
#define ERR_DNN_BASE							0x60
#define ERR_DNN_BAD_VERSION						ERR_DNN_BASE+0x00  // dnn not right format/version
#define ERR_DNN_TOO_MANY_NETS					ERR_DNN_BASE+0x01  // too many dnn nets (> MAX_DNN_NETS)
#define ERR_DNN_BAD_FORMAT						ERR_DNN_BASE+0x02
#define ERR_DNN_SV_OR_NNPQ_MISMATCH				ERR_DNN_BASE+0x03  // grammar for nnSv is not SV; or NNPQ is not supported in this version
#define ERR_DNN_UNALIGNED                       ERR_DNN_BASE+0x04  // Net memory must be 4-byte aligned
#define ERR_DNN_ROM_COPY_CONFIG                 ERR_DNN_BASE+0x0A
                                                    
#endif  //SENSORY_LIB_INCLUDED
