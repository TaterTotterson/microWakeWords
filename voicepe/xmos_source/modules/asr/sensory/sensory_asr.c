// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <string.h>
#include <xcore/assert.h>
#include <xcore/hwtimer.h>

#include <sensorytypes.h>
#include <sensorylib.h>

#include "asr.h"
#include "device_memory.h"
#include "sensory_conf.h"

typedef struct sensory_asr_struct
{
    int32_t brick_count;
    int32_t start_index;
    int32_t end_index;
    int32_t duration;
    int32_t word_id;
    appStruct_T app;

} sensory_asr_t;

static devmem_manager_t *devmem_ctx = NULL;
static sensory_asr_t sensory_asr;

/**
 * Wrapper for devmem_read_ext called by libTHFMicro.
**/
__attribute__((fptrgroup("sensory_asr_xcore_memcpy_fptr_grp")))
static void xcore_memcpy(void * dst, const void * src, size_t size) \
{
    devmem_read_ext(devmem_ctx, dst, src, size);
}

/**
 * Return TRUE if ptr is in SwMem (flash) address range.
**/
__attribute__((fptrgroup("sensory_asr_xcore_is_flash_fptr_grp")))
static BOOL xcore_is_flash(const void * ptr)
{
    if (IS_FLASH((intptr_t) ptr)) return TRUE;
    else return FALSE;
}

#pragma stackfunction 50
asr_port_t asr_init(int32_t *model, int32_t *grammar, devmem_manager_t *devmem)
{
    errors_t error;
    appStruct_T *app = &(sensory_asr.app);
    t2siStruct *t = &(app->_t);
    unsigned int sppSize;
    sensory_asr.brick_count = 0;
    devmem_ctx = devmem;

    memset((void *) app, 0, sizeof(appStruct_T)); // Most app parameters can be zero

    if (xcore_is_flash(grammar)) {
        asr_printf("ERROR: Search part (-search.BIN file) should be in SRAM.\n");
        return NULL;
    }

    // Some parameters
    t->maxResults = SENSORY_ASR_MAX_RESULTS ? SENSORY_ASR_MAX_RESULTS : MAX_RESULTS;
    t->maxTokens = SENSORY_ASR_MAX_TOKENS ? SENSORY_ASR_MAX_TOKENS : MAX_TOKENS;
    t->sdet_type = SENSORY_ASR_SDET_TYPE;
    // Both these functions MUST be set for xcore if model is in ROM (must copy from ROM to work at all)
    t->devMemCpy = xcore_memcpy;
    t->devIsFlash = xcore_is_flash;
    t->romCacheSize = SENSORY_ASR_ADDITIONAL_ROM_CACHE;

    // initialize audio buffer items
    app->audioBufferLen = AUDIO_BUFFER_LEN;
    // if not using malloc for the audio buffer, just point audioBufferStart to the statically
    // allocated audio buffer:
    app->audioBufferStart = devmem_malloc(devmem_ctx, AUDIO_BUFFER_LEN * sizeof(s16));
    if (app->audioBufferStart == NULL)
    {
        asr_printf("ERROR: Audio buffer out of memory\n");
        return NULL;
    }

#if (SENSORY_ASR_SDET_TYPE == SDET_LPSD)
    SensoryLPSDInit(app);
#endif

    t->net = (intptr_t) model;
    t->gram = (intptr_t) grammar;

    error = SensoryAlloc(app, &sppSize);  // Find size needed
    if (error) {
        asr_printf("ERROR: SensoryAlloc failed with error 0x%x\n", error);
        return NULL;
    }
    asr_printf("Sensory SPP size=%u (bytes)\n", sppSize);

    // Allocate a single block of memory for all dynamic persistent data
    t->spp = (void *)devmem_malloc(devmem_ctx, sppSize);
    if (t->spp == NULL)
    {
        asr_printf("ERROR: No memory left for SPP\n");
        return NULL;
    }

    // initialize
    error = SensoryProcessInit(app);
    if (error) {
        asr_printf("ERROR: SensoryProcessInit failed with error 0x%x\n", error);
        return NULL;
    }

    asr_printf("SensoryProcessInit succeeded\n");
    return (asr_port_t) &sensory_asr;
}

#pragma stackfunction 250
asr_error_t asr_process(asr_port_t *ctx, int16_t *audio_buf, size_t buf_len)
{
    xassert(ctx);
    xassert(buf_len == FRAME_LEN);

    sensory_asr_t *sensory_asr = (sensory_asr_t *) ctx;
    appStruct_T *app = &(sensory_asr->app);
    t2siStruct *t = &(app->_t);
    errors_t error;

    sensory_asr->brick_count++;
    sensory_asr->word_id = -1;

    // Uncomment the line below to compute MIPS usage.
    // uint32_t timer_start = get_reference_time();

    error = SensoryProcessData((s16 *) audio_buf, app);

    // Uncomment the two lines below to compute MIPS usage.
    // uint32_t timer_end = get_reference_time();
    // asr_printf("Sensory processing time: %lu (us)\n", (timer_end - timer_start) / 100);

    // if (t->tokensPruned) {
    //     asr_printf("Search for recognizer was limited by maxTokens count %d\n"
    //                "You may wish to increase it.\n",  t->maxTokens);
    // }
    if (error == ERR_OK) {
        if (t->wordID) {
            AUDIOINDEX epIndex, stIndex, tailCount, startBackupFrames, endBackupFrames;

            asr_printf("Sensory Recognizer found wordID=%d  score=%d\n", t->wordID, t->finalScore);
            // if (t->svScore >= 0) asr_printf("svscore= %d", t->svScore);
            // if (t->nnpqScore > 0) asr_printf("\nNNPQ score = %d, NNPQ threshold = %d, NNPQ check pass = %d",
            //     t->nnpqScore, t->nnpqThreshold, t->nnpqPass);
            // asr_printf("\brick count= %d, duration= %d\n", sensory_asr->brick_count, t->duration);

            startBackupFrames = SensoryFindStartpoint(app, &stIndex);
            endBackupFrames = SensoryFindEndpoint(app, &epIndex, &tailCount);
            // asr_printf("startBackupFrames: %d, endBackupFrames: %d\n", (int) startBackupFrames, (int) endBackupFrames);
            // if (stIndex < 0)
            //     asr_printf("Start point is not in the audio buffer\n");
            sensory_asr->word_id = (int32_t)t->wordID;
            sensory_asr->start_index = (sensory_asr->brick_count - startBackupFrames) * FRAME_LEN;
            sensory_asr->end_index = (sensory_asr->brick_count - endBackupFrames) * FRAME_LEN;
            sensory_asr->duration = t->duration * FRAME_LEN;
            // asr_printf("start point = %d; end point = %d, duration = %d\n", sensory_asr->start_index, sensory_asr->end_index, sensory_asr->duration);

        } else {
            // Should not get here
            asr_printf("Not recognized\n");
        }
        if (SENSORY_ASR_KEEP_GOING) {
            SensoryProcessRestart(app);
        } else {
            return ASR_ERROR;
        }
    } else if (error != ERR_NOT_FINISHED) {
        if (error == ERR_LICENSE) {
            asr_printf("ERROR: Sensory Lib license error\n");
            return ASR_EVALUATION_EXPIRED;
        } else {
            asr_printf("ERROR: SensoryProcessData returned error code 0x%x, wordID= %d\n", error, t->wordID);
            return ASR_ERROR;
        }
    }

    return ASR_OK; // more to process
}

asr_error_t asr_get_result(asr_port_t *ctx, asr_result_t *result)
{
    xassert(ctx);
    xassert(result);

    sensory_asr_t *sensory_asr = (sensory_asr_t *) ctx;
    appStruct_T *app = &(sensory_asr->app);
    t2siStruct *t = &(app->_t);

    if (sensory_asr->word_id > 0) {
        result->id = t->wordID;
        result->score = t->finalScore;
        result->start_index = sensory_asr->start_index;
        result->end_index = sensory_asr->end_index;
        result->duration = sensory_asr->duration;
    } else {
        result->id = 0;
        result->score = 0;
        result->start_index = -1;
        result->end_index = -1;
        result->duration = -1;
    }


    return ASR_OK;
}

asr_error_t asr_get_attributes(asr_port_t *ctx, asr_attributes_t *attributes)
{
    xassert(ctx);
    xassert(attributes);

    attributes->samples_per_brick = FRAME_LEN;

    infoStruct_T info;
    errors_t err = SensoryInfo(&info);
    if (err == ERR_OK)
    {
        sprintf(attributes->engine_version, "%lu", info.version);
    }

    return ASR_OK;
}

asr_error_t asr_reset(asr_port_t *ctx)
{
    xassert(ctx);
    sensory_asr_t *sensory_asr = (sensory_asr_t *) ctx;
    appStruct_T *app = &(sensory_asr->app);

    SensoryProcessRestart(app);
    return ASR_OK;
}

asr_error_t asr_release(asr_port_t *ctx)
{
    xassert(ctx);

    sensory_asr_t *sensory_asr = (sensory_asr_t *) ctx;
    appStruct_T *app = &(sensory_asr->app);
    t2siStruct *t = &(app->_t);

    if (t->spp) {
        devmem_free(devmem_ctx, (void *) t-> spp);
        t->spp = 0;
    }
    if (app->audioBufferStart) {
        devmem_free(devmem_ctx, (void *) app->audioBufferStart);
        app->audioBufferStart = 0;
    }

    ctx = NULL;
    return ASR_OK;
}
