// Copyright 2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#ifndef XCORE_VOICE_ASR_H
#define XCORE_VOICE_ASR_H

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "device_memory.h"

/**
 * \addtogroup asr_api asr_api
 *
 * The public API for XCORE-VOICE ASR ports.
 * @{
 */

/**
 * String output function that allows the application  
 * to provide an alternative implementation.
 * 
 * ASR ports should call asr_printf instead of printf
 */
__attribute__((weak))
void asr_printf(const char * format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}


/**
 * Typedef to the ASR port context struct.
 *
 * An ASR port can store any data needed in the context.
 * The context pointer is passed to all API methods and 
 * can be cast to any struct defined by the ASR port.
 */
typedef void* asr_port_t;

/**
 * Typedef representing the base type of an audio sample.
 */
typedef int16_t asr_sample_t;

/**
 * Typedef to the ASR port and model attributes
 */
typedef struct asr_attributes_struct
{
    int16_t     samples_per_brick;  ///< Input brick length (in samples) required for calls to asr_process
    char        engine_version[10];     ///< ASR port engine version
    char        model_version[10];      ///< Model version
    size_t      required_memory;    ///< Memory (in bytes) required by engine and model
    void*       reserved;           ///< Reserved for future use
} asr_attributes_t;

/**
 * Typedef to the ASR result
 */
typedef struct asr_result_struct
{
    uint16_t id;             ///< Keyword or command ID
    uint16_t score;          ///< The confidence score of the detection
    uint16_t gscore;         ///< The garbage score
    int32_t  start_index;    ///< The audio sample index that corresponds to the start of the utterance
    int32_t  end_index;      ///< The audio sample index that corresponds to the end of the utterance
    int32_t  duration;       ///< THe length of the utterance in samples
    void*    reserved;       ///< Reserved for future use
} asr_result_t;

/**
 * Enumerator type representing error return values.
 */
typedef enum asr_error_enum {
    ASR_OK = 0,              ///< Ok
    ASR_ERROR,               ///< General error  
    ASR_INSUFFICIENT_MEMORY, ///< Insufficient memory for given model
    ASR_NOT_SUPPORTED,       ///< Function not supported for given model
    ASR_INVALID_PARAMETER,   ///< Invalid Parameter
    ASR_MODEL_INCOMPATIBLE,  ///< Model type or version is not compatible with the ASR library
    ASR_MODEL_CORRUPT,       ///< Model malformed
    ASR_NOT_INITIALIZED,     ///< Not Initialized
    ASR_EVALUATION_EXPIRED,   ///< Evaluation period has expired
} asr_error_t;

/**
 * Initialize an ASR port.
 *
 * \param model      A pointer to the model data.
 * \param grammar    A pointer to the grammar data (Optional).
 * \param devmem_ctx A pointer to the device manager (Optional). 
 *                   Save this pointer if calling any device manager API functions.
 *
 * \returns the ASR port context.
 */
asr_port_t asr_init(int32_t *model, int32_t *grammar, devmem_manager_t *devmem_ctx);

/**
 * Get engine and model attributes.
 *
 * \param ctx         A pointer to the ASR port context.
 * \param attributes  The attributes result.
 * 
 * \returns Success or error code.  
 */
asr_error_t asr_get_attributes(asr_port_t *ctx, asr_attributes_t *attributes);

/**
 * Process an audio buffer.
 *
 * \param ctx        A pointer to the ASR port context.
 * \param audio_buf  A pointer to the 16-bit PCM samples.
 * \param buf_len    The number of PCM samples.
 * 
 * \returns Success or error code.  
 */
asr_error_t asr_process(asr_port_t *ctx, int16_t *audio_buf, size_t buf_len);

/**
 * Get the most recent results.
 *
 * \param ctx        A pointer to the ASR port context.
 * \param result     The processed result.
 * 
 * \returns Success or error code.  
 */
asr_error_t asr_get_result(asr_port_t *ctx, asr_result_t *result);

/**
 * Reset ASR port (if necessary).
 * 
 * Called before the next call to asr_process.
 *
 * \param ctx        A pointer to the ASR port context.
 * 
 * \returns Success or error code.  
 */
asr_error_t asr_reset(asr_port_t *ctx);

/**
 * Release ASR port (if necessary).
 * 
 * The ASR port must deallocate any memory.
 *
 * \param ctx        A pointer to the ASR port context.
 * 
 * \returns Success or error code.  
 */
asr_error_t asr_release(asr_port_t *ctx);

/**@}*/

#endif // XCORE_VOICE_ASR_H
