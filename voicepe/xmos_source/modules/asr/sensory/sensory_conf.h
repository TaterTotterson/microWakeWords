// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef SENSORY_CONF_H_
#define SENSORY_CONF_H_

#include "app_conf.h"
#include <sensorytypes.h>
#include "device_memory.h"

#include <sensorytypes.h>
#include <sensorylib.h>

// configuration variables

#ifndef SENSORY_ASR_ADDITIONAL_ROM_CACHE
// OPTIONAL larger ROM cache to speed up processing
#define SENSORY_ASR_ADDITIONAL_ROM_CACHE    (0)
#endif

#ifndef SENSORY_ASR_KEEP_GOING
#define SENSORY_ASR_KEEP_GOING              (1)
#endif

#ifndef SENSORY_ASR_MAX_RESULTS
// > 0 : override MAX_RESULTS
#define SENSORY_ASR_MAX_RESULTS             (0)
#endif

#ifndef SENSORY_ASR_MAX_TOKENS
// > 0 : override MAX_TOKENS
#define SENSORY_ASR_MAX_TOKENS              (500)
#endif

#ifndef SENSORY_ASR_SDET_TYPE
// Use SDET_LPSD for Low Power Sound Detect
#define SENSORY_ASR_SDET_TYPE               (SDET_NONE)
#endif

#endif /* SENSORY_CONF_H_ */
