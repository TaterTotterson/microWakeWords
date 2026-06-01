// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef ADAPTIVE_RATE_ADJUST_H_
#define ADAPTIVE_RATE_ADJUST_H_

#include "tusb_config.h"

#if CFG_TUD_AUDIO

#include <xcore/channel.h>
#include <xcore/clock.h>

#define USB_DIR_OUT 0
#define USB_DIR_IN  1

/* Add q formats as a hack to build */
#define Q0(N) Q ## N
#define Q(N) Q0(N)

// Convert from floating point to fixed point Q format.
// The number indicates the fractional bits or the position of the binary point
#define Q31(f) (int)((signed long long)((f) * ((unsigned long long)1 << (31+20)) + (1<<19)) >> 20)
#define Q30(f) (int)((signed long long)((f) * ((unsigned long long)1 << (30+20)) + (1<<19)) >> 20)
#define Q29(f) (int)((signed long long)((f) * ((unsigned long long)1 << (29+20)) + (1<<19)) >> 20)
#define Q28(f) (int)((signed long long)((f) * ((unsigned long long)1 << (28+20)) + (1<<19)) >> 20)
#define Q27(f) (int)((signed long long)((f) * ((unsigned long long)1 << (27+20)) + (1<<19)) >> 20)
#define Q26(f) (int)((signed long long)((f) * ((unsigned long long)1 << (26+20)) + (1<<19)) >> 20)
#define Q25(f) (int)((signed long long)((f) * ((unsigned long long)1 << (25+20)) + (1<<19)) >> 20)
#define Q24(f) (int)((signed long long)((f) * ((unsigned long long)1 << (24+20)) + (1<<19)) >> 20)
#define Q23(f) (int)((signed long long)((f) * ((unsigned long long)1 << (23+20)) + (1<<19)) >> 20)
#define Q22(f) (int)((signed long long)((f) * ((unsigned long long)1 << (22+20)) + (1<<19)) >> 20)
#define Q21(f) (int)((signed long long)((f) * ((unsigned long long)1 << (21+20)) + (1<<19)) >> 20)
#define Q20(f) (int)((signed long long)((f) * ((unsigned long long)1 << (20+20)) + (1<<19)) >> 20)
#define Q19(f) (int)((signed long long)((f) * ((unsigned long long)1 << (19+20)) + (1<<19)) >> 20)
#define Q18(f) (int)((signed long long)((f) * ((unsigned long long)1 << (18+20)) + (1<<19)) >> 20)
#define Q17(f) (int)((signed long long)((f) * ((unsigned long long)1 << (17+20)) + (1<<19)) >> 20)
#define Q16(f) (int)((signed long long)((f) * ((unsigned long long)1 << (16+20)) + (1<<19)) >> 20)
#define Q15(f) (int)((signed long long)((f) * ((unsigned long long)1 << (15+20)) + (1<<19)) >> 20)
#define Q14(f) (int)((signed long long)((f) * ((unsigned long long)1 << (14+20)) + (1<<19)) >> 20)
#define Q13(f) (int)((signed long long)((f) * ((unsigned long long)1 << (13+20)) + (1<<19)) >> 20)
#define Q12(f) (int)((signed long long)((f) * ((unsigned long long)1 << (12+20)) + (1<<19)) >> 20)
#define Q11(f) (int)((signed long long)((f) * ((unsigned long long)1 << (11+20)) + (1<<19)) >> 20)
#define Q10(f) (int)((signed long long)((f) * ((unsigned long long)1 << (10+20)) + (1<<19)) >> 20)
#define Q9(f)  (int)((signed long long)((f) * ((unsigned long long)1 << (9+20)) + (1<<19)) >> 20)
#define Q8(f)  (int)((signed long long)((f) * ((unsigned long long)1 << (8+20)) + (1<<19)) >> 20)

void adaptive_rate_adjust_init(void);

#endif

#endif /* ADAPTIVE_RATE_ADJUST_H_ */
