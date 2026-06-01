// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* System headers */
#include <platform.h>
#include <xs1.h>
#include <xcore/hwtimer.h>
#include <xcore/assert.h>

/* App headers */
#include "platform/app_pll_ctrl.h"

void app_pll_set_numerator(int numerator)
{
    const unsigned tileid = get_local_tile_id();
    uint32_t fracval = APP_PLL_FRAC_NOM & 0xFFFF00FF;
    uint32_t f;

    if (numerator > 255) {
        f = 255;
    } else if (numerator < 0) {
        f = 0;
    } else {
        f = numerator;
    }

    fracval |= (f << 8);
    write_sswitch_reg_no_ack(tileid, XS1_SSWITCH_SS_APP_PLL_FRAC_N_DIVIDER_NUM, fracval);
}

void app_pll_init(void)
{
    unsigned tileid = get_local_tile_id();

    const unsigned APP_PLL_DISABLE = 0x0201FF04;
    const unsigned APP_PLL_DIV_0   = 0x80000004;

    write_sswitch_reg(tileid, XS1_SSWITCH_SS_APP_PLL_CTL_NUM, APP_PLL_DISABLE);

    hwtimer_t tmr = hwtimer_alloc();
    {
        xassert(tmr != 0);
        hwtimer_delay(tmr, 100000); // 1ms with 100 MHz timer tick
    }
    hwtimer_free(tmr);

    write_sswitch_reg(tileid, XS1_SSWITCH_SS_APP_PLL_CTL_NUM, APP_PLL_CTL_VAL);
    write_sswitch_reg(tileid, XS1_SSWITCH_SS_APP_PLL_CTL_NUM, APP_PLL_CTL_VAL);
    write_sswitch_reg(tileid, XS1_SSWITCH_SS_APP_PLL_FRAC_N_DIVIDER_NUM, APP_PLL_FRAC_NOM);
    write_sswitch_reg(tileid, XS1_SSWITCH_SS_APP_CLK_DIVIDER_NUM, APP_PLL_DIV_0);
}
