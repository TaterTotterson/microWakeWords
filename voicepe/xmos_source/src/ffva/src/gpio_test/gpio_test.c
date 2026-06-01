// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <platform.h>

#include "FreeRTOS.h"

#include "platform/app_pll_ctrl.h"
#include "gpio_test/gpio_test.h"

#if XK_VOICE_L71
#define BUTTON_MUTE_BITMASK PIN_MUTE_DET_IN
#define BUTTON_BTN_BITMASK  PIN_BUT_A_IN
#define GPIO_BITMASK    (BUTTON_MUTE_BITMASK | BUTTON_BTN_BITMASK)
#define GPIO_PORT       PORT_GPI

static int mute_status = -1;
static int button_status = -1;

#elif XCOREAI_EXPLORER
#define BUTTON_0_BITMASK    0x01
#define BUTTON_1_BITMASK    0x02
#define GPIO_BITMASK    (BUTTON_0_BITMASK | BUTTON_1_BITMASK)
#define GPIO_PORT       PORT_BUTTONS

#elif OSPREY_BOARD
#define BUTTON_0_BITMASK    0x04
#define GPIO_BITMASK    (BUTTON_0_BITMASK)
#define GPIO_PORT       PORT_BUTTON

#else
#define GPIO_PORT       0
#endif

RTOS_GPIO_ISR_CALLBACK_ATTR
static void gpio_callback(rtos_gpio_t *ctx, void *app_data, rtos_gpio_port_id_t port_id, uint32_t value)
{
    TaskHandle_t task = app_data;
    BaseType_t yield_required = pdFALSE;

    value = (~value) & GPIO_BITMASK;

    xTaskNotifyFromISR(task, value, eSetValueWithOverwrite, &yield_required);

    portYIELD_FROM_ISR(yield_required);
}

static void gpio_handler(rtos_gpio_t *gpio_ctx)
{
    uint32_t value;
    uint32_t gpio_val;

    const rtos_gpio_port_id_t gpio_port = rtos_gpio_port(GPIO_PORT);

    rtos_gpio_port_enable(gpio_ctx, gpio_port);

    rtos_gpio_isr_callback_set(gpio_ctx, gpio_port, gpio_callback, xTaskGetCurrentTaskHandle());
    rtos_gpio_interrupt_enable(gpio_ctx, gpio_port);

    for (;;) {
        xTaskNotifyWait(
                0x00000000UL,    /* Don't clear notification bits on entry */
                0xFFFFFFFFUL,    /* Reset full notification value on exit */
                &value,          /* Pass out notification value into value */
                portMAX_DELAY ); /* Wait indefinitely until next notification */

        gpio_val = rtos_gpio_port_in(gpio_ctx, gpio_port);

#if XK_VOICE_L71
        gpio_val = gpio_val & GPIO_BITMASK;

        if (((gpio_val & BUTTON_MUTE_BITMASK) != 0) && (mute_status != 1)) {
            rtos_printf("Mute active\n");
            mute_status = 1;
        } else if (((gpio_val & BUTTON_MUTE_BITMASK) == 0) && (mute_status != 0)) {
            rtos_printf("Mute inactive\n");
            mute_status = 0;
        }

        if ((gpio_val & BUTTON_BTN_BITMASK) == 0 && button_status != 1) {
            rtos_printf("Button pressed\n");
            button_status = 1;
        } else if ((gpio_val & BUTTON_BTN_BITMASK) != 0 && button_status != 0) {
            rtos_printf("Button released\n");
            button_status = 0;
        }
#elif XCOREAI_EXPLORER
        extern volatile int mic_from_usb;
        extern volatile int aec_ref_source;
        if (( gpio_val & BUTTON_0_BITMASK ) == 0) {
            mic_from_usb = !mic_from_usb;
            rtos_printf("Microphone from USB: %s\n", mic_from_usb ? "true" : "false");
        } else if (( gpio_val & BUTTON_1_BITMASK ) == 0) {
            aec_ref_source = !aec_ref_source;
            rtos_printf("AEC reference source: %s\n", aec_ref_source == appconfAEC_REF_I2S ? "I2S" : "USB");
        }
#elif OSPREY_BOARD
        extern volatile int mic_from_usb;
        if (( gpio_val & BUTTON_0_BITMASK ) == 0) {
            mic_from_usb = !mic_from_usb;
            rtos_printf("Microphone from USB: %s\n", mic_from_usb ? "true" : "false");
        }
#endif
    }
}

void gpio_test(rtos_gpio_t *gpio_ctx)
{
    if (GPIO_PORT != 0) {
        xTaskCreate((TaskFunction_t) gpio_handler,
                    "gpio_handler",
                    RTOS_THREAD_STACK_SIZE(gpio_handler),
                    gpio_ctx,
                    configMAX_PRIORITIES-2,
                    NULL);
    }
}
