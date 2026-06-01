#include "debug_print.h"

#include <stdio.h>
#include <string.h>
#include <platform.h>
#include <xassert.h>

#include "platform/platform_conf.h"
#include "servicer.h"
#include "led_ring_servicer.h"

#include "led_ring_cmds.h"
#include "rtos_ws2812.h"

#include "FreeRTOS.h"


static control_cmd_info_t led_ring_servicer_resid_cmd_map[] = {
    { LED_RING_SERVICER_CMD_WRITE_RAW,  3 * LED_RING_NUM_LEDS , sizeof(uint8_t), CMD_WRITE_ONLY },
};

//-----------------Servicer read write callback functions-----------------------//
DEVICE_CONTROL_CALLBACK_ATTR
static control_ret_t led_ring_servicer_read_cmd(control_resid_t resid, control_cmd_t cmd, uint8_t *payload, size_t payload_len, void *app_data)
{
    // no read command implemented yet
    payload[0] = CONTROL_BAD_COMMAND;
    return CONTROL_BAD_COMMAND;
}

DEVICE_CONTROL_CALLBACK_ATTR
static control_ret_t led_ring_servicer_write_cmd(control_resid_t resid, control_cmd_t cmd, const uint8_t *payload, size_t payload_len, void *app_data)
{
    control_ret_t ret = CONTROL_SUCCESS;
    led_ring_servicer_ctx_t *led_servicer_ctx = (led_ring_servicer_ctx_t*) app_data;
    servicer_t *servicer = led_servicer_ctx->servicer; 
    rtos_ws2812_t *ws2812_ctx = led_servicer_ctx->ws2812_ctx;
    
    debug_printf("LED-Ring servicer on tile %d received WRITE command %02x for resid %02x\n\t", THIS_XCORE_TILE, cmd, resid);
    debug_printf("The command has %d bytes\n\t", payload_len);

    control_resource_info_t *current_res_info = get_res_info(resid, servicer);
    xassert(current_res_info != NULL);
    control_cmd_info_t *current_cmd_info;
    ret = validate_cmd(&current_cmd_info, current_res_info, cmd, payload, payload_len);
    if(ret != CONTROL_SUCCESS)
    {
        debug_printf("Validation of command failed! error: %d\n", ret);
        return ret;
    }
    
    //handle command
    uint8_t cmd_id = CONTROL_CMD_CLEAR_READ(cmd);
    debug_printf("led_ring_servicer_write_cmd cmd_id %d.\n", cmd_id);

    switch (cmd_id)
    {
    
    case LED_RING_SERVICER_CMD_WRITE_RAW:
    {
        debug_printf("GPIO_CONTROLLER_SERVICER_RESID_WRITE_PORT\n");
        debug_printf("%d %d %d\n", payload[0], payload[1], payload[2]);
        rtos_ws2812_write( ws2812_ctx, &payload[0] );
        break;
    }
    
    default:
        debug_printf("LED_RING_CONTROLLER_SERVICER UNHANDLED COMMAND!!!\n");
        ret = CONTROL_BAD_COMMAND;
        break;
    }

    return ret;
}



void led_ring_servicer_init(servicer_t *servicer)
{
    // Servicer resource info
    static control_resource_info_t led_ring_res_info[NUM_RESOURCES_LED_RING];

    memset(servicer, 0, sizeof(servicer_t));
    servicer->id = LED_RING_SERVICER_RESID;
    servicer->start_io = 0;
    servicer->num_resources = NUM_RESOURCES_LED_RING;

    servicer->res_info = &led_ring_res_info[0];
    // Servicer resource
    servicer->res_info[0].resource = LED_RING_SERVICER_RESID;
    servicer->res_info[0].command_map.num_commands = NUM_LED_RING_SERVICER_CMDS;
    servicer->res_info[0].command_map.commands = led_ring_servicer_resid_cmd_map;
}

void led_ring_servicer(void *args) {
    device_control_servicer_t servicer_ctx;

    servicer_register_ctx_t *servicer_reg_ctx = (servicer_register_ctx_t*)args;
    
    servicer_t *servicer = servicer_reg_ctx->servicer;
    rtos_ws2812_t *ws2812_ctx = (rtos_ws2812_t *) servicer_reg_ctx->app_data;
    
    xassert(servicer != NULL);
    xassert(ws2812_ctx != NULL);
    
    control_resid_t *resources = (control_resid_t*)pvPortMalloc(servicer->num_resources * sizeof(control_resid_t));
    for(int i=0; i<servicer->num_resources; i++)
    {
        resources[i] = servicer->res_info[i].resource;
    }

    control_ret_t dc_ret;
    debug_printf("Calling device_control_servicer_register(), servicer ID %d, on tile %d, core %d.\n", servicer->id, THIS_XCORE_TILE, rtos_core_id_get());

    dc_ret = device_control_servicer_register(&servicer_ctx,
                                            servicer_reg_ctx->device_control_ctx,
                                            1,
                                            resources, servicer->num_resources);
    debug_printf("Out of device_control_servicer_register(), servicer ID %d, on tile %d. servicer_ctx address = 0x%x\n", servicer->id, THIS_XCORE_TILE, &servicer_ctx);

    vPortFree(resources);

    led_ring_servicer_ctx_t led_servicer_ctx = {
        servicer,
        ws2812_ctx
    };

    for(;;){
        device_control_servicer_cmd_recv(&servicer_ctx, led_ring_servicer_read_cmd, led_ring_servicer_write_cmd, &led_servicer_ctx, RTOS_OSAL_WAIT_FOREVER);
    }
}

