#include "debug_print.h"

#include <stdio.h>
#include <string.h>
#include <platform.h>
#include <xassert.h>

#include "platform/platform_conf.h"
#include "servicer.h"
#include "gpio_servicer.h"

#include "gpio_cmds.h"
#include "rtos_gpio.h"

#include "FreeRTOS.h"

//#include "platform/app_pll_ctrl.h"

control_resid_t port_id_to_cntr_res[GPIO_CONTROLLER_MAX_RESOURCES] = {
    GPIO_CONTROLLER_RESOURCE_IN_A,
    GPIO_CONTROLLER_RESOURCE_IN_B,
    GPIO_CONTROLLER_RESOURCE_OUT_A
};

static rtos_gpio_port_id_t gpio_ctrl_ports[GPIO_CONTROLLER_MAX_RESOURCES]; 
static rtos_gpio_t *gpio_servicer_gpio_ctx = NULL;

static control_cmd_info_t gpio_servicer_resid_out_cmd_map[] = {
    { GPIO_CONTROLLER_SERVICER_CMD_READ_PORT,  1, sizeof(uint8_t), CMD_READ_ONLY   },
    { GPIO_CONTROLLER_SERVICER_CMD_WRITE_PORT, 1, sizeof(uint8_t), CMD_WRITE_ONLY  },
    { GPIO_CONTROLLER_SERVICER_CMD_SET_PIN,    2, sizeof(uint8_t), CMD_WRITE_ONLY  },
};

static control_cmd_info_t gpio_servicer_resid_in_cmd_map[] = {
    { GPIO_CONTROLLER_SERVICER_CMD_READ_PORT,  1, sizeof(uint8_t), CMD_READ_ONLY   },
};


//-----------------Servicer read write callback functions-----------------------//
DEVICE_CONTROL_CALLBACK_ATTR
static control_ret_t gpio_servicer_read_cmd(control_resid_t resid, control_cmd_t cmd, uint8_t *payload, size_t payload_len, void *app_data)
{
    control_ret_t ret = CONTROL_SUCCESS;
    servicer_t *servicer = (servicer_t*)app_data;

    // For read commands, payload[0] is reserved from status. So payload_len is one more than the payload_len stored in the resource command map
    payload_len -= 1;
    uint8_t *payload_ptr = &payload[1]; //Excluding the status byte, which is updated later.

    debug_printf("GPIO servicer on tile %d received READ command %02x for resid %02x\n\t", THIS_XCORE_TILE, cmd, resid);
    debug_printf("The command is requesting %d bytes\n\t", payload_len);


    control_resource_info_t *current_res_info = get_res_info(resid, servicer);
    xassert(current_res_info != NULL); // This should never happen
    control_cmd_info_t *current_cmd_info;
    ret = validate_cmd(&current_cmd_info, current_res_info, cmd, payload_ptr, payload_len);
    if(ret != CONTROL_SUCCESS)
    {
        payload[0] = ret; // Update status in byte 0
        return ret;
    }
    
    rtos_gpio_port_id_t target_port;
    switch( resid ){
        case GPIO_CONTROLLER_RESOURCE_IN_A:
            target_port = gpio_ctrl_ports[RESOURCE_IN_A];
            break;
        case GPIO_CONTROLLER_RESOURCE_IN_B:
            target_port = gpio_ctrl_ports[RESOURCE_IN_B];
            break;
        case GPIO_CONTROLLER_RESOURCE_OUT_A:
            target_port = gpio_ctrl_ports[RESOURCE_OUT_A];
            break;
        default:
            debug_printf("GPIO_CONTROLLER_SERVICER unknown resource!!!\n");
            ret = CONTROL_BAD_RESOURCE;
            payload[0] = ret;
            return ret;  
    }
    
    // Handle command
    uint8_t cmd_id = CONTROL_CMD_CLEAR_READ(cmd);
    switch (cmd_id)
    {
    case GPIO_CONTROLLER_SERVICER_CMD_READ_PORT:
        {
          debug_printf("GPIO_CONTROLLER_SERVICER_RESID_GET_PIN\n");
          payload[1] = rtos_gpio_port_in( gpio_servicer_gpio_ctx, target_port );
          payload[0] = CONTROL_SUCCESS;
          return CONTROL_SUCCESS;
        }

    default:
        {
          debug_printf("GPIO_CONTROLLER_SERVICER UNHANDLED COMMAND!!!\n");
          ret = CONTROL_BAD_COMMAND;
          payload[0] = ret;
          return ret;
        }
    }
    return CONTROL_ERROR;
}

DEVICE_CONTROL_CALLBACK_ATTR
static control_ret_t gpio_servicer_write_cmd(control_resid_t resid, control_cmd_t cmd, const uint8_t *payload, size_t payload_len, void *app_data)
{
    control_ret_t ret = CONTROL_SUCCESS;
    servicer_t *servicer = (servicer_t*)app_data;
    //debug_printf("Device control WRITE. Servicer ID %d\n\t", servicer->id);

    debug_printf("GPIO servicer on tile %d received WRITE command %02x for resid %02x\n\t", THIS_XCORE_TILE, cmd, resid);
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
    
    rtos_gpio_port_id_t target_port;
    switch( resid ){
        case GPIO_CONTROLLER_RESOURCE_IN_A:
            debug_printf("Writing to PORTB not allowed!!!\n");
            ret = CONTROL_BAD_RESOURCE;
            return ret;  
            break;
        case GPIO_CONTROLLER_RESOURCE_IN_B:
            debug_printf("Writing to PORTB not allowed!!!\n");
            ret = CONTROL_BAD_RESOURCE;
            return ret;  
            break;
        case GPIO_CONTROLLER_RESOURCE_OUT_A:
            target_port = gpio_ctrl_ports[RESOURCE_OUT_A];
            break;
        default:
            debug_printf("GPIO_CONTROLLER_SERVICER unknown resource!!!\n");
            ret = CONTROL_BAD_RESOURCE;
            return ret;  
    }
    
    
    //handle command
    uint8_t cmd_id = CONTROL_CMD_CLEAR_READ(cmd);
    debug_printf("gpio_servicer_write_cmd cmd_id %d.\n", cmd_id);

    switch (cmd_id)
    {
    case GPIO_CONTROLLER_SERVICER_CMD_WRITE_PORT:
    {
        debug_printf("GPIO_CONTROLLER_SERVICER_RESID_WRITE_PORT\n");
        const uint8_t val  = payload[0];
        rtos_gpio_port_out( gpio_servicer_gpio_ctx, target_port, val );
        break;
    }
    case GPIO_CONTROLLER_SERVICER_CMD_SET_PIN:
    {        
        const uint8_t pin  = payload[0];
        const uint8_t val  = payload[1];
        debug_printf("GPIO_CONTROLLER_SERVICER_RESID_SET_PIN pin: %d val: %d\n", payload[0], payload[1]);
        uint8_t port_val = rtos_gpio_port_in( gpio_servicer_gpio_ctx, target_port );
        if ( val ){
            port_val |=  (( val & 1) << pin );
        } else {
            port_val &= ~( 1 << pin );
        }
        rtos_gpio_port_out( gpio_servicer_gpio_ctx, target_port, port_val );
        break;
    }
    default:
        debug_printf("GPIO_CONTROLLER_SERVICER UNHANDLED COMMAND!!!\n");
        ret = CONTROL_BAD_COMMAND;
        break;
    }

    return ret;
}

RTOS_GPIO_ISR_CALLBACK_ATTR
static void gpio_callback(rtos_gpio_t *ctx, void *app_data, rtos_gpio_port_id_t port_id, uint32_t value)
{
    TaskHandle_t task = app_data;
    BaseType_t yield_required = pdFALSE;

    //value = (~value) & GPIO_BITMASK;

    xTaskNotifyFromISR(task, value, eSetValueWithOverwrite, &yield_required);

    portYIELD_FROM_ISR(yield_required);
}


void gpio_handler_task( device_control_gpio_ctx_t *ctx )
{
    debug_printf("GPIO handler task on tile %d, core %d.\n", THIS_XCORE_TILE, rtos_core_id_get());
    uint32_t value;
    uint32_t gpio_val;
    rtos_gpio_t *gpio_ctx = ctx->gpio_ctx;
    
    for( int p=0; p<ctx->num_of_ports; p++){
      if( !ctx->port_defs[p].writeable){
        rtos_gpio_port_pull_up(gpio_ctx, gpio_ctrl_ports[ctx->port_defs[p].resource_idx]);
        rtos_gpio_isr_callback_set(gpio_ctx, gpio_ctrl_ports[ctx->port_defs[p].resource_idx], gpio_callback, xTaskGetCurrentTaskHandle());
        rtos_gpio_interrupt_enable(gpio_ctx, gpio_ctrl_ports[ctx->port_defs[p].resource_idx]);
      }
    }

    for (;;) {
        xTaskNotifyWait(
                0x00000000UL,    /* Don't clear notification bits on entry */
                0xFFFFFFFFUL,    /* Reset full notification value on exit */
                &value,          /* Pass out notification value into value */
                portMAX_DELAY ); /* Wait indefinitely until next notification */

        for( int p=0; p<ctx->num_of_ports; p++){
          gpio_val = rtos_gpio_port_in(gpio_ctx, gpio_ctrl_ports[ctx->port_defs[p].resource_idx]);
          gpio_val &= ctx->port_defs[p].bit_mask;
          gpio_val >>= ctx->port_defs[p].bit_shift;
          debug_printf("GPIO handler task:  Port %d to %d.\n", p, gpio_val );
          device_control_set_resource_status( ctx->device_control_ctx[0], ctx->port_defs[p].status_register, gpio_val );    
        }   
    }
}



void gpio_servicer_task(void *args) {
    device_control_servicer_t servicer_ctx;
    
    device_control_gpio_ctx_t *ctx = (device_control_gpio_ctx_t*) args;
    servicer_t *servicer = ctx->servicer;
    rtos_gpio_t *gpio_ctx = ctx->gpio_ctx;
    
    xassert(servicer != NULL);
    xassert(gpio_ctx != NULL);
    
    debug_printf("Number of ports: %d\n", ctx->num_of_ports);
    for(int p=0; p < ctx->num_of_ports; p++){
        debug_printf("Enabling, resource_idx %d, port_idx: .\n", ctx->port_defs[p].resource_idx);
        rtos_gpio_port_enable(gpio_ctx, gpio_ctrl_ports[ctx->port_defs[p].resource_idx]);
    }
    
    control_resid_t *resources = (control_resid_t*)pvPortMalloc(servicer->num_resources * sizeof(control_resid_t));
    for(int i=0; i<servicer->num_resources; i++)
    {
        resources[i] = servicer->res_info[i].resource;
    }

    control_ret_t dc_ret;
    debug_printf("Calling device_control_servicer_register(), servicer ID %d, on tile %d, core %d.\n", servicer->id, THIS_XCORE_TILE, rtos_core_id_get());

    dc_ret = device_control_servicer_register(&servicer_ctx,
                                            ctx->device_control_ctx,
                                            ctx->device_control_ctx_count,
                                            resources, servicer->num_resources);
    debug_printf("Out of device_control_servicer_register(), servicer ID %d, on tile %d. servicer_ctx address = 0x%x\n", servicer->id, THIS_XCORE_TILE, &servicer_ctx);

    vPortFree(resources);

    xTaskCreate((TaskFunction_t) gpio_handler_task,
                  "gpio_handler",
                   RTOS_THREAD_STACK_SIZE(gpio_handler_task),
                   ctx,
                   configMAX_PRIORITIES-1,
                   NULL);
    
    for(;;){
        device_control_servicer_cmd_recv(&servicer_ctx, gpio_servicer_read_cmd, gpio_servicer_write_cmd, servicer, RTOS_OSAL_WAIT_FOREVER);
    }
}


void gpio_servicer_init(device_control_gpio_ctx_t *ctx, rtos_gpio_t *gpio_ctx, device_control_gpio_ports_t* port_defs, uint8_t num_of_ports ){
    static servicer_t servicer;
    static control_resource_info_t gpio_res_info[GPIO_CONTROLLER_MAX_RESOURCES];
    
    ctx->servicer = &servicer;    
    ctx->gpio_ctx = gpio_ctx;
    ctx->num_of_ports = num_of_ports;
    ctx->port_defs = port_defs;
    
    memset(&servicer, 0, sizeof(servicer_t));
    servicer.id = GPIO_CONTROLLER_SERVICER_RESID;
    servicer.num_resources = num_of_ports;
    servicer.res_info = &gpio_res_info[0];
    for( int p=0; p < ctx->num_of_ports; p++ ){
       if( ctx->port_defs[p].writeable){
         servicer.res_info[p].resource = port_id_to_cntr_res[ctx->port_defs[p].resource_idx];
         servicer.res_info[p].command_map.num_commands = 3;
         servicer.res_info[p].command_map.commands = gpio_servicer_resid_out_cmd_map; 
       } else {
         servicer.res_info[p].resource = port_id_to_cntr_res[ctx->port_defs[p].resource_idx];
         servicer.res_info[p].command_map.num_commands = 1;
         servicer.res_info[p].command_map.commands = gpio_servicer_resid_in_cmd_map; 
       }
       gpio_ctrl_ports[ctx->port_defs[p].resource_idx] = rtos_gpio_port(ctx->port_defs[p].port_id);
    }
}


void gpio_servicer_start(device_control_gpio_ctx_t *ctx, device_control_t **device_control_ctx, size_t device_control_ctx_count ){
    ctx->device_control_ctx = device_control_ctx;
    ctx->device_control_ctx_count = device_control_ctx_count;
    
    xTaskCreate(
        gpio_servicer_task,
        "GPIO servicer",
        RTOS_THREAD_STACK_SIZE(gpio_servicer_task),
        ctx,
        appconfDEVICE_CONTROL_SPI_PRIORITY-1,
        NULL
    );
}