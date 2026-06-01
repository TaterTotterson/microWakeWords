#include "debug_print.h"

#include <platform.h>
#include <stdint.h>
#include <string.h>
#include <xassert.h>

#include "FreeRTOS.h"

#include "doa_cmds.h"
#include "doa_estimator.h"
#include "doa_servicer.h"
#include "servicer.h"

static control_cmd_info_t doa_servicer_cmd_map[] = {
    { DOA_SERVICER_CMD_READ_STATE, DOA_SERVICER_STATE_NUM_VALUES, sizeof(uint8_t), CMD_READ_ONLY },
};

static void store_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFF);
    dst[1] = (uint8_t)((value >> 8) & 0xFF);
}

static void store_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFF);
    dst[1] = (uint8_t)((value >> 8) & 0xFF);
    dst[2] = (uint8_t)((value >> 16) & 0xFF);
    dst[3] = (uint8_t)((value >> 24) & 0xFF);
}

DEVICE_CONTROL_CALLBACK_ATTR
static control_ret_t doa_servicer_read_cmd(control_resid_t resid,
                                           control_cmd_t cmd,
                                           uint8_t *payload,
                                           size_t payload_len,
                                           void *app_data)
{
    control_ret_t ret = CONTROL_SUCCESS;
    servicer_t *servicer = (servicer_t *)app_data;

    payload_len -= 1;
    uint8_t *payload_ptr = &payload[1];

    control_resource_info_t *current_res_info = get_res_info(resid, servicer);
    xassert(current_res_info != NULL);

    control_cmd_info_t *current_cmd_info;
    ret = validate_cmd(&current_cmd_info, current_res_info, cmd, payload_ptr, payload_len);
    if (ret != CONTROL_SUCCESS) {
        payload[0] = ret;
        return ret;
    }

    switch (CONTROL_CMD_CLEAR_READ(cmd)) {
    case DOA_SERVICER_CMD_READ_STATE:
    {
        doa_estimator_state_t state;
        doa_estimator_get_state(&state);

        store_u16_le(&payload_ptr[0], (uint16_t)state.sample_delay);
        payload_ptr[2] = state.confidence;
        payload_ptr[3] = state.flags;
        store_u32_le(&payload_ptr[4], state.energy);
        store_u32_le(&payload_ptr[8], state.frame_counter);

        payload[0] = CONTROL_SUCCESS;
        return CONTROL_SUCCESS;
    }

    default:
        payload[0] = CONTROL_BAD_COMMAND;
        return CONTROL_BAD_COMMAND;
    }
}

DEVICE_CONTROL_CALLBACK_ATTR
static control_ret_t doa_servicer_write_cmd(control_resid_t resid,
                                            control_cmd_t cmd,
                                            const uint8_t *payload,
                                            size_t payload_len,
                                            void *app_data)
{
    return CONTROL_BAD_COMMAND;
}

void doa_servicer_init(servicer_t *servicer)
{
    static control_resource_info_t doa_res_info[NUM_RESOURCES_DOA];

    memset(servicer, 0, sizeof(servicer_t));
    servicer->id = DOA_SERVICER_RESID;
    servicer->start_io = 0;
    servicer->num_resources = NUM_RESOURCES_DOA;
    servicer->res_info = &doa_res_info[0];

    servicer->res_info[0].resource = DOA_SERVICER_RESID;
    servicer->res_info[0].command_map.num_commands = NUM_DOA_SERVICER_CMDS;
    servicer->res_info[0].command_map.commands = doa_servicer_cmd_map;
}

void doa_servicer(void *args)
{
    device_control_servicer_t servicer_ctx;
    servicer_t *servicer = (servicer_t *)args;

    xassert(servicer != NULL);

    control_resid_t resources[NUM_RESOURCES_DOA];
    for (int i = 0; i < servicer->num_resources; i++) {
        resources[i] = servicer->res_info[i].resource;
    }

    (void)device_control_servicer_register(&servicer_ctx,
                                           device_control_ctxs,
                                           1,
                                           resources,
                                           servicer->num_resources);

    for (;;) {
        device_control_servicer_cmd_recv(&servicer_ctx,
                                         doa_servicer_read_cmd,
                                         doa_servicer_write_cmd,
                                         servicer,
                                         RTOS_OSAL_WAIT_FOREVER);
    }
}
