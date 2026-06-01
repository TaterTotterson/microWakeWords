
#ifndef __CONFIGURATION_COMMON_H__
#define __CONFIGURATION_COMMON_H__

#include <stdint.h>

#define CONFIGURATION_FACTORY_ZONE      0
#define CONFIGURATION_CUSTOMER_ZONE     1

uint32_t configuration_init();

uint32_t configuration_flush();

uint32_t configuration_write_to_flash(uint8_t zone,
                                   uint8_t const *data,
                                   uint16_t length);

uint16_t configuration_read_from_flash(uint8_t zone,
                                    uint8_t *data,
                                    uint16_t length);

#endif