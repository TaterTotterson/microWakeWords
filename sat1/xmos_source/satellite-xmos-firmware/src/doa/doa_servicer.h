#pragma once

#include "servicer.h"

#define DOA_SERVICER_RESID      (231)
#define NUM_RESOURCES_DOA       (1)

void doa_servicer(void *args);
void doa_servicer_init(servicer_t *servicer);
