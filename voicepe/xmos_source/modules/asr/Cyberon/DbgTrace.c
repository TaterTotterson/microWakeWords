#include <platform.h>
#include "app_conf.h"   // For __VALIST
#include "DbgTrace.h"


void DbgTrace(const char *lpszFormat, ...)
{
    char szTemp[256];
    __VALIST args;

    va_start(args, lpszFormat);
	int n = vsnprintf(szTemp, 256, lpszFormat, args);
	if (n >= 0 && n < 256)
	{
		rtos_printf(szTemp);
		rtos_uart_tx_write(uart_tx_ctx, (uint8_t*)szTemp, strlen(szTemp));
	}
    va_end(args);
}
