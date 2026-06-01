#ifndef _DBG_TRACE_H
#define _DBG_TRACE_H

#ifdef DBG_TRACE
    #undef  DBG_TRACE
    #define DBG_TRACE    DbgTrace
#else
    #define DBG_TRACE    rtos_printf
#endif


#ifdef __cplusplus
extern "C" {
#endif

void DbgTrace(const char *lpszFormat, ...);
void UartTrace(const char *lpszFormat, ...);

#ifdef __cplusplus
}
#endif

#endif //_DBG_TRACE_H
