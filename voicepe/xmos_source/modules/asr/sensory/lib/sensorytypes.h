/*
 * File      : sensorytypes.h
 * Purpose   : typedefs and code/data section definitions
 * Project   : any 
 * Platform  : PC simulation of VPC and CSR code 
 *
 * Copyright (C) 1995-2022 Sensory Inc., All Rights Reserved
 *
 * ******* SENSORY CONFIDENTIAL *************
 *
 *****************************************************************************
*/
#ifndef SENSORYTYPES_H_INCLUDED
#define SENSORYTYPES_H_INCLUDED

// define for Sensory logging feature
#define xSENSORY_LOGGING

#include <stddef.h>
#ifndef NO_STDINT

#include <stdint.h>

typedef uint8_t  u08;
typedef uint16_t u16;
typedef uint32_t u24;
typedef uint32_t u32;
typedef uint64_t u48;
typedef uint64_t u64;

typedef int8_t   s08;
typedef int16_t  s16;
typedef int32_t  s24;
typedef int32_t  s32;
typedef int64_t  s48;
typedef int64_t  s64;
// type needed when doing pointer + offset arithmetic to create pointer
typedef intptr_t INTFORPTR;

#else // No stdint

typedef unsigned char       u08;  
typedef unsigned short      u16;  
typedef unsigned long       u24;  
typedef unsigned long       u32;  
typedef unsigned long long  u48;
typedef unsigned long long  u64;

typedef signed char         s08;  
typedef short               s16;
typedef long                s24;  
typedef long                s32;
typedef long long           s48;  
typedef long long           s64;
// type needed when doing pointer + offset arithmetic to create pointer
typedef int                 INTFORPTR;

#endif

#ifndef bool
typedef u08  bool;
#endif

#ifndef BOOL
typedef u08  BOOL;
#endif

#ifndef TRUE
#define TRUE ((BOOL) 1)
#endif

#ifndef FALSE
#define FALSE ((BOOL) 0)
#endif

// types needed for C nets and grammars
typedef const u16  __nn_t;
typedef const u16  __dnn_t;
typedef const u16  __gs_t;

#ifndef NULL
#define NULL  (void *) 0
#endif

// needed for compatibility with VPC
#define UCODE
#define IRAM
#define IRAM1
#define RDATA const 
#define NDATA const         
#define CONST const

// limits
#ifndef UINT16_MAX
#define UINT16_MAX (~(~0<<16))
#endif
#ifndef INT16_MAX
#define INT16_MAX  (~(~0<<15))
#endif
#ifndef INT16_MIN
#define INT16_MIN  (-32768)
#endif
#ifndef UINT32_MAX
#define UINT32_MAX (0x00000000ffffffffUL)
#endif
#ifndef INT32_MAX
#define INT32_MAX  (0x000000007fffffffL)
#endif
#ifndef INT32_MIN
#define INT32_MIN  (0xffffffff80000000L)
#endif
//----------- platform-specific memory sizes

#define FFT_BUFFER_LEN  512
#define TMP_BUFFER_LEN  240  // 144 will skip first 96 samples of first frame

//----------- platform-specific options for generating debug info 

#define SAVE_SAMPLES      0  // 1 = store microphone samples for review

//----------- debug output defs

#define Trace(x)

#define FDEBUG_FILE     "pcfrontend.dbg"
#define SDEBUG_FILE	    "pcsearch.dbg"
#define NDEBUG_FILE     "pcnnet.dbg"
#ifdef _MSC_VER
#define ALIGNED(x)
#else
#define ALIGNED(x) __attribute__ ((aligned (x)))
#endif

typedef void (*MemcpyFunction)(void * dest, const void * src, size_t size);
typedef BOOL (*IsFlashFunction)(const void * src);

#endif // SENSORYTYPES_H_INCLUDED
