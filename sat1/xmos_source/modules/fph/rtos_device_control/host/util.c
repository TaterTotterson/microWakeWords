// Copyright 2016-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#include <stdio.h>
#ifdef _WIN32
#include <windows.h>
#elif __xcore__
#include <xs1.h>
#include <timer.h>
#else
#include <unistd.h>
#endif
#include "util.h"

#ifdef _WIN32

void pause_short(void)
{
  Sleep(0);
}

void pause_long(void)
{
  Sleep(1000);
}

#elif __xcore__

void pause_short(void)
{
  delay_milliseconds(100);
}

void pause_long(void)
{
  delay_milliseconds(1000);
}

#else

void pause_short(void)
{
  usleep(0);
}

void pause_long(void)
{
  sleep(1);
}

#endif // _WIN32

void print_bytes(const unsigned char data[], int num_bytes)
{
  int i;
  for (i = 0; i < num_bytes; i++) {
    printf("%02x ", data[i]);
  }
  printf("\n");
}
