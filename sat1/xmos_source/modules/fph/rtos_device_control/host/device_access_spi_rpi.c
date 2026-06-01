// Copyright 2017-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#if USE_SPI && RPI

#include <stdio.h>
#include <unistd.h>
#include "device_control_host.h"
#include "control_host_support.h"
#include "bcm2835.h"
#include <time.h>

//#define DBG(x) x
#define DBG(x)
#define PRINT_ERROR(...)   fprintf(stderr, "Error  : " __VA_ARGS__)

// Number of nsec to delay between spi transactions
static long intertransaction_delay;

// Sleep for intertransaction_delay nanoseconds. Yields to the OS so expect minimum delay to be hundreds
// of microseconds at least.
static void apply_intertransaction_delay() {
	if(0 < intertransaction_delay) {
		struct timespec req = {0, intertransaction_delay};
		struct timespec rem;
		while(nanosleep(&req, &rem));
	}
}

control_ret_t
control_init_spi_pi(spi_mode_t spi_mode, bcm2835SPIClockDivider clock_divider, long delay_ns)
{
  if(!bcm2835_init() ||
     !bcm2835_spi_begin()) {
    fprintf(stderr, "BCM2835 initialisation failed. Possibly not running as root\n");
    return CONTROL_ERROR;
  }

  intertransaction_delay = delay_ns;
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(spi_mode);
  bcm2835_spi_setClockDivider(clock_divider);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
	
  return CONTROL_SUCCESS;
}
control_ret_t
control_write_command(control_resid_t resid, control_cmd_t cmd,
                      const uint8_t payload[], size_t payload_len)
{
  uint8_t data_sent_recieved[SPI_TRANSACTION_MAX_BYTES];

  do
  {
      int data_len;
// When LOW_LEVEL_TESTING is defined, resid and cmd fields are ignored and payload is sent directly to the device.
// This allows the user to write a stream of bytes directly into the device allowing for low level testing like testing the
// error handling mechanism.
#if LOW_LEVEL_TESTING
      if((resid == 0) && (cmd == 0))
      {
        memcpy(data_sent_recieved, payload, payload_len);
        data_len = payload_len;
      }
      else
      {
        data_len = control_build_spi_data(data_sent_recieved, resid, cmd, payload, payload_len);
      }
#else
      data_len = control_build_spi_data(data_sent_recieved, resid, cmd, payload, payload_len);
#endif
      bcm2835_spi_transfern((char *)data_sent_recieved, data_len);
      apply_intertransaction_delay();
  }while(data_sent_recieved[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE);

  do
  {
      // get status
      memset(data_sent_recieved, 0, SPI_TRANSACTION_MAX_BYTES);
      unsigned transaction_length = payload_len < 8 ? 8 : payload_len;  
      bcm2835_spi_transfern((char *)data_sent_recieved, payload_len);
      apply_intertransaction_delay();
  }while(data_sent_recieved[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE);
  //printf("data_sent_recieved[0] = 0x%x, 0x%x, 0x%x, 0x%x\n",data_sent_recieved[0], data_sent_recieved[1], data_sent_recieved[2], data_sent_recieved[3]);
  // data_sent_received[0] contains write command status so return it.
  return data_sent_recieved[0];
}

control_ret_t
control_read_command(control_resid_t resid, control_cmd_t cmd,
                     uint8_t payload[], size_t payload_len)
{
  uint8_t data_sent_recieved[SPI_TRANSACTION_MAX_BYTES] = {0};
  //printf("control_read_command(): resid 0x%x, cmd_id 0x%x, payload_len 0x%x\n",resid, cmd, payload_len);
  do
  {
      int data_len;
#if LOW_LEVEL_TESTING
      if((resid == 0) && (cmd == 0))
      {
        memcpy(data_sent_recieved, payload, payload_len);
        data_len = payload_len;
      }
      else
      {
        data_len = control_build_spi_data(data_sent_recieved, resid, cmd, payload, payload_len);
      }
#else
      data_len = control_build_spi_data(data_sent_recieved, resid, cmd, payload, payload_len);
#endif
      bcm2835_spi_transfern((char *)data_sent_recieved, data_len);
      apply_intertransaction_delay();

  }while(data_sent_recieved[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE);
  
  do
  {
      memset(data_sent_recieved, 0, SPI_TRANSACTION_MAX_BYTES);
      unsigned transaction_length = payload_len < 8 ? 8 : payload_len;  

      bcm2835_spi_transfern((char *)data_sent_recieved, payload_len);
      apply_intertransaction_delay();
  }while(data_sent_recieved[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE);

  //printf("data_sent_recieved[0] = 0x%x, 0x%x, 0x%x, 0x%x\n",data_sent_recieved[0], data_sent_recieved[1], data_sent_recieved[2], data_sent_recieved[3]);
  memcpy(payload, data_sent_recieved, payload_len);
  // TODO - For write commands, control_write_command() is returning status from the device. For read commands payload[0] has the
  // status from the device and control_read_command() always returns CONTROL_SUCCESS. Make status returning consistent across
  // for read and write command functions.
  return CONTROL_SUCCESS;
}

control_ret_t
control_cleanup_spi(void)
{
  bcm2835_spi_end();
  bcm2835_close();
  return CONTROL_SUCCESS;
}

#endif /* USE_SPI && RPI */
