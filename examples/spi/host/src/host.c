// Copyright 2016-2021 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#include <stdio.h>
#include <stdlib.h>
#include "control_host.h"
#include "signals.h"
#include "resource.h"
#include "util.h"
#include <bcm2835.h>

int done = 0;

void shutdown(void)
{
  done = 1;
}

int main(void)
{
  control_version_t version = 0xFF;
  unsigned char payload[4];
  uint8_t i;

  signals_init();
  signals_setup_int(shutdown);

  if (control_init_spi_pi(SPI_MODE_3, BCM2835_SPI_CLOCK_DIVIDER_1024, 2) != CONTROL_SUCCESS) {
    printf("control init failed\n");
    exit(1);
  }

  printf("device found\n");

  if (control_query_version(&version) != CONTROL_SUCCESS) {
    printf("control query version failed\n");
    exit(1);
  }
  if (version != CONTROL_VERSION) {
    printf("version expected 0x%X, received 0x%X\n", CONTROL_VERSION, version);
  }

  printf("started\n");

  for (i = 0; i < 4; i++) {
    payload[0] = i;
    if (control_write_command(RESOURCE_ID, CONTROL_CMD_SET_WRITE(0), payload, 1) != CONTROL_SUCCESS) {
      printf("control write command failed\n");
      exit(1);
    }
    fflush(stdout);

    pause_short();

    if (control_read_command(RESOURCE_ID, CONTROL_CMD_SET_READ(0), payload, 1) != CONTROL_SUCCESS) {
      printf("control read command failed\n");
      exit(1);
    }

    if (payload[0] != i) {
      printf("control read command returned the wrong value, expected %d, returned %d\n", i, payload[0]);
      exit(1);
    }
    fflush(stdout);
  }

  control_cleanup_spi();
  printf("done\n");

  return 0;
}