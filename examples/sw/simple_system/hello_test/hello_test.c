// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "custom-includes.h"
#include "simple_system_common.h"

int main(int argc, char **argv) {
  pcount_enable(0);
  pcount_reset();
  pcount_enable(1);

  puts("Hello simple system\n");
  puthex(0xDEADBEEF);
  putchar('\n');
  puthex(0xBAADF00D);
  putchar('\n');

  // Set up the fast-vdma
  fast_vdma_setup_chan((uint32_t*)FAST_VDMA_MEM_BASE, 0x00100800, 0x00100900, 32, 32, 32);

  pcount_enable(0);

  // Enable periodic timer interrupt
  // (the actual timebase is a bit meaningless in simulation)
  timer_enable(1000);

  uint64_t last_elapsed_time = get_elapsed_time();
  uint32_t toe_read = 0;

  while (last_elapsed_time <= 4) {
    uint64_t cur_time = get_elapsed_time();
    if (cur_time != last_elapsed_time) {
      last_elapsed_time = cur_time;

      if (last_elapsed_time & 1) {
        puts("Tick! (Writing to ToE, Reading from DMA)\n");
        DEV_WRITE(TOE_BASE, 0x0000DEAD + toe_read);
      } else {
        puts("Tock! (Reading from ToE)\n");
        toe_read = DEV_READ(TOE_BASE, 0);
      }
    }
    asm volatile("wfi"); // WFI = Wait For Interrupt
  }

  return 0;
}
