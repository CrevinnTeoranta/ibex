// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "simple_system_common.h"
#include "custom-includes.h"

int main(int argc, char **argv) {
  pcount_enable(0);
  pcount_reset();
  pcount_enable(1);

  puts("Hello simple system\n");
  puthex(0xDEADBEEF);
  putchar('\n');
  puthex(0xBAADF00D);
  putchar('\n');

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

  // Disable the timer, we don't want any other interrupts while we perform DMA transfer
  timer_disable();

  // Set up the fast-vdma
  fast_vdma_setup_chan(FAST_VDMA_MEM_BASE, 0x00040000, 0x00100800, 32, 32, 0);

  // Configure and enable external interrupts
  cfg_ext_irq(1);

  // Initiate the DMA transfer
  fast_vdma_start(FAST_VDMA_MEM_BASE, 0, 1);

  asm volatile("wfi\nwfi\n"); // Wait For 2 Interrupts, one from reader one from writer

  // Disable the fastvdma
  fast_vdma_stop(FAST_VDMA_MEM_BASE);

  return 0;
}
