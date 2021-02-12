// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

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

  pcount_enable(0);

  // Enable periodic timer interrupt
  // (the actual timebase is a bit meaningless in simulation)
  timer_enable(500);

  uint64_t last_elapsed_time = get_elapsed_time();

  while (last_elapsed_time <= 4) {
    uint32_t toe_read = 0;
    uint32_t dma_read = 0;
    uint64_t cur_time = get_elapsed_time();
    if (cur_time != last_elapsed_time) {
      last_elapsed_time = cur_time;

      if (last_elapsed_time & 1) {
        puts("Tick! (Writing to ToE, Reading from DMA)\n");
        DEV_WRITE(TOE_BASE, 0x0000DEAD + toe_read);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_CONTROL, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_STATUS, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_IRQ_MASK, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_IRQ_STATUS, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_READER_START_ADDR, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_READER_LINE_LEN, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_READER_LINE_COUNT, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_READER_STRIDE, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_WRITER_START_ADDR, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_WRITER_LINE_LEN, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_WRITER_LINE_COUNT, 0);
        dma_read = DEV_READ(DMA_CTRL_BASE + DMA_CTRL_WRITER_STRIDE, 0);
      } else {
        puts("Tock! (Reading from ToE)\n");
        toe_read = DEV_READ(TOE_BASE, 0);
        DEV_WRITE(DMA_CTRL_BASE + DMA_CTRL_IRQ_MASK, dma_read);
      }
    }
    asm volatile("wfi"); // WFI = Wait For Interrupt
  }

  return 0;
}
