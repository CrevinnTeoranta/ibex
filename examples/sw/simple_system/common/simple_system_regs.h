// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef SIMPLE_SYSTEM_REGS_H__
#define SIMPLE_SYSTEM_REGS_H__

#define SIM_CTRL_BASE 0x20000
#define SIM_CTRL_OUT 0x0
#define SIM_CTRL_CTRL 0x8

#define TIMER_BASE 0x30000
#define TIMER_MTIME 0x0
#define TIMER_MTIMEH 0x4
#define TIMER_MTIMECMP 0x8
#define TIMER_MTIMECMPH 0xC

#define TOE_BASE 0x40000

#define DMA_CTRL_BASE 0x50000
#define DMA_CTRL_CONTROL    0x00
#define DMA_CTRL_STATUS     0x04
#define DMA_CTRL_IRQ_MASK   0x08
#define DMA_CTRL_IRQ_STATUS 0x0c
#define DMA_CTRL_READER_START_ADDR 0x10
#define DMA_CTRL_READER_LINE_LEN   0x14
#define DMA_CTRL_READER_LINE_COUNT 0x18
#define DMA_CTRL_READER_STRIDE     0x1c
#define DMA_CTRL_WRITER_START_ADDR 0x20
#define DMA_CTRL_WRITER_LINE_LEN   0x24
#define DMA_CTRL_WRITER_LINE_COUNT 0x28
#define DMA_CTRL_WRITER_STRIDE     0x2c

#endif  // SIMPLE_SYSTEM_REGS_H__
