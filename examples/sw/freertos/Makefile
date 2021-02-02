# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Generate a baremetal application

# riscv32-unknown-elf-gcc -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall -g -x assembler-with-cpp -DDEBUG -DportasmHANDLE_INTERRUPT=SystemIrqHandler -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Source\portable\GCC\RISC-V\chip_specific_extensions\Pulpino_Vega_RV32M1RM -c -o full_demo\RegTest.o ..\full_demo\RegTest.S 

# riscv32-unknown-elf-gcc -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall -g -DDEBUG -DCPU_RV32M1_ri5cy -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -I../../../common -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Demo\Common\include -I../../../common/rv32m1_sdk_riscv/board -I../../../common/rv32m1_sdk_riscv/RISCV -I../../../common/rv32m1_sdk_riscv/devices/RV32M1 -I../../../common/rv32m1_sdk_riscv/devices/RV32M1/utilities -I../../../common/rv32m1_sdk_riscv/devices/RV32M1/drivers -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Demo\RISC-V_RV32M1_Vega_GCC_Eclipse\projects\RTOSDemo_ri5cy -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Source\portable\GCC\RISC-V -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Source\include -std=gnu99 -march=rv32imcxpulpv2 -Wa,-march=rv32imcxpulpv2 -c -o full_demo\common_demo_files\AbortDelay.o ..\..\..\..\Common\Minimal\AbortDelay.c 

# riscv32-unknown-elf-gcc -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall -g -T C:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Demo\RISC-V_RV32M1_Vega_GCC_Eclipse\projects\RTOSDemo_ri5cy\RV32M1_ri5cy_flash.ld -nostdlib -Xlinker --gc-sections -Wl,-Map,RTOSDemo_ri5cy.map -march=rv32imcxpulpv2 -Xlinker -static -Xlinker -z -Xlinker muldefs -Xlinker -static -Xlinker -z -Xlinker muldefs -o RTOSDemo_ri5cy.elf -Xlinker --start-group FreeRTOS_Source\event_groups.o FreeRTOS_Source\list.o FreeRTOS_Source\portable\GCC\RISC-V\port.o FreeRTOS_Source\portable\GCC\RISC-V\portASM.o FreeRTOS_Source\portable\MemMang\heap_4.o FreeRTOS_Source\queue.o FreeRTOS_Source\stream_buffer.o FreeRTOS_Source\tasks.o FreeRTOS_Source\timers.o startup_RV32M1_ri5cy.o blinky_demo\main_blinky.o common\pin_mux.o common\rv32m1_sdk_riscv\board\board.o common\rv32m1_sdk_riscv\board\clock_config.o common\rv32m1_sdk_riscv\devices\RV32M1\drivers\fsl_clock.o common\rv32m1_sdk_riscv\devices\RV32M1\drivers\fsl_common.o common\rv32m1_sdk_riscv\devices\RV32M1\drivers\fsl_gpio.o common\rv32m1_sdk_riscv\devices\RV32M1\drivers\fsl_lpuart.o common\rv32m1_sdk_riscv\devices\RV32M1\drivers\fsl_msmc.o common\rv32m1_sdk_riscv\devices\RV32M1\system_RV32M1_ri5cy.o common\rv32m1_sdk_riscv\devices\RV32M1\utilities\fsl_debug_console.o common\rv32m1_sdk_riscv\devices\RV32M1\utilities\fsl_io.o common\rv32m1_sdk_riscv\devices\RV32M1\utilities\fsl_log.o common\rv32m1_sdk_riscv\devices\RV32M1\utilities\fsl_str.o full_demo\RegTest.o full_demo\common_demo_files\AbortDelay.o full_demo\common_demo_files\EventGroupsDemo.o full_demo\common_demo_files\GenQTest.o full_demo\common_demo_files\MessageBufferDemo.o full_demo\common_demo_files\StreamBufferDemo.o full_demo\common_demo_files\StreamBufferInterrupt.o full_demo\common_demo_files\TaskNotify.o full_demo\common_demo_files\TimerDemo.o full_demo\common_demo_files\blocktim.o full_demo\common_demo_files\countsem.o full_demo\common_demo_files\death.o full_demo\common_demo_files\dynamic.o full_demo\common_demo_files\recmutex.o full_demo\main_full.o main.o -lnosys -lgcc -lc -lm -lm -lg -lgcc -lnosys -Xlinker --end-group 
# Info: Parallel threads used: 4

# riscv32-unknown-elf-gcc -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall -g -DDEBUG -DCPU_RV32M1_ri5cy -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -I../../../common -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Demo\Common\include -I../../../common/rv32m1_sdk_riscv/board -I../../../common/rv32m1_sdk_riscv/RISCV -I../../../common/rv32m1_sdk_riscv/devices/RV32M1 -I../../../common/rv32m1_sdk_riscv/devices/RV32M1/utilities -I../../../common/rv32m1_sdk_riscv/devices/RV32M1/drivers -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Demo\RISC-V_RV32M1_Vega_GCC_Eclipse\projects\RTOSDemo_ri5cy -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Source\portable\GCC\RISC-V -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Source\include -std=gnu99 -march=rv32imcxpulpv2 -Wa,-march=rv32imcxpulpv2 -c

#
# Taken from Eclipse RTOSDEMO_ri5cy
#   C-files with the following gcc args: 
#     -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin 
#     -Wall -g -DDEBUG -DCPU_RV32M1_ri5cy -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0
#     -std=gnu99 -march=rv32imcxpulpv2 -Wa,-march=rv32imcxpulpv2 -c
#   C-files with the following gcc incdirs: 
#     -I../../../common -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Demo\Common\include -I../../../common/rv32m1_sdk_riscv/board 
#     -I../../../common/rv32m1_sdk_riscv/RISCV -I../../../common/rv32m1_sdk_riscv/devices/RV32M1 -I../../../common/rv32m1_sdk_riscv/devices/RV32M1/utilities 
#     -I../../../common/rv32m1_sdk_riscv/devices/RV32M1/drivers 
#     -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Demo\RISC-V_RV32M1_Vega_GCC_Eclipse\projects\RTOSDemo_ri5cy 
#     -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Source\portable\GCC\RISC-V 
#     -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Source\include
#
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\drivers\fsl_lpuart.c
#  ..\..\..\..\Common\Minimal\death.c
#  ..\..\..\..\..\Source\portable\MemMang\heap_4.c
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\drivers\fsl_gpio.c
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\drivers\fsl_msmc.c
#  ..\..\..\..\..\Source\timers.c
#  ..\..\..\..\Common\Minimal\EventGroupsDemo.c
#  ..\..\..\..\Common\Minimal\recmutex.c
#  ..\full_demo\main_full.c
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\utilities\fsl_str.c
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\utilities\fsl_io.c
#  ..\..\..\common\rv32m1_sdk_riscv\board\clock_config.c
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\drivers\fsl_common.c
#  ..\..\..\..\Common\Minimal\TaskNotify.c
#  ..\..\..\..\..\Source\list.c
#  ..\..\..\..\Common\Minimal\StreamBufferInterrupt.c
#  ..\..\..\..\..\Source\event_groups.c
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\utilities\fsl_log.c
#  ..\..\..\common\pin_mux.c
#  ..\..\..\..\Common\Minimal\blocktim.c
#  ..\main.c
#  ..\..\..\common\rv32m1_sdk_riscv\board\board.c
#  ..\blinky_demo\main_blinky.c
#  ..\..\..\..\Common\Minimal\MessageBufferDemo.c
#  ..\..\..\..\..\Source\tasks.c
#  ..\..\..\..\Common\Minimal\TimerDemo.c
#  ..\..\..\..\Common\Minimal\GenQTest.c
#  ..\..\..\..\Common\Minimal\countsem.c
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\utilities\fsl_debug_console.c
#  ..\..\..\..\..\Source\stream_buffer.c
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\drivers\fsl_clock.c
#  ..\..\..\..\..\Source\queue.c
#  ..\..\..\..\Common\Minimal\StreamBufferDemo.c
#  ..\..\..\..\Common\Minimal\dynamic.c
#  ..\..\..\..\..\Source\portable\GCC\RISC-V\port.c
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\system_RV32M1_ri5cy.c
#  ..\..\..\..\Common\Minimal\AbortDelay.c

# riscv32-unknown-elf-gcc -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall -g -x assembler-with-cpp -DDEBUG -DportasmHANDLE_INTERRUPT=SystemIrqHandler -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Source\portable\GCC\RISC-V\chip_specific_extensions\Pulpino_Vega_RV32M1RM -c -o startup_RV32M1_ri5cy.o ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\gcc\startup_RV32M1_ri5cy.S 
#
# Taken from Eclipse RTOSDEMO_ri5cy
#   ASM-files with the following gcc args: 
#     -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding 
#     -fno-builtin -Wall -g -x assembler-with-cpp 
#     -DDEBUG -DportasmHANDLE_INTERRUPT=SystemIrqHandler
#   ASM-files with the following gcc incdirs: 
#     -IC:\Users\PaulOK\Documents\projects\CreVinn\riscv_ibex\FreeRTOSv202012.00\FreeRTOS\Source\portable\GCC\RISC-V\chip_specific_extensions\Pulpino_Vega_RV32M1RM
# 
#  ..\..\..\common\rv32m1_sdk_riscv\devices\RV32M1\gcc\startup_RV32M1_ri5cy.S
#  ..\full_demo\RegTest.S
#  ..\..\..\..\..\Source\portable\GCC\RISC-V\portASM.S


# ../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1/system_RV32M1_ri5cy.c
# ../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1/gcc/startup_RV32M1_ri5cy.S


#COMMON_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
#FREERTOS_SRC_DIR_1=$(shell find ../../../../../../../freertos_202012.00/FreeRTOS/Source/ -type d)

# ../../../../../../../freertos_202012.00/FreeRTOS/Source/
# ../../../../../../../freertos_202012.00/FreeRTOS/Source/portable/GCC/RISC-V/
# ../../../../../../../freertos_202012.00/FreeRTOS/Source/include/
# ../../../../../../../freertos_202012.00/FreeRTOS/Source/portable/GCC/RISC-V/chip_specific_extensions/Pulpino_Vega_RV32M1RM/
FREERTOS_C_SRC_DIR_1=../../../../../../../freertos_202012.00/FreeRTOS/Source
FREERTOS_C_SRC_DIR_2=../../../../../../../freertos_202012.00/FreeRTOS/Source/portable/MemMang
FREERTOS_C_SRC_DIR_3=../../../../../../../freertos_202012.00/FreeRTOS/Source/portable/GCC/RISC-V
FREERTOS_C_SRC_DIR_4=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/projects/RTOSDemo_ri5cy/blinky_demo
FREERTOS_C_SRC_DIR_5=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/projects/RTOSDemo_ri5cy/full_demo
FREERTOS_C_SRC_DIR_6=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/projects/RTOSDemo_ri5cy
FREERTOS_C_SRC_DIR_7=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1/utilities
FREERTOS_C_SRC_DIR_8=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1/drivers
FREERTOS_C_SRC_DIR_9=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1
FREERTOS_C_SRC_DIR_10=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/board
FREERTOS_C_SRC_DIR_11=../../../../../../../freertos_202012.00/FreeRTOS/Demo/Common/Minimal
FREERTOS_C_SRC_DIR_12=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common
FREERTOS_C_SRCS_1 = $(wildcard $(FREERTOS_C_SRC_DIR_1)/*.c)
FREERTOS_C_SRCS_2 = $(wildcard $(FREERTOS_C_SRC_DIR_2)/*.c)
FREERTOS_C_SRCS_3 = $(wildcard $(FREERTOS_C_SRC_DIR_3)/*.c)
FREERTOS_C_SRCS_4 = $(wildcard $(FREERTOS_C_SRC_DIR_4)/*.c)
FREERTOS_C_SRCS_5 = $(wildcard $(FREERTOS_C_SRC_DIR_5)/*.c)
FREERTOS_C_SRCS_6 = $(wildcard $(FREERTOS_C_SRC_DIR_6)/*.c)
FREERTOS_C_SRCS_7 = $(wildcard $(FREERTOS_C_SRC_DIR_7)/*.c)
FREERTOS_C_SRCS_8 = $(wildcard $(FREERTOS_C_SRC_DIR_8)/*.c)
#FREERTOS_C_SRCS_9 = $(FREERTOS_C_SRC_DIR_9)/system_RV32M1_ri5cy.c)
FREERTOS_C_SRCS_9 = $(FREERTOS_C_SRC_DIR_9)/system_RV32M1_zero_riscy.c
FREERTOS_C_SRCS_10 = $(wildcard $(FREERTOS_C_SRC_DIR_10)/*.c)
#FREERTOS_C_SRCS_11 = $(wildcard $(FREERTOS_C_SRC_DIR_11)/*.c)
FREERTOS_C_SRCS_12 = $(wildcard $(FREERTOS_C_SRC_DIR_12)/*.c)

FREERTOS_C_INC_DIR_1=../../../../../../../freertos_202012.00/FreeRTOS/Source/include
FREERTOS_C_INC_DIR_2=../../../../../../../freertos_202012.00/FreeRTOS/Source/portable/GCC/RISC-V
FREERTOS_C_INC_DIR_3=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/projects/RTOSDemo_ri5cy
FREERTOS_C_INC_DIR_4=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common
FREERTOS_C_INC_DIR_5=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/board
FREERTOS_C_INC_DIR_6=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1/drivers
FREERTOS_C_INC_DIR_7=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1/utilities
FREERTOS_C_INC_DIR_8=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1
FREERTOS_C_INC_DIR_9=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/RISCV
FREERTOS_C_INC_DIR_10=../../../../../../../freertos_202012.00/FreeRTOS/Demo/Common/include

#FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1/

FREERTOS_ASM_SRC_DIR_1=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1/gcc
FREERTOS_ASM_SRC_DIR_2=../../../../../../../freertos_202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/projects/RTOSDemo_ri5cy/full_demo
FREERTOS_ASM_SRC_DIR_3=../../../../../../../freertos_202012.00/FreeRTOS/Source/portable/GCC/RISC-V
#FREERTOS_ASM_SRCS_1 = $(FREERTOS_ASM_SRC_DIR_1)/startup_RV32M1_ri5cy.S
FREERTOS_ASM_SRCS_1 =$(FREERTOS_ASM_SRC_DIR_1)/startup_RV32M1_zero_riscy.S
FREERTOS_ASM_SRCS_2 = $(wildcard $(FREERTOS_ASM_SRC_DIR_2)/*.S)
FREERTOS_ASM_SRCS_3 = $(wildcard $(FREERTOS_ASM_SRC_DIR_3)/*.S)

FREERTOS_ASM_INC_DIR_1=../../../../../../../freertos_202012.00/FreeRTOS/Source/portable/GCC/RISC-V/chip_specific_extensions/Pulpino_Vega_RV32M1RM


#FREERTOS_SRCS_1 = $(wildcard Source/*.c)
#FREERTOS_SRCS_2 = $(wildcard Source/portable/*.c)

INCS := -I$(FREERTOS_C_INC_DIR_1) -I$(FREERTOS_C_INC_DIR_2) -I$(FREERTOS_C_INC_DIR_3) -I$(FREERTOS_C_INC_DIR_4) -I$(FREERTOS_C_INC_DIR_5) -I$(FREERTOS_C_INC_DIR_6) -I$(FREERTOS_C_INC_DIR_7) -I$(FREERTOS_C_INC_DIR_8) -I$(FREERTOS_C_INC_DIR_9) -I$(FREERTOS_C_INC_DIR_10) -I$(FREERTOS_ASM_INC_DIR_1)

# This is used in fsl_device_registers.h: CPU_RV32M1_zero_riscy

DEF_CPU_INT_HANDLER = -DportasmHANDLE_INTERRUPT=SystemIrqHandler

DEF_CPU_TYPE = -DCPU_RV32M1_zero_riscy
#DEF_CPU_TYPE = -DCPU_RV32M1_ri5cy

PROGRAM ?= RTOSDemo_zero_riscy
#PROGRAM ?= RTOSDemo_ri5cy
PROGRAM_CFLAGS = -Wall -g -Os 
PROGRAM_CFLAGS_EXT = -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall -g -DDEBUG -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -std=gnu99 -c

ARCH = rv32imc
# ARCH = rv32im # to disable compressed instructions
#SRCS = $(PROGRAM).c

#  ..\..\..\..\Common\Minimal\death.c
#  ..\..\..\..\Common\Minimal\EventGroupsDemo.c
#  ..\..\..\..\Common\Minimal\recmutex.c
#  ..\..\..\..\Common\Minimal\TaskNotify.c
#  ..\..\..\..\Common\Minimal\StreamBufferInterrupt.c
#  ..\..\..\..\Common\Minimal\blocktim.c
#  ..\..\..\..\Common\Minimal\MessageBufferDemo.c
#  ..\..\..\..\Common\Minimal\TimerDemo.c
#  ..\..\..\..\Common\Minimal\GenQTest.c
#  ..\..\..\..\Common\Minimal\countsem.c
#  ..\..\..\..\Common\Minimal\StreamBufferDemo.c
#  ..\..\..\..\Common\Minimal\dynamic.c
#  ..\..\..\..\Common\Minimal\AbortDelay.c
EXTRA_SRCS = $(FREERTOS_C_SRC_DIR_11)/death.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/EventGroupsDemo.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/recmutex.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/TaskNotify.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/StreamBufferInterrupt.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/blocktim.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/MessageBufferDemo.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/TimerDemo.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/GenQTest.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/countsem.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/StreamBufferDemo.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/dynamic.c
EXTRA_SRCS += $(FREERTOS_C_SRC_DIR_11)/AbortDelay.c

SRCS = $(FREERTOS_C_SRCS_1) $(FREERTOS_C_SRCS_2) $(FREERTOS_C_SRCS_3) $(FREERTOS_C_SRCS_4) $(FREERTOS_C_SRCS_5) $(FREERTOS_C_SRCS_6) $(FREERTOS_C_SRCS_7) $(FREERTOS_C_SRCS_8) $(FREERTOS_C_SRCS_9) $(FREERTOS_C_SRCS_10) $(FREERTOS_C_SRCS_12) $(EXTRA_SRCS)
C_SRCS = $(filter %.c, $(SRCS))

CC = riscv32-unknown-elf-gcc

OBJCOPY ?= $(subst gcc,objcopy,$(wordlist 1,1,$(CC)))
OBJDUMP ?= $(subst gcc,objdump,$(wordlist 1,1,$(CC)))

#LINKER_SCRIPT ?= link.ld
LINKER_SCRIPT ?= RV32M1_ri5cy_flash.ld

#CRT ?= crt0.S
#CRT ?= Source/portable/portASM.S
ASM_SRCS_DIRS = $(FREERTOS_ASM_SRCS_1) $(FREERTOS_ASM_SRCS_2) $(FREERTOS_ASM_SRCS_3) 
ASM_SRCS = $(filter %.S, $(ASM_SRCS_DIRS))

CFLAGS ?= -march=$(ARCH) -mabi=ilp32 -static -mcmodel=medany \
	-fvisibility=hidden -nostdlib -nostartfiles $(PROGRAM_CFLAGS) $(PROGRAM_CFLAGS_EXT) $(DEF_CPU_INT_HANDLER) $(DEF_CPU_TYPE)
LDFLAGS ?= -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall -g \
	-nostdlib -Xlinker --gc-sections -Wl,-Map,$(PROGRAM).map -march=$(ARCH) -Xlinker -static -Xlinker -z -Xlinker muldefs -Xlinker -static -Xlinker -z -Xlinker muldefs
LDFLAGS_EXT ?= -lnosys -lgcc -lc -lm -lm -lg -lgcc -lnosys -Xlinker --end-group

OBJS := ${C_SRCS:.c=.o} ${ASM_SRCS:.S=.o}
DEPS = $(OBJS:%.o=%.d)

OUTFILES = $(PROGRAM).elf $(PROGRAM).vmem $(PROGRAM).bin $(PROGRAM).dis

all: $(OUTFILES)

$(PROGRAM).elf: $(OBJS) $(LINKER_SCRIPT)
	$(CC) $(LDFLAGS) -T $(LINKER_SCRIPT) -Xlinker --start-group $(OBJS) -o $@ $(LIBS) $(LDFLAGS_EXT)

%.dis: %.elf
	$(OBJDUMP) -SD $^ > $@

# Note: this target requires the srecord package to be installed.
# XXX: This could be replaced by objcopy once
# https://sourceware.org/bugzilla/show_bug.cgi?id=19921
# is widely available.
# XXX: Currently the start address 0x00000000 is hardcoded. It could/should be
# read from the elf file, but is lost in the bin file.
# Switching to objcopy will resolve that as well.
%.vmem: %.bin
	srec_cat $^ -binary -offset 0x0000 -byte-swap 4 -o $@ -vmem

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

%.o: %.c
	$(CC) $(CFLAGS) -MMD -c $(INCS) -o $@ $<

%.o: %.S
	$(CC) $(CFLAGS) -MMD -c $(INCS) -o $@ $<

clean:
	$(RM) -f *.o *.d

distclean: clean
	$(RM) -f $(OUTFILES)
