  
# Changes made to files from the RISC-V_RV32M1_Vega_GCC_Eclipse RTOS demo   
  1. linker_RTOSDemo_zero_riscy.ld  (Based on: FreeRTOSv202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/projects/RTOSDemo_ri5cy/RV32M1_ri5cy_flash.ld)  
  2. startup_RV32M1_zero_riscy.S   (Based on: FreeRTOSv202012.00/FreeRTOS/Demo/RISC-V_RV32M1_Vega_GCC_Eclipse/common/rv32m1_sdk_riscv/devices/RV32M1/gcc folder)    
  
# Boot address is set up as following in the file: ibex_if_stage.sv  
  
 // fetch address selection mux  
  always_comb begin : fetch_addr_mux  
    unique case (pc_mux_internal)  
      PC_BOOT: fetch_addr_n = { boot_addr_i[31:8], 8'h80 };  

This is why the reset vector is set to 0x80 using "org" command in "led/crt0.S"   
  
An equivilent change has been applied to "startup_RV32M1_zero_riscy.S"   
    .org 0x80  
    jal x0, Reset_Handler  
  
