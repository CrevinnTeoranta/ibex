  
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
 
Make the following files (RTOSDemo_ibex.vmem is used by the xcelium sim and FPGA flow):   
  RTOSDemo_ibex.elf  
  RTOSDemo_ibex.map  
  RTOSDemo_ibex.bin  
  RTOSDemo_ibex.vmem  
  RTOSDemo_ibex.dis  
  RTOSDemo_ibex.out  
  
make clean  
make > & RTOSDemo_ibex.out  
  
Make the following files (For reference only, this is the demo output without IBEX modifications, c define used for this: CPU_RV32M1_ibex):   
  RTOSDemo_zero_riscy.elf  
  RTOSDemo_zero_riscy.map  
  RTOSDemo_zero_riscy.bin  
  RTOSDemo_zero_riscy.vmem  
  RTOSDemo_zero_riscy.dis  
  RTOSDemo_zero_riscy.out  
make clean  
make PROGRAM=RTOSDemo_zero_riscy > & RTOSDemo_zero_riscy.out  
   
