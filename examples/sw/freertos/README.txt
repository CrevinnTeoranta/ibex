
# Boot address is set up as following in the file: ibex_if_stage.sv

 // fetch address selection mux
  always_comb begin : fetch_addr_mux
    unique case (pc_mux_internal)
      PC_BOOT: fetch_addr_n = { boot_addr_i[31:8], 8'h80 };

This is why the reset vector is set to 0x80 using "org" command in "led/crt0.S" 

An equivilent change has been applied to "startup_RV32M1_zero_riscy.S" 
    .org 0x80
    jal x0, Reset_Handler

