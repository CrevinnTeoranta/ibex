////////////////////////////////////////////////////////////////////////////////
// Company:        IIS @ ETHZ - Federal Institute of Technology               //
//                 DEI @ UNIBO - University of Bologna                        //
//                                                                            //
// Engineer:       Renzo Andri - andrire@student.ethz.ch                      //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                                                                            //
//                                                                            //
// Create Date:    01/07/2014                                                 //
// Design Name:    Write Back stage                                           //
// Module Name:    wb_stage.sv                                                //
// Project Name:   OR10N                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Execution stage: hosts a Multiplexer that select data to   //
//                 write in the register file (from data interface or SP reg  //
//                                                                            //
// Revision:                                                                  //
// Revision v0.1 - File Created                                               //
// Revision v0.2 - (August 6th 2014) Changed port and signal names, addedd    //
//                 comments                                                   //
//                                                                            //
//                                                                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
// sp = special register
// id = instruction decode
// if = instruction fetch
// ex = execute stage
// wb = write back
// data = from data memory


`include "defines.sv"

module wb_stage
(
    // MUX SELECTOR --> Used to select what write in the register file
    input  logic        regfile_wdata_mux_sel_i,  // Comes from the controller (thru id-ex and ex-wb pipe)

    // MUX INPUTS
    input  logic [31:0] sp_rdata_i,              // From the read port of the special register
    input  logic [31:0] data_rdata_i,            // read Data from data memory system
    input  logic [31:0] lsu_data_reg_i,          // read data registered in LSU
    // MUX OUTPUT
    output logic [31:0] regfile_wdata_o,         // write data for register file
    output logic [31:0] wdata_reg_o,             // goes to pc_mux, origin is always a register!

    input  logic        eoc_i,
    output logic        eoc_o

);

   assign eoc_o = eoc_i;

   // Register Write Data Selection --> Data to write in the regfile
   // Select between:
   // 00,01: From EX stage (Alu Results)
   // 10:    From Special Register
   // 11:    From Data Memory
   always_comb
   begin : REGFILE_WDATA_MUX
      casex (regfile_wdata_mux_sel_i)
        1'b0:  begin regfile_wdata_o <= sp_rdata_i;        end
        1'b1:  begin regfile_wdata_o <= data_rdata_i;      end
      endcase; // case (regfile_wdata_mux_sel_i)
   end

   // wdata_reg_o is very similar to regfile_wdata_o, except that the
   // output of the LSU is registered. This signal is then used by the PC
   // mux instead of regfile_wdata_o in case forwarding is necessary
   always_comb
   begin : WDATA_FW_MUX
      casex (regfile_wdata_mux_sel_i)
        1'b0:  begin wdata_reg_o <= sp_rdata_i;        end
        1'b1:  begin wdata_reg_o <= lsu_data_reg_i;    end
      endcase; // case (regfile_wdata_mux_sel_i)
   end

endmodule