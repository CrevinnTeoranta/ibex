// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// VCS does not support overriding enum and string parameters via command line. Instead, a `define
// is used that can be set from the command line. If no value has been specified, this gives a
// default. Other simulators don't take the detour via `define and can override the corresponding
// parameters directly.
`ifndef RV32M
  `define RV32M ibex_pkg::RV32MFast
`endif

`ifndef RV32B
  `define RV32B ibex_pkg::RV32BNone
`endif

`ifndef RegFile
  `define RegFile ibex_pkg::RegFileFF
`endif

/**
 * Ibex simple system
 *
 * This is a basic system consisting of an ibex, a 1 MB sram for instruction/data
 * and a small memory mapped control module for outputting ASCII text and
 * controlling/halting the simulation from the software running on the ibex.
 *
 * It is designed to be used with verilator but should work with other
 * simulators, a small amount of work may be required to support the
 * simulator_ctrl module.
 */

module ibex_simple_system (
  input IO_CLK,
  input IO_RST_N
);

  parameter bit                 SecureIbex               = 1'b0;
  parameter bit                 PMPEnable                = 1'b0;
  parameter int unsigned        PMPGranularity           = 0;
  parameter int unsigned        PMPNumRegions            = 4;
  parameter bit                 RV32E                    = 1'b0;
  parameter ibex_pkg::rv32m_e   RV32M                    = `RV32M;
  parameter ibex_pkg::rv32b_e   RV32B                    = `RV32B;
  parameter ibex_pkg::regfile_e RegFile                  = `RegFile;
  parameter bit                 BranchTargetALU          = 1'b0;
  parameter bit                 WritebackStage           = 1'b0;
  parameter bit                 ICache                   = 1'b0;
  parameter bit                 ICacheECC                = 1'b0;
  parameter bit                 BranchPredictor          = 1'b0;
  parameter                     SRAMInitFile             = "";

  parameter bit                 HPipeLine                = 1'b0;

  import axi_pkg::*;
  import top_pkg::*;

  localparam int HFifoPass = HPipeLine ? 1'b0 : 1'b1;
  localparam int HFifoDepth = HPipeLine ? 4'h2 : 4'h0;

  logic clk_sys = 1'b0, rst_sys_n;

  typedef enum logic {
    CoreD,
    ToE_DMA
  } bus_host_e;

  typedef enum logic[2:0] {
    Ram,
    SimCtrl,
    Timer,
    ToE,
    DMACtrl // TODO priority
  } bus_device_e;

  localparam int NrDevices = 5;
  localparam int NrHosts = 1;
  localparam int NrDMAMasters = 1;

  // interrupts
  logic timer_irq;

  // host and device signals
  logic                        host_req    [NrHosts];
  logic                        host_gnt    [NrHosts];
  logic [top_pkg::AXI_AW -1:0] host_addr   [NrHosts];
  logic                        host_we     [NrHosts];
  logic [top_pkg::AXI_DSW-1:0] host_be     [NrHosts];
  logic [top_pkg::AXI_DW -1:0] host_wdata  [NrHosts];
  logic                        host_rvalid [NrHosts];
  logic [top_pkg::AXI_DW -1:0] host_rdata  [NrHosts];
  logic                        host_err    [NrHosts];

  // devices (slaves)
  logic                        device_req    [NrDevices];
  logic [top_pkg::AXI_AW -1:0] device_addr   [NrDevices];
  logic                        device_we     [NrDevices];
  logic [top_pkg::AXI_DSW-1:0] device_be     [NrDevices];
  logic [top_pkg::AXI_DW -1:0] device_wdata  [NrDevices];
  logic                        device_rvalid [NrDevices];
  logic [top_pkg::AXI_DW -1:0] device_rdata  [NrDevices];
  logic                        device_err    [NrDevices];

  // dma
  logic                        toe2ram_req;
  logic [top_pkg::AXI_AW -1:0] toe2ram_addr;
  logic                        toe2ram_we;
  logic [top_pkg::AXI_DSW-1:0] toe2ram_be;
  logic [top_pkg::AXI_DW -1:0] toe2ram_wdata;
  logic                        toe2ram_rvalid;
  logic [top_pkg::AXI_DW -1:0] toe2ram_rdata;
  logic                        toe2ram_err;

  // Device address mapping
  logic [top_pkg::AXI_AW-1:0] cfg_device_addr_base [NrDevices];
  logic [top_pkg::AXI_AW-1:0] cfg_device_addr_mask [NrDevices];
  assign cfg_device_addr_base[Ram] = 32'h100000;
  assign cfg_device_addr_mask[Ram] = ~32'hFFFFF; // 1 MB
  assign cfg_device_addr_base[SimCtrl] = 32'h20000;
  assign cfg_device_addr_mask[SimCtrl] = ~32'h3FF; // 1 kB
  assign cfg_device_addr_base[Timer] = 32'h30000;
  assign cfg_device_addr_mask[Timer] = ~32'h3FF; // 1 kB
  assign cfg_device_addr_base[ToE] = 32'h40000; // TODO address base
  assign cfg_device_addr_mask[ToE] = ~32'h1FFFF; // TODO memory size 64 kB
  assign cfg_device_addr_base[DMACtrl] = 32'h50000; // TODO address base
  assign cfg_device_addr_mask[DMACtrl] = ~32'h3F; // TODO memory size 32B

  // Instruction fetch signals
  logic instr_req;
  logic instr_gnt;
  logic instr_rvalid;
  logic [top_pkg::AXI_AW-1:0] instr_addr;
  logic [top_pkg::AXI_DW-1:0] instr_rdata;
  logic instr_err;

  assign instr_gnt = instr_req;
  assign instr_err = '0;

  `ifdef VERILATOR
    assign clk_sys = IO_CLK;
    assign rst_sys_n = IO_RST_N;
  `else
    initial begin
      rst_sys_n = 1'b0;
      #8
      rst_sys_n = 1'b1;
    end
    always begin
      #1 clk_sys = 1'b0;
      #1 clk_sys = 1'b1;
    end
  `endif

  // Tie-off unused error signals
  assign device_err[Ram] = 1'b0;
  assign device_err[SimCtrl] = 1'b0;
  assign toe2ram_err = 1'b0;

  // Connections from Host to Xbar
  axi_pkg::axi_h2d_t axi_d_host2xbar;
  axi_pkg::axi_d2h_t axi_d_xbar2host;

  // Connections from xbar to devices
  axi_pkg::axi_h2d_t axi_d_xbar2device[NrDevices];
  axi_pkg::axi_d2h_t axi_d_device2xbar[NrDevices];

  // Connections from Xbar and DMA Masters to DMA Controller
  axi_pkg::axi_h2d_t axi_d_master2dma[NrHosts + NrDMAMasters];
  axi_pkg::axi_d2h_t axi_d_dma2master[NrHosts + NrDMAMasters];

  // Connections from DMA Controller to RAM
  axi_pkg::axi_h2d_t axi_d_dma2ram;
  axi_pkg::axi_d2h_t axi_d_ram2dma;

  // DMA signals
  logic dma_irq_reader_done;
  logic dma_irq_writer_done;

  logic ext_irq;

  assign axi_d_master2dma[CoreD] = axi_d_xbar2device[Ram];
  assign axi_d_device2xbar[Ram] = axi_d_dma2master[CoreD];

  // Extra sim ctrl signals
  logic end_sim;

  ibex_core_tracing #(
      .SecureIbex      ( SecureIbex      ),
      .PMPEnable       ( PMPEnable       ),
      .PMPGranularity  ( PMPGranularity  ),
      .PMPNumRegions   ( PMPNumRegions   ),
      .MHPMCounterNum  ( 29              ),
      .RV32E           ( RV32E           ),
      .RV32M           ( RV32M           ),
      .RV32B           ( RV32B           ),
      .RegFile         ( RegFile         ),
      .BranchTargetALU ( BranchTargetALU ),
      .ICache          ( ICache          ),
      .ICacheECC       ( ICacheECC       ),
      .WritebackStage  ( WritebackStage  ),
      .BranchPredictor ( BranchPredictor ),
      .DmHaltAddr      ( 32'h00100000    ),
      .DmExceptionAddr ( 32'h00100000    )
    ) u_core (
      .clk_i                 (clk_sys),
      .rst_ni                (rst_sys_n),

      .test_en_i             (1'b0),

      .hart_id_i             (32'h0),
      // First instruction executed is at 0x0 + 0x80
      .boot_addr_i           (top_pkg::AXI_AW'('h00100000)),

      .instr_req_o           (instr_req),
      .instr_gnt_i           (instr_gnt),
      .instr_rvalid_i        (instr_rvalid),
      .instr_addr_o          (instr_addr),
      .instr_rdata_i         (instr_rdata),
      .instr_err_i           (instr_err),

      .data_req_o            (host_req[CoreD]),
      .data_gnt_i            (host_gnt[CoreD]),
      .data_rvalid_i         (host_rvalid[CoreD]),
      .data_we_o             (host_we[CoreD]),
      .data_be_o             (host_be[CoreD]),
      .data_addr_o           (host_addr[CoreD]),
      .data_wdata_o          (host_wdata[CoreD]),
      .data_rdata_i          (host_rdata[CoreD]),
      .data_err_i            (host_err[CoreD]),

      .irq_software_i        (1'b0),
      .irq_timer_i           (timer_irq),
      .irq_external_i        (ext_irq),
      .irq_fast_i            (15'b0),
      .irq_nm_i              (1'b0),

      .debug_req_i           (1'b0),

      .fetch_enable_i        (1'b1),
      .alert_minor_o         (),
      .alert_major_o         (),
      .core_sleep_o          ()
    );

  axi_adapter_host #(
      .MAX_REQS (1)
    ) axi_d_host2xbar_adapter (
      .clk_i   (clk_sys),
      .rst_ni  (rst_sys_n),

      .req_i   (host_req[CoreD]),
      .gnt_o   (host_gnt[CoreD]),
      .we_i    (host_we[CoreD]),
      .be_i    (host_be[CoreD]),
      .addr_i  (host_addr[CoreD]),
      .wdata_i (host_wdata[CoreD]),
      .valid_o (host_rvalid[CoreD]),
      .rdata_o (host_rdata[CoreD]),
      .err_o   (host_err[CoreD]),

      .axi_o   (axi_d_host2xbar),
      .axi_i   (axi_d_xbar2host)
    );

  // Create steering signal
  logic [2:0] dev_sel_s1n;
  always_comb begin
    // default steering to generate error response if address is not within the range
    dev_sel_s1n = 3'd5;
    for (int i = 0; i < NrDevices; i++) begin
      if ((axi_d_host2xbar.awaddr & cfg_device_addr_mask[i]) == cfg_device_addr_base[i]) begin
        dev_sel_s1n = i;
      end
    end
  end

  axi_socket_1n #(
    .N         (NrDevices),
    .HReqPass  (HFifoPass),
    .HRspPass  (HFifoPass),
    .HReqDepth (HFifoDepth),
    .HRspDepth (HFifoDepth),
    .DReqPass  ((NrDevices'('1))),
    .DRspPass  ((NrDevices'('1))),
    .DReqDepth ((NrDevices*4)'('0)),
    .DRspDepth ((NrDevices*4)'('0))
  ) u_s1n (
    .clk_i        (clk_sys),
    .rst_ni       (rst_sys_n),
    .axi_h_i      (axi_d_host2xbar),
    .axi_h_o      (axi_d_xbar2host),
    .axi_d_o      (axi_d_xbar2device),
    .axi_d_i      (axi_d_device2xbar),
    .dev_select_i (dev_sel_s1n)
  );

  // SRAM block for instruction and data storage
  axi_socket_m1 #(
    .M         (NrHosts + NrDMAMasters),
    .HReqPass  (2'h3),
    .HRspPass  (2'h3),
    .HReqDepth (8'h0),
    .HRspDepth (8'h0),
    .DReqPass  (1'b1),
    .DRspPass  (1'b1),
    .DReqDepth (4'h0),
    .DRspDepth (4'h0)
  ) u_sm1 (
    .clk_i   (clk_sys),
    .rst_ni  (rst_sys_n),
    .axi_h_i (axi_d_master2dma),
    .axi_h_o (axi_d_dma2master),
    .axi_d_o (axi_d_dma2ram),
    .axi_d_i (axi_d_ram2dma)
  );

  axi_adapter_device axi_d_xbar2ram_adapter (
      .clk_i   (clk_sys),
      .rst_ni  (rst_sys_n),

      .req_o   (device_req[Ram]),
      .gnt_i   (1'b1),
      .we_o    (device_we[Ram]),
      .be_o    (device_be[Ram]),
      .addr_o  (device_addr[Ram]),
      .wdata_o (device_wdata[Ram]),
      .valid_i (device_rvalid[Ram]),
      .rdata_i (device_rdata[Ram]),
      .err_i   (device_err[Ram]),

      .axi_o   (axi_d_ram2dma),
      .axi_i   (axi_d_dma2ram)
    );

  ram_3p #(
      .Depth(1024*1024/4),
      .MemInitFile(SRAMInitFile)
    ) u_ram (
      .clk_i       (clk_sys),
      .rst_ni      (rst_sys_n),

      .a_req_i     (device_req[Ram]),
      .a_we_i      (device_we[Ram]),
      .a_be_i      (device_be[Ram]),
      .a_addr_i    (device_addr[Ram]),
      .a_wdata_i   (device_wdata[Ram]),
      .a_rvalid_o  (device_rvalid[Ram]),
      .a_rdata_o   (device_rdata[Ram]),

      .b_req_i     (instr_req),
      .b_we_i      (1'b0),
      .b_be_i      (top_pkg::AXI_DSW'(0)),
      .b_addr_i    (instr_addr),
      .b_wdata_i   (top_pkg::AXI_DW'(0)),
      .b_rvalid_o  (instr_rvalid),
      .b_rdata_o   (instr_rdata)
    );

  // Simulator Ctrl
  axi_adapter_device axi_d_xbar2simulatorctrl_adapter (
      .clk_i   (clk_sys),
      .rst_ni  (rst_sys_n),

      .req_o   (device_req[SimCtrl]),
      .gnt_i   (1'b1),
      .we_o    (device_we[SimCtrl]),
      .be_o    (device_be[SimCtrl]),
      .addr_o  (device_addr[SimCtrl]),
      .wdata_o (device_wdata[SimCtrl]),
      .valid_i (device_rvalid[SimCtrl]),
      .rdata_i (device_rdata[SimCtrl]),
      .err_i   (device_err[SimCtrl]),

      .axi_o   (axi_d_device2xbar[SimCtrl]),
      .axi_i   (axi_d_xbar2device[SimCtrl])
    );
  sim_ctrl #(
    .LogName("ibex_simple_system.log")
    ) u_simulator_ctrl (
      .clk_i     (clk_sys),
      .rst_ni    (rst_sys_n),

      .req_i     (device_req[SimCtrl]),
      .we_i      (device_we[SimCtrl]),
      .be_i      (device_be[SimCtrl]),
      .addr_i    (device_addr[SimCtrl]),
      .wdata_i   (device_wdata[SimCtrl]),
      .rvalid_o  (device_rvalid[SimCtrl]),
      .rdata_o   (device_rdata[SimCtrl]),

      .end_sim   (end_sim)
    );

  // Timer
  axi_adapter_device axi_d_xbar2timer_adapter (
      .clk_i   (clk_sys),
      .rst_ni  (rst_sys_n),

      .req_o   (device_req[Timer]),
      .gnt_i   (1'b1),
      .we_o    (device_we[Timer]),
      .be_o    (device_be[Timer]),
      .addr_o  (device_addr[Timer]),
      .wdata_o (device_wdata[Timer]),
      .valid_i (device_rvalid[Timer]),
      .rdata_i (device_rdata[Timer]),
      .err_i   (device_err[Timer]),

      .axi_o   (axi_d_device2xbar[Timer]),
      .axi_i   (axi_d_xbar2device[Timer])
    );
  timer_custom #(
    .DataWidth    (top_pkg::AXI_DW),
    .AddressWidth (top_pkg::AXI_AW)
    ) u_timer (
      .clk_i          (clk_sys),
      .rst_ni         (rst_sys_n),

      .timer_req_i    (device_req[Timer]),
      .timer_we_i     (device_we[Timer]),
      .timer_be_i     (device_be[Timer]),
      .timer_addr_i   (device_addr[Timer]),
      .timer_wdata_i  (device_wdata[Timer]),
      .timer_rvalid_o (device_rvalid[Timer]),
      .timer_rdata_o  (device_rdata[Timer]),
      .timer_err_o    (device_err[Timer]),
      .timer_intr_o   (timer_irq)
    );

  // ToE Stub
  toe #(
    .DataWidth    (top_pkg::AXI_DW),
    .AddressWidth (top_pkg::AXI_AW)
    ) u_toe (
      .clk_i      (clk_sys),
      .rst_ni     (rst_sys_n),

      .axi_o      (axi_d_device2xbar[ToE]),
      .axi_i      (axi_d_xbar2device[ToE]),

      .axi_dma_o  (axi_d_master2dma[ToE_DMA]),
      .axi_dma_i  (axi_d_dma2master[ToE_DMA])

      //.io_read_tdata('0),
      //.io_read_tvalid('0),
      //.io_read_tready(),
      //.io_read_tuser('0),
      //.io_read_tlast('0),
    );

  // DMA Controller
  DMATop u_dma_top (
    .clock(clk_sys),
    .reset(~rst_sys_n),
    .io_control_aw_awaddr(axi_d_xbar2device[DMACtrl].awaddr),
    .io_control_aw_awprot('0),
    .io_control_aw_awvalid(axi_d_xbar2device[DMACtrl].awvalid),
    .io_control_aw_awready(axi_d_device2xbar[DMACtrl].awready),
    .io_control_w_wdata(axi_d_xbar2device[DMACtrl].wdata),
    .io_control_w_wstrb(axi_d_xbar2device[DMACtrl].wstrb),
    .io_control_w_wvalid(axi_d_xbar2device[DMACtrl].wvalid),
    .io_control_w_wready(axi_d_device2xbar[DMACtrl].wready),
    .io_control_b_bresp(axi_d_device2xbar[DMACtrl].bresp[1:0]),
    .io_control_b_bvalid(axi_d_device2xbar[DMACtrl].bvalid),
    .io_control_b_bready(axi_d_xbar2device[DMACtrl].bready),
    .io_control_ar_araddr(axi_d_xbar2device[DMACtrl].araddr),
    .io_control_ar_arprot('0),
    .io_control_ar_arvalid(axi_d_xbar2device[DMACtrl].arvalid),
    .io_control_ar_arready(axi_d_device2xbar[DMACtrl].arready),
    .io_control_r_rdata(axi_d_device2xbar[DMACtrl].rdata),
    .io_control_r_rresp(axi_d_device2xbar[DMACtrl].rresp[1:0]),
    .io_control_r_rvalid(axi_d_device2xbar[DMACtrl].rvalid),
    .io_control_r_rready(axi_d_xbar2device[DMACtrl].rready),
    .io_read_tdata('0),
    .io_read_tvalid('0),
    .io_read_tready(),
    .io_read_tuser('0),
    .io_read_tlast('0),
    .io_write_aw_awid(),
    .io_write_aw_awaddr(),
    .io_write_aw_awlen(),
    .io_write_aw_awsize(),
    .io_write_aw_awburst(),
    .io_write_aw_awlock(),
    .io_write_aw_awcache(),
    .io_write_aw_awprot(),
    .io_write_aw_awqos(),
    .io_write_aw_awvalid(),
    .io_write_aw_awready('0),
    .io_write_w_wdata(),
    .io_write_w_wstrb(),
    .io_write_w_wlast(),
    .io_write_w_wvalid(),
    .io_write_w_wready('0),
    .io_write_b_bid('0),
    .io_write_b_bresp('0),
    .io_write_b_bvalid('0),
    .io_write_b_bready(),
    .io_write_ar_arid(),
    .io_write_ar_araddr(),
    .io_write_ar_arlen(),
    .io_write_ar_arsize(),
    .io_write_ar_arburst(),
    .io_write_ar_arlock(),
    .io_write_ar_arcache(),
    .io_write_ar_arprot(),
    .io_write_ar_arqos(),
    .io_write_ar_arvalid(),
    .io_write_ar_arready('0),
    .io_write_r_rid('0),
    .io_write_r_rdata('0),
    .io_write_r_rresp('0),
    .io_write_r_rlast('0),
    .io_write_r_rvalid('0),
    .io_write_r_rready(),
    .io_irq_readerDone(dma_irq_reader_done),
    .io_irq_writerDone(dma_irq_writer_done),
    .io_sync_readerSync('0),
    .io_sync_writerSync('0)
  );

  // Tying the extra dma control axi response signals off
  assign axi_d_device2xbar[DMACtrl].bid = axi_d_xbar2device[DMACtrl].awid;
  assign axi_d_device2xbar[DMACtrl].rid = axi_d_xbar2device[DMACtrl].arid;
  assign axi_d_device2xbar[DMACtrl].rlast = axi_d_device2xbar[DMACtrl].rvalid;

  assign ext_irq = dma_irq_reader_done | dma_irq_writer_done;

  export "DPI-C" function mhpmcounter_get;

  function automatic longint mhpmcounter_get(int index);
    return u_core.u_ibex_core.cs_registers_i.mhpmcounter[index];
  endfunction

`ifdef UVM_TB
  `include "BenGen_bridge_tb_top.sv"
`else
  always @(posedge clk_i or negedge rst_ni) begin
    if (rst_ni) begin
      if (end_sim) $finish;
    end
  end
`endif

endmodule
