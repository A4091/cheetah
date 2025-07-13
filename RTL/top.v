`timescale 1ns / 1ps

//
// MODULE: A4092 (top.v)
// DESCRIPTION:
// Final version incorporating all discussed fixes: interrupt logic, DMA architecture,
// SCSI slave interface, and correct FCS signal generation and buffering.
//
module A4092(
    // Address Bus
    inout  wire [31:0] A,
    // Data Bus
    inout  wire [31:0] D,
    // External 50MHz Clock
    input  wire CLK_50M,
    // Internal 25MHz Clock
    output wire CLK,

    // Zorro Bus Interface
    input  wire IORST_n,
    inout  wire [3:0] DS_n,
    input  wire [2:0] FC,
    input  wire Z_LOCK,  // Zorro LOCK signal
    input  wire C7M,     // 7MHz clock for arbitration
    inout  wire Z_FCS_n, // ZIII Signal, Is input and output (driven during DMA)
    output wire FCS,     // Output to U1 and U4 Addresslatch, high = latched!
    inout  wire DOE,
    input  wire READ,
    inout  wire DTACK_n,
    output wire INT2_n,
    input  wire CFGIN_n,
    output wire CFGOUT_n,
    output wire SLAVE_n,
    output wire CINH_n,
    inout  wire MTCR_n, // Is input for IACK, output for master
    input  wire BERR_n,
    input  wire BGn, // Zorro Bus Grant
    output wire BRn, // Zorro Bus Request
    input  wire SENSEZ3_n,

    // Buffer Control
    output wire DBLT,
    output wire DBOE_n,
    output wire ABOEL_n,
    output wire ABOEH_n,
    output wire D2Z_n,
    output wire Z2D_n,

    // SCSI Chip Interface
    input  wire SLACK_n,   // SCSI ack during slave access
    input  wire SINT_n,    // SCSI interrupt
    input  wire SBR_n,     // SCSI bus request (for DMA)
    input  wire MASTER_n,  // SCSI chip is master of local bus
    output wire [1:0] SIZ, // Sizing bits from SCSI (for DMA)
    output wire SBG_n,     // SCSI bus grant (for DMA)
    output wire BMASTER,   // Inverted MASTER_n signal
    output wire SCSI_AS_n, // Address Strobe to SCSI chip (PLD_AS)
    output wire SCSI_DS_n, // Data Strobe to SCSI chip (PLD_DS)
    output wire SCSI_SREG_n, // Register select to SCSI chip
    output wire SCSI_STERM_n,

    // ROM Interface
    output wire ROM_OE_n,
    output wire ROM_CE_n,
    output wire ROM_WE_n,

    // Alternative SPI Interface
    input  wire SPI_MISO,
    output wire SPI_MOSI,
    output wire SPI_CLK,
    output wire SPI_CS_n,

    // Board Control
    output wire SID_n,
    output wire DIP_EXT_TERM,

    // Unused:
    // We _never_ issue a CBACK, since BURST isn't supported
    input  wire CBREQ_n,
    output wire CBACK_n,
    output wire MTACK_n
);

`include "globalparams.vh"

// --- Wires and Registers ---
wire CLKI;
reg  autoconfig_addr_match;
reg  scsi_addr_match;
wire match = autoconfig_addr_match || scsi_addr_match;
wire configured;
wire validspace = FC[1] ^ FC[0];
wire shutup;
reg  [1:0] z3_state;
reg  dtack;
reg  scsi_cycle;
reg  autoconfig_cycle;
wire mybus_n; // bus ownership signal

// DS_n activity synchronizer (any strobe active)
reg [1:0] ds_active_sync;
wire ds_active = DS_n != 4'b1111;  // any strobe active

// DOE synchronizer (when used as input)
reg [1:0] doe_sync;
wire doe_synced = doe_sync[1];

// Connections to Sub-modules
wire bfcs; // The internal, buffered FCS signal
wire [3:0] autoconfig_dout;
wire autoconfig_cfgout;
wire [7:0] scsi_base_addr;

wire autoconfig_dtack;
wire scsi_dtack;
wire rom_dtack;
wire spi_dtack;
wire sid_dtack;

wire iack_slave_n;
wire iack_dtack_n;
wire [7:0] iack_dout;

// Buffer Control
wire DBOE_n_int;
wire ABOEL_n_int;
wire ABOEH_n_int;
wire D2Z_n_int;
wire Z2D_n_int;
wire DBLT_int;

`ifndef USE_DIP_SWITCH
wire [7:0] dip_shadow;
`endif
wire [7:0] spi_shadow;

wire slave_cycle = mybus_n &&  MASTER_n;

wire dma_fcs_n, dma_doe;
wire [3:0] dma_ds_n;

assign FCS = ~Z_FCS_n;// Only used for Addresslatch U1 and U4 in Slave Mode simply use Z_FCS_n to minimize delay
assign DS_n  = BMASTER ? dma_ds_n  : 4'bzzzz;
//assign DOE   = (BMASTER && !READ) || (slave_cycle && !READ && !bfcs);
assign DOE   = !mybus_n ? dma_doe : 1'bz;	// output from DMA engine when DMA else input...

/* Unused */
assign MTACK_n = 1'bz;
assign CBACK_n = 1'bz;

// --- Clock Generation ---
reg clk_int;
always @(posedge CLK_50M)
  clk_int <= ~clk_int;

assign CLKI = ~clk_int;
assign CLK = clk_int;

// --- Memory Map / Device Mapper (U203) ---

/* The basic configuration unit takes 16MB of space.  I slice it up based on A23,
 * A19..A17.  That's because, while the slicing is rather arbitrary, the interrupt
 * cycle decode needs A19..A17, so I use these for partitioning too.
 *
 *  8C0000       IDREG
 *  880000       INTREG
 *  840000       SCSI write
 *  800000       SCSI read
 *  000000       ROM
 */

wire rom_region       = scsi_addr_match && slave_cycle && (A[23:0] >= 24'h000000 && A[23:0] < 24'h800000);
wire scsi_region      = scsi_addr_match && slave_cycle && (A[23:0] >= 24'h800000 && A[23:0] < 24'h880000);
wire interrupt_region = scsi_addr_match && slave_cycle && (A[23:0] >= 24'h880000 && A[23:0] < 24'h8c0000);
wire idreg_region     = scsi_addr_match && slave_cycle && (A[23:0] >= 24'h8c0000 && A[23:0] < 24'h8f0000);

// --- Address Latching and Matching ---
always @(negedge Z_FCS_n or negedge IORST_n) begin
  if (!IORST_n) begin
    scsi_addr_match <= 0;
    autoconfig_addr_match <= 0;
  end else begin
    //master_n_int <= READ;
    if (A[31:24] == scsi_base_addr && configured) begin
      scsi_addr_match <= 1;
    end else begin
      scsi_addr_match <= 0;
    end
    if ({A[31:24]} == 8'hFF && !configured && !shutup && !CFGIN_n && SENSEZ3_n) begin	// SENSEZ3_n = 0 disable autoconfig
      autoconfig_addr_match <= 1;
    end else begin
      autoconfig_addr_match <= 0;
    end
  end
end

// --- DS_n Activity Synchronizer ---
always @(posedge CLK or negedge IORST_n) begin
  if (!IORST_n) begin
    ds_active_sync <= 2'b00;
    doe_sync <= 2'b00;
  end else begin
    ds_active_sync <= {ds_active_sync[0], ds_active};
    doe_sync <= {doe_sync[0], DOE};
  end
end

// --- Main Zorro Slave State Machine ---
always @(posedge CLK or negedge IORST_n) begin
  if (!IORST_n) begin
    z3_state         <= Z3_IDLE;
    dtack            <= 1'b0;
    scsi_cycle       <= 1'b0;
    autoconfig_cycle <= 1'b0;
  end else begin
    // Use the new internal 'bfcs' for cycle detection
    case (z3_state)
      Z3_IDLE: begin
          dtack <= 0;
          if (!bfcs && match && validspace) begin
            z3_state         <= Z3_START;
            autoconfig_cycle <= autoconfig_addr_match;
            scsi_cycle       <= scsi_addr_match;
          end
        end
      Z3_START: begin
          if (bfcs) begin
            z3_state <= Z3_IDLE;
          end else if (doe_synced && ds_active_sync[1]) begin
            z3_state <= Z3_DATA;
          end
        end
      Z3_DATA: begin
          if (bfcs) begin
            z3_state <= Z3_IDLE;
          end else if ((autoconfig_dtack && autoconfig_cycle) ||
                       (scsi_dtack && scsi_cycle) ||
                       (rom_dtack) || (spi_dtack) || (sid_dtack) || !iack_dtack_n) begin
            z3_state <= Z3_END;
          end
        end
      Z3_END: begin
          if (bfcs) begin
            z3_state         <= Z3_IDLE;
            scsi_cycle       <= 0;
            autoconfig_cycle <= 0;
            dtack            <= 0;
          end else begin
            z3_state         <= Z3_END;
            dtack            <= 1;
          end
        end
    endcase
  end
end

// --- Top-Level Bus Assignments ---
assign DTACK_n  = dtack ? 1'b0 : 1'bz;
//assign SLAVE_n  = !((slave_cycle && configured) || !iack_slave_n);
assign SLAVE_n = (!(!bfcs && match && validspace) && iack_slave_n);

assign CFGOUT_n = (SENSEZ3_n) ? autoconfig_cfgout : CFGIN_n;
assign CINH_n   = !(slave_cycle && configured);

// Buffer Control
assign DBOE_n   = DBOE_n_int;
assign ABOEL_n  = ABOEL_n_int;
assign ABOEH_n  = ABOEH_n_int;
assign D2Z_n    = D2Z_n_int;
assign Z2D_n    = Z2D_n_int;
assign DBLT     = DBLT_int;

// --- Data Bus Multiplexer for Read Cycles ---
// The CPLD drives the data bus (D) only during a READ cycle when one of
// its internal registers/logic is selected and ready. Otherwise, D is tristated.

// Autoconfig data is driven on D[31:28].
// This happens when it's an autoconfig cycle, dtack is active (meaning data phase), and it's a READ.

// Data goes out to Card internal nonmultiplexed databus, Mux and OE to/from Zorro is done in buffer_control
//assign D[31:28] = (autoconfig_cycle && dtack && READ) ? autoconfig_dout :
assign D[31:28] = (autoconfig_cycle && READ) ? autoconfig_dout :
                  (spi_dtack) ? spi_shadow[7:4] :
                  4'bZZZZ;
assign D[15:12] = spi_dtack ? spi_shadow[3:0] : 4'bZZZZ;

// Interrupt Vector (iack_dout) or SCSI ID (dip_shadow) are driven on D[7:0].
// The conditions `!iack_dtack_n` and `sid_dtack` already imply a READ cycle
// from their respective modules.
assign D[7:0]   = !iack_dtack_n ? iack_dout :        // Interrupt vector has priority
`ifndef USE_DIP_SWITCH
                  sid_dtack     ? dip_shadow :
`endif
                                  8'bzzzzzzzz;       // Tristate if neither

// Bits D[27:16] and D[11:8] are not driven by this CPLD.
// The dummy TIE_OFF_CONDITION silences "never assigned" warnings.
localparam TIE_OFF_CONDITION = 1'b0;
assign D[27:16] = TIE_OFF_CONDITION ? 12'd0 : 12'bZZZZZZZZZZZZ;
assign D[11:8]  = TIE_OFF_CONDITION ? 4'd0  : 4'bZZZZ;

// --- SCSI Slave Interface  ---



// --- Module Instantiations ---

Autoconfig AUTOCONFIG (
  .scsi_base_addr(scsi_base_addr),
  .ADDRL({A[8:2]}),
  .FCS_n(!bfcs),
  .CLK(CLK),
  .READ(READ),
  .DIN(D[31:24]),
  .RESET_n(IORST_n),
  .CFGOUT_n(autoconfig_cfgout),
  .autoconfig_cycle(autoconfig_cycle),
  .dtack(autoconfig_dtack),
  .configured(configured),
  .DOUT(autoconfig_dout),
  .shutup(shutup)
);

scsi_access SCSI_ACCESS (
  .CLK(CLK),
  .RESET_n(IORST_n),
  .scsi_region(scsi_region),
  .FCS_n(!bfcs),
  .slave_cycle(slave_cycle),
  .configured(configured),
  .SLACK_n(SLACK_n),
  .scsi_dtack(scsi_dtack)
);

scsi_slave SCSI_SLAVE (
  // --- Inputs
  .CLK(CLK),
  .CLKI(CLKI),
  .IORST_n(IORST_n),
  .SCSI_n(SCSI_n),
  .READ(READ),
  .DS_n(DS_n),
  .DOE(DOE),
  .DTACK_n(dtack),
  .SCSI_STERM_n(SCSI_STERM_n),
  .MYBUS_n(mybus_n),
  .A2(A[2]),
  .scsi_cycle(scsi_cycle),
  .slave_cycle(slave_cycle),

  // --- Outputs
  .SCSI_SREG_n(SCSI_SREG_n),
  .SCSI_DS_n(SCSI_DS_n),
  .SCSI_AS_n(SCSI_AS_n),
  .SIZ(SIZ),
  .ADDRL(A[1:0])
);

rom_access ROM_ACCESS (
  .CLK(CLK),
  .RESET_n(IORST_n),
  .rom_region(rom_region),
  .READ(READ),
  .FCS_n(!bfcs),
  .shutup(shutup),
  .rom_dtack(rom_dtack),
  .ROM_CE_n(ROM_CE_n),
  .ROM_OE_n(ROM_OE_n),
  .ROM_WE_n(ROM_WE_n)
);

spi_access SPI_ACCESS (
    .ADDR(A[20:0]),
    .SPI_MISO(SPI_MISO),
    .SPI_CLK(SPI_CLK),
    .SPI_MOSI(SPI_MOSI),
    .SPI_CS_n(SPI_CS_n),
    .spi_dtack(spi_dtack),
    .DOUT(spi_shadow)
);

sid_access SID_ACCESS (
  // --- Inputs
  .CLK(CLK),
  .RESET_n(IORST_n),
  .idreg_region(idreg_region),
  .READ(READ),
  .FCS_n(!bfcs),
  .slave_cycle(slave_cycle),
  .configured(configured),

`ifndef USE_DIP_SWITCH
  .DIN(D[7:0]),
  .DOUT(dip_shadow),
  .dip_ext_term(DIP_EXT_TERM),
`endif
  .sid_dtack(sid_dtack),
  .SID_n(SID_n)
);

intreg_access INTREG_ACCESS (
  // --- Core Inputs
  .CLK(CLK),
  .RESET_n(IORST_n),
  .FCS_n(!bfcs),
  .configured(configured),
  .interrupt_region(interrupt_region),
  // --- Zorro III Bus Inputs
  .FC(FC),
  .ADDR(A[23:17]),
  .LOCK(Z_LOCK),
  .READ(READ),
  .DS0_n(DS_n[0]),
  .MTCR_n(MTCR_n),

  // --- Interrupt Input from SCSI chip
  .NCR_INT(SINT_n),

  // --- Outputs
  .INT2_n(INT2_n),
  .iack_slave_n(iack_slave_n),
  .iack_dtack_n(iack_dtack_n),
  .DOUT(iack_dout)
);

buffer_control BUFFER_CONTROL (
  .CLK(CLK),
  .RESET_n(IORST_n),

  // --- Control Signals ---
  .READ(READ),
  .FCS_n(bfcs),
  .DOE(DOE),
  .DTACK_n(DTACK_n),

  // --- Master/Slave Cycle Controls ---
  .MYBUS_n(mybus_n),
  .MASTER_n(MASTER_n),
  .SLAVE_n(SLAVE_n),

  // --- Outputs to Transceivers ---
  .DBOE_n(DBOE_n_int),
  .ABOEL_n(ABOEL_n_int),
  .ABOEH_n(ABOEH_n_int),
  .D2Z_n(D2Z_n_int),
  .Z2D_n(Z2D_n_int),
  .DBLT(DBLT_int)
);

zorro_master_arbiter ZMA (
  // --- Inputs ---
  .C7M(C7M),
  .RESET_n(IORST_n),
  .MASTER_n(MASTER_n),
  .SBR_n(SBR_n),
  .EBG_n(BGn),
  .FCS(!FCS_n), // Pass active-high FCS
  //.FCS(!bfcs),
  .DTACK_n(~dtack), // FIXME
  // --- Outputs ---
  .MYBUS_n(mybus_n),
  .SBG_n(SBG_n),
  .EBR_n(BRn), // Drives bus request
  .BMASTER(BMASTER)
);

zorro_dma_master ZDMA (
  .CLK(CLK),
  .RESET_n(IORST_n),
  .BMASTER(BMASTER),
  .READ(READ),
  .SIZ(SIZ),
  .A(A[1:0]),
  .SCSI_AS_n(SCSI_AS_n),
  .ZORRO_FCS_n(Z_FCS_n),
  .ZORRO_DTACK_n(DTACK_n),
  .DMA_DOE(dma_doe),
  .DMA_DS_n(dma_ds_n),
  .DMA_FCS_n(dma_fcs_n),
  .SCSI_STERM_n(SCSI_STERM_n),
  .BFCS_out(bfcs)
);

endmodule
