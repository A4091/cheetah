`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:01:28 08/02/2025 
// Design Name: 
// Module Name:    top 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////

// ###########################################################
`define USE_SPIROM          // undef for Parallel ROM define for SPI ROM
`undef USE_DIP_SWITCH       // undef to use Virtual Register, define to use Hardware Switch
`define A22_21_MISSING      // undef if A22 and A21 are connected to CPLD define if address missing
// ###########################################################

module A4092 (
    input [31:2] A,     // Address Bus
    inout [1:0] AL,     // lower Addresslines for NCR
    inout [31:0] D,     // Data Bus
    input CLK_50M,      // 50MHz Clock Input
    output reg CLK = 0, // 25MHz Clock Output

    // Zorro Bus Interface
    input IORST_n,      // I/O Reset
    inout [3:0] DS_n,   // Datastrobe
    input [2:0] FC,     // Function Code
    input Z_LOCK,       // Zorro LOCK signal or A1 while Quick interrupt cycle
    input C7M,          // 7MHz clock for arbitration
    inout Z_FCS_n,      // ZIII Signal, Is input and output (driven during DMA)
    inout DOE,          // Databuffer Output Enable
    input READ,         // Read /Write
    inout DTACK_n,      
    output INT2_n,
    input CFGIN_n,
    output CFGOUT_n,
    inout SLAVE_n,
    output CINH_n,
    inout MTCR_n,       // Is input for IACK, output for master
    inout MTACK_n,      // Multi Transfer Acknowledge, not used
    input BERR_n,
    input BGn,          // Zorro Bus Grant
    output BRn,         // Zorro Bus Request
    input SENSEZ3,      // ZIII Sense, 0 = Z2 Slot

    // Buffer Control
    output DBLT,
    output DBOE_n,
    output ABOEL_n,
    output ABOEH_n,
    output D2Z_n,
    output Z2D_n,
    output FCS,         // Output to U1 and U4 Addresslatch, high = latched!
    output BMASTER,     // Inverted MASTER_n signal

    // SCSI Chip Interface
    input SLACK_n,      // SCSI ack during slave access
    input SINT_n,       // SCSI interrupt
    input SBR_n,        // SCSI bus request (for DMA)
    inout [1:0] SIZ,    // Sizing bits from SCSI (for DMA)
    output SBG_n,       // SCSI bus grant (for DMA)
    input MASTER_n,     // SCSI chip is master of local bus
    inout SCSI_AS_n,    // Address Strobe to SCSI chip (PLD_AS)
    inout SCSI_DS_n,    // Data Strobe to SCSI chip (PLD_DS)
    output SCSI_SREG_n, // Register select to SCSI chip
    output reg SCSI_STERM_n  = 1,
    input CBREQ_n,      // Cache Burst Request, not used
    inout CBACK_n,      // Cache Burst Acknowledge
    input SC0,          // SCSI snoop control

    // ROM Interface
    output ROM_OE_n,
    output ROM_CE_n,
    output ROM_WE_n,

    // Alternative SPI Interface
    input SPI_MISO,
    output SPI_MOSI,
    output SPI_CLK,
    output SPI_CS_n,

    // SCSI ID Register
    output SID_n,       // Buffer Enable if DIP Switch is used
    output DIP_EXT_TERM,// Termination Enable if NO DIP Switch

    //test
    output test
    );

    wire slave_sig;
    wire slavecycle;
    wire mastercycle;
    wire dtack_sig;
    wire nobuscycle;

    // Buffercontrol
    wire [1:0] siz_sig;
    wire [1:0] addrl_sig;

    // Autoconfig
    wire card_cycle;                        // card cycle detected from autoconfig logic
    wire config_cycle;                      // config cycle detected
    wire cfgout;
    wire [31:28] configdata_out;	        // Autoconfig data to Zorro
    wire config_dtack;

    // SPI ROM
    wire rom_cycle;                         // ROM "Chipselect"
    wire rom_dtack;                         // dtack from ROM Module
`ifdef USE_SPIROM
    wire [7:0]spi_dataout;                  // spi read data
    wire spi_read;                          // spi data enable
    assign ROM_CE_n = 1;
    assign ROM_OE_n = 1;
    assign ROM_WE_n = 1;
`else
    assign SPI_CLK = 1'bZ;
    assign SPI_CS_n = 1'bZ;
    assign SPI_MOSI = 1'bZ;
`endif

    // SCSI Chip
    wire scsi_cycle;                        // SCSI "Chipselect"
    wire scsi_as_sig;                       // AS to NCR
    wire scsi_ds_sig;                       // DS to NCR
    wire scsi_dtack;                        // SCSI DTACK

    // Interrupt handling
    wire intreg_cycle;                      // intreg "Chipselect"
    wire int_dtack;                         // intreg DTACK
    wire int_sig;                           // Interrupt to Zorro
    wire quickint_cycle;                    // ZIII Quick Int Cycle running
    wire quickint_slave;                    // quickint slave signal
    wire [7:0] intdata_out;                 // Interrupt Register out
    wire intvector_read;                    // intvector data enable

    // SCSI ID (Switch Block)
    wire sid_cycle;                         // SID "Chipselect"
    wire [7:0] siddata_out;                 // SID output Data
    wire sid_read;                          // SID data Enable
    wire sid_dtack;                         // SID DTACK

    // DMA
    wire efcs;                              // FCS from DMA Engine, 1 = active
    wire mybus;                             // 1 when Card is Zorro Busmater
    wire dma_aboel;
    wire dma_aboeh;
    wire dma_doe;
    wire [3:0] ds_n_sig;

    // generate 25MHz Clock
    always @(posedge CLK_50M) begin
        CLK <= !CLK;
    end  

    assign test = spi_read;
 
    // ########################################
    // Zorro signal assignment
    assign CFGOUT_n = (!SENSEZ3 || cfgout) ? 1'bZ : 1;
    assign SLAVE_n = slave_sig ? 0 : 1'bZ;
    assign DTACK_n = dtack_sig ? 0 : 1'bZ;
    assign CINH_n = slave_sig ? 0 : 1'bZ;
    assign MTACK_n = slave_sig ? 1 : 1'bZ;
    assign INT2_n = (int_sig) ? 0 : 1'bZ;

    assign MTCR_n = mybus ? 1 : 1'bZ;
    assign Z_FCS_n = mybus ? !efcs : 1'bZ;
    assign DOE = mybus ? dma_doe : 1'bZ;

    // ########################################
    // Card internal Signal assignment
    assign CBACK_n = (MASTER_n) ? 1'bZ : 1;

    // SCSI Bus Termination Control
`ifdef USE_DIP_SWITCH
    assign DIP_EXT_TERM = 1'bZ;
    assign SID_n = !sid_read;
`else
    assign DIP_EXT_TERM = siddata_out[6];
    assign SID_n = 1;
`endif

    /* The SCSI termination is based on a synchronized DTACK.  I
    synchronize DTACK for either slave or master cycle, since the
    NCR 53C710 wants the effect of SLACK (which makes a DTACK on slave
    to SCSI cycles) reflected on STERM to actually end the cycle. */
    always @ (posedge Z_FCS_n, negedge CLK) begin
        if (Z_FCS_n) begin
            SCSI_STERM_n <= 1;
        end else begin
            if (!SCSI_AS_n && !DTACK_n) begin
                SCSI_STERM_n <= 0;
            end
        end
    end

    assign SCSI_AS_n = (mybus) ? 1'bz : !scsi_as_sig;
    assign SCSI_DS_n = (mybus) ? 1'bz : !scsi_ds_sig;

    // DS_n is output to cardinternal bus when MASTER_n is active (low)
    assign DS_n = MASTER_n ? 4'bZZZZ : ds_n_sig;
            
    // SIZ and AL are output to NCR when MASTER_n is inactive (high)
    assign SIZ = MASTER_n ? siz_sig : 2'bZZ;
    assign AL = MASTER_n ? addrl_sig : 2'bZZ;

    // Data go out to cardinternal bus so ignore DOE and Z_FCS_n since they considered at buffercontrol!
`ifdef USE_SPIROM
    assign D[31:28] = (config_cycle && READ) ? configdata_out : spi_read ? spi_dataout[7:4] : 4'bZZZZ;
    assign D[27:16] = {12{1'bZ}};
    assign D[15:12] = spi_read ? spi_dataout[3:0] : 4'bZZZZ;
    assign D[11:8] = 4'bZZZZ;
`else
    assign D[31:28] = (config_cycle && READ) ? configdata_out : 4'bZZZZ;
    assign D[27:8] = {20{1'bZ}};
`endif
`ifdef USE_DIP_SWITCH
    assign D[7:0] = intvector_read ? intdata_out : {8{1'bZ}};
`else
    assign D[7:0] = intvector_read ? intdata_out : sid_read ? siddata_out : {8{1'bZ}};
`endif
        
    assign rom_cycle    = card_cycle && !A[23];                     // $000000 - $7FFFFF
    assign scsi_cycle   = card_cycle &&  A[23] && !A[19];           // $800000 - $87FFFF
    assign intreg_cycle = card_cycle &&  A[23] &&  A[19] && !A[18]; // $880000 - $8BFFFF
    assign sid_cycle    = card_cycle &&  A[23] &&  A[19] &&  A[18]; // $8C0000 - $8FFFFF

    // qualifier for ZIII Quick Interrupt Cycle for INT2
    assign quickint_cycle = !Z_FCS_n && &FC && &A[19:16] && A[3:2] == 2'b01 && !Z_LOCK && !MTCR_n;

    // dtack logic
    assign dtack_sig = config_dtack || rom_dtack || scsi_dtack || int_dtack || sid_dtack;

    // slave logic
    assign slave_sig = config_cycle || card_cycle || quickint_slave;

    // Slavecycle when Card is not Zorro Master and no DMA from NCR pending
    assign slavecycle = !mybus && MASTER_n;
    // Mastercycle when Card is Zorro Master and DMA active
    assign mastercycle = mybus && !MASTER_n;

    // Module Instantiations
    buffercontrol BUFFER_CONTROL (
        .MASTER_n (MASTER_n),
       	.Z_FCS_n (Z_FCS_n),
        .slavecycle (slavecycle),
       	.mastercycle (mastercycle),
       	.slave (slave_sig),
       	.READ (READ),
        .DOE (DOE),
       	.DTACK_n (DTACK_n),
        .dma_aboel (dma_aboel),
        .dma_aboeh (dma_aboeh),
       	.DS_n (DS_n),
        .BMASTER (BMASTER),
       	.DBOE_n (DBOE_n),
       	.D2Z_n (D2Z_n),
        .Z2D_n (Z2D_n),
       	.DBLT (DBLT),
       	.FCS (FCS),
       	.ABOEL_n (ABOEL_n),
        .ABOEH_n (ABOEH_n),
       	.addrl (addrl_sig),
       	.siz (siz_sig)
    );

    autoconfig AUTO_CONFIG (
        .clk (CLK),
       	.Z_FCS_n (Z_FCS_n),
       	.FC (FC[1:0]),
       	.DOE (DOE),
       	.DS3_n (DS_n[3]),
        .READ (READ),
       	.DIN (D[31:24]),
       	.data_out (configdata_out),
       	.addrh (A[31:24]),
       	.addrl (A[8:2]),
       	.IORST_n (IORST_n),
        .BERR_n (BERR_n),
       	.SENSEZ3 (SENSEZ3),
       	.CFGIN_n (CFGIN_n),
       	.cfgout (cfgout),
        .config_cycle (config_cycle),
       	.dtack (config_dtack),
       	.card_cycle (card_cycle)
    );

	dmaarbiter DMA_ARBITER (
        .clk7m (C7M),
        .clk (CLK),
        .IORST_n (IORST_n),
        .MASTER_n (MASTER_n),
        .SBR_n (SBR_n),
        .SC0 (SC0),
        .EBG_n (BGn),
        .FCS_n (Z_FCS_n),
        .DTACK_n (DTACK_n),
        .mybus (mybus),
        .SBG_n (SBG_n),
        .EBR_n (BRn)
	);

	dmamaster DMA_MASTER (
        .clk (CLK_50M),
        .bclk (CLK),
        .IORST_n (IORST_n),
        .SLAVE_n (SLAVE_n),
        .mybus (mybus),
        .MASTER_n (MASTER_n),
        .SCSI_AS_n (SCSI_AS_n),
        .SCSI_DS_n (SCSI_DS_n),
        .READ (READ),
        .Z_FCS_n (Z_FCS_n),
        .DTACK_n (DTACK_n),
        .ADDRL (AL),
        .SIZ (SIZ),
        .efcs (efcs),
        .dma_aboel (dma_aboel),
        .dma_aboeh (dma_aboeh),
        .dma_doe (dma_doe),
        .ds_n (ds_n_sig)
	);

`ifdef USE_SPIROM
	spirom SPI_ROM (
        .clk (CLK_50M),
        .IORST_n (IORST_n),
        .romcycle (rom_cycle),
`ifdef A22_21_MISSING
        .addr ({A[23], A[23], A[20:2]}),
`else
        .addr (A[22:2]),
`endif
        .DOE (DOE),
        .DS_n (DS_n),
        .READ (READ),
        .FC2 (FC[2]),
        .dtack (rom_dtack),
        .spi_read (spi_read),
        .spi_dataout (spi_dataout),
        .spi_datain ({D[31:28],D[15:12]}),
        .SPI_CLK (SPI_CLK),
        .SPI_CS_n (SPI_CS_n),
        .SPI_MOSI (SPI_MOSI),
        .SPI_MISO (SPI_MISO)
	);
`else
	parallelrom PARALLEL_ROM (
        .CLK (CLK),
        .romcycle (rom_cycle),
        .DOE (DOE),
        .DS_n (DS_n),
        .READ (READ),
        .FC2 (FC[2]),
        .dtack (rom_dtack),
        .ROM_CE_n (ROM_CE_n),
        .ROM_OE_n (ROM_OE_n),
        .ROM_WE_n (ROM_WE_n)
	);
`endif
	scsiaccess SCSI_ACCESS (
        .bclk (CLK),
        .DOE (DOE),
        .DS_n (DS_n),
        .READ (READ),
        .scsi_cycle (scsi_cycle),
        .mybus (mybus),
        .SCSI_SREG_n (SCSI_SREG_n),
        .scsi_as_sig (scsi_as_sig),
        .scsi_ds_sig (scsi_ds_sig),		
        .SLACK_n (SLACK_n),
        .dtack (scsi_dtack)
	);

	interrupthandling INTERRUPT_HANDLING (
        .clk (CLK_50M),
        .intreg_cycle (intreg_cycle),
        .IORST_n (IORST_n),
        .DOE (DOE),
        .DS0_n (DS_n[0]),
        .READ (READ),
        .set_reset (!A[17]),
        .din (D[7:0]),
        .dout (intdata_out),
        .vector_read (intvector_read),
        .dtack (int_dtack),
        .SINT_n (SINT_n),
        .int_sig (int_sig),
        .FCS_n (Z_FCS_n),
        .SLAVE_n (SLAVE_n),
        .quickint_cycle (quickint_cycle),
        .slave (quickint_slave)
	);

    sidregister SID_REGISTER (
        .clk (CLK),
        .sid_cycle (sid_cycle),
        .IORST_n (IORST_n),
        .DOE (DOE),
        .DS0_n (DS_n[0]),
        .READ (READ),
        .DIN (D[7:0]),
        .DOUT (siddata_out),
        .sid_read (sid_read),
        .dtack (sid_dtack)
    );

endmodule
