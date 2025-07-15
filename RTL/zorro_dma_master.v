`timescale 1ns / 1ps

//
// MODULE: zorro_dma_master
// DESCRIPTION:
// Manages a full Zorro III DMA master cycle after the bus has been
// granted. This module implements the logic from the A4091's U305
// and U306 GALs.
//
module zorro_dma_master (
    input  wire CLK,
    input  wire RESET_n,

    // Control Signals
    input  wire BMASTER,      // Input from the arbiter, enables this module
    input  wire READ,
    input  wire [1:0] SIZ,    // Sizing signals from SCSI chip
    input  wire [1:0] A,      // Low address bits from SCSI chip
    input  wire SCSI_AS_n,    // Address Strobe from SCSI slave logic

    // Zorro Bus Interface
    input  wire ZORRO_FCS_n,  // FCS as seen from the bus
    input  wire ZORRO_DTACK_n,// DTACK as seen from the bus
    output wire DMA_DOE,      // Data Output Enable to be driven on bus
    output wire [3:0] DMA_DS_n, // Data Strobes to be driven on bus
    output wire DMA_FCS_n,    // FCS to be driven on bus

    // Outputs
    output wire SCSI_STERM_n, // Termination signal to SCSI chip
    output wire BFCS_out      // Buffered FCS for other internal modules
);

// Internal state registers from U305/U306
reg asq;
reg cycz3;
reg efcs;
reg bdtack;
reg sterm_n;

// The internal buffered FCS is a combination of the external FCS (during slave mode)
// and the generated external FCS (during master mode)
assign BFCS_out = (efcs && BMASTER) | (!ZORRO_FCS_n && !BMASTER);

// This logic runs on the main 25MHz clock
always @(posedge CLK or negedge RESET_n) begin
    if(!RESET_n) begin
        asq     <= 1'b0;
        cycz3   <= 1'b0;
        efcs    <= 1'b0;
        bdtack  <= 1'b0;
        sterm_n <= 1'b1;
    end else begin
        // ASQ logic from U305: Qualified SCSI Address Strobe
        if(!SCSI_AS_n) asq <= 1'b1;
        else if (BFCS_out) asq <= 1'b0;

        // CYCZ3 logic from U306: Asserts that we are driving a cycle on the Zorro bus
        if(BMASTER && !BFCS_out && asq && ZORRO_DTACK_n) cycz3 <= 1'b1;
        else if (!ZORRO_DTACK_n) cycz3 <= 1'b0;

        // EFCS logic from U306: The external FCS we drive during DMA
        if(BMASTER && cycz3) efcs <= 1'b1;
        else efcs <= 1'b0;

        // BDTACK logic from U306: A latched version of the Zorro DTACK
        if (!BFCS_out) begin
            if(!ZORRO_DTACK_n) bdtack <= 1'b1;
        end else begin
            bdtack <= 1'b0;
        end

        // STERM logic from U306: Termination signal for the SCSI chip
        if (!BFCS_out) begin
            if (bdtack) sterm_n <= 1'b0;
        end else begin
            sterm_n <= 1'b1;
        end
    end
end

// --- Combinatorial Output Assignments ---

// The Data Output Enable for the main data buffers
assign DMA_DOE = (BMASTER && !READ);

// The Zorro III Data Strobes generated from SCSI sizing info (from U305)
wire [3:0] dma_ds_n_logic = {
    ~( READ || (!A[1] && !A[0]) ),                                      // DS3 equivalent from u305.pld
    ~( READ || (!A[1] && !SIZ[0]) || (!A[1] && A[0]) || (!A[1] && SIZ[1]) ), // DS2 equivalent from u305.pld
    ~( READ || (!A[1] && !SIZ[1] && !SIZ[0]) || (!A[1] && SIZ[1] && SIZ[0]) || (!A[1] && A[0] && !SIZ[0]) || (A[1] && !A[0]) ), // DS1 equivalent from u305.pld
    ~( READ || (A[0] && SIZ[1] && SIZ[0]) || (!SIZ[1] && !SIZ[0]) || (A[1] && A[0]) || (A[1] && SIZ[1]) ) // DS0 equivalent from u305.pld
};

// The Data Strobes and FCS are only driven when this module is active
assign DMA_DS_n = (BMASTER && efcs) ? dma_ds_n_logic : 4'bxxxx;
assign DMA_FCS_n = !efcs;

// Final output for SCSI Termination
assign SCSI_STERM_n = sterm_n;

endmodule
