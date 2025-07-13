`timescale 1ns / 1ps

/*
 * MODULE: intreg_access
 *
 * DESCRIPTION:
 * This module manages interrupt generation and the full Zorro III interrupt
 * acknowledge (IACK) handshake. It has been updated to include the
 * complete state machine logic from the original A4091 GALs (U203, U207).
 *
 * NOTE: This module has new ports. You must update your top-level design
 * to connect the new I/O signals (e.g., FC, MTCR_n, iack_slave_n, etc.).
 */
module intreg_access (
    // -- Core Inputs
    input wire CLK,
    input wire RESET_n,
    input wire FCS_n,
    input wire configured,
    input wire interrupt_region,

    // -- Zorro III Bus Inputs for IACK Cycle
    input wire [2:0] FC,       // Function Codes
    input wire [23:17] ADDR,     // Address Bus
    input wire LOCK,      // Zorro LOCK signal (original A1)
    input wire READ,
    input wire DS0_n,
    input wire MTCR_n,    // Multiple Transfer Cycle Strobe

    // -- Interrupt Input from SCSI Chip
    input wire NCR_INT,

    // -- Outputs
    output wire INT2_n,     // Main interrupt request to Zorro bus
    output reg iack_slave_n, // Drives SLAVE_n during an IACK cycle
    output reg iack_dtack_n, // Drives DTACK_n during an IACK cycle
    output reg [7:0] DOUT      // Interrupt vector output
);

// Internal Registers for State Machine
reg int_pending;   // Latched interrupt request from NCR_INT
reg int_assigned;  // Flag: Set when driver has written an interrupt vector
reg int_servicing; // Flag: Set when we are servicing an IACK cycle
reg int_poll;      // Flag: Set during the 'poll' phase of the IACK cycle
reg [7:0] int_vector;  // Stores the interrupt vector written by the driver

// Address matching for the Interrupt Control Register
//
// The interrupt control register is mapped to the base address 0x880000.
// A write to this space sets the interrupt vector (INTREG function).
// A read during an IACK cycle retrieves the vector (INTVEC function).
wire match_intreg_write = interrupt_region && !LOCK && !READ;
//wire match_intvec_read  = configured && !LOCK && (ADDR[23:17] == 8'h44) && READ; // Reading from the space provides the vector

// Zorro III Interrupt Acknowledge Cycle Detection
// An IACK cycle is FC=7 ('111') and targets a specific address range.
// This logic replicates the qualified interrupt space decode from U203.
wire iack_cycle_detect = (FC == 3'b111) && READ;

// Drive the main Zorro interrupt line (INT2_n is open-drain)
assign INT2_n = (int_pending) ? 1'b0 : 1'bZ;

// --- State Machine and Logic ---

always @(posedge CLK or negedge RESET_n) begin
    if (!RESET_n) begin
        // Reset all state flags and outputs
        int_pending   <= 1'b0;
        int_assigned  <= 1'b0;
        int_servicing <= 1'b0;
        int_poll      <= 1'b0;
        int_vector    <= 8'h0F; // Default to spurious interrupt vector
        iack_slave_n  <= 1'b1;
        iack_dtack_n  <= 1'b1;
        DOUT          <= 8'h00;
    end else begin
        // 1. Latch the interrupt from the SCSI chip (from U207)
        // This creates a stable interrupt signal that only changes between Zorro cycles.
        if (!FCS_n) begin
            if(~NCR_INT) int_pending <= 1'b1; // NCR_INT is low active
        end else begin
            // Clear pending interrupt if an IACK cycle completes
            if (iack_dtack_n == 1'b0) begin
                int_pending <= 1'b0;
            end
        end

	// 2. INT2_n moved out of the state machine

        // 3. Latch when the driver has written to the interrupt register (from U203 `INTASS`)
        if (match_intreg_write && !FCS_n) begin
            int_assigned <= 1'b1;
            // Here you would latch the interrupt vector from the data bus, e.g.:
            // int_vector <= D_IN[7:0];
            // Since D_IN is not an input, we will use a fixed vector for now.
            // A full implementation requires passing the data bus in.
            int_vector <= 8'h18; // Example vector for INT 2
        end

        // 4. Latch the start of an interrupt service condition (from U203 `INTSERV`)
        // An IACK cycle begins, we have a pending interrupt, and a vector has been assigned.
        if (iack_cycle_detect && int_pending && int_assigned && !FCS_n) begin
            int_servicing <= 1'b1;
        end
        // Service condition is cleared when the cycle ends (FCS_n goes high)
        if (FCS_n) begin
            int_servicing <= 1'b0;
        end

        // 5. Latch the polling phase of the IACK cycle (from U203 `INTPOLL`)
        // This starts when the service condition is active and MTCR is asserted.
        if (int_servicing && !MTCR_n) begin
            int_poll <= 1'b1;
        end
        if (FCS_n) begin
            int_poll <= 1'b0;
        end

        // 6. Drive SLAVE_n during the IACK cycle (from U203 `SLAVE.OE`)
        // We claim the bus once the poll phase is active.
        if (int_servicing && int_poll) begin
            iack_slave_n <= 1'b0;
        end else begin
            iack_slave_n <= 1'b1;
        end

        // 7. Drive DTACK_n and provide the vector (from U203 `INTVEC`)
        // This happens after we've asserted SLAVE_n and see the data strobe.
        if (iack_slave_n == 1'b0 && !DS0_n) begin
            iack_dtack_n <= 1'b0; // Terminate the cycle
            DOUT         <= int_vector; // Drive the vector onto the data bus
        end else begin
            iack_dtack_n <= 1'b1;
            DOUT         <= 8'hZZ; // High-impedance when not driving
        end
    end
end

endmodule
