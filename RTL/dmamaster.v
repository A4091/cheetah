`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date:    18:20:37 07/21/2025
// Design Name:
// Module Name:    dmamaster
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
module dmamaster(
    input clk,
    input bclk,
    input IORST_n,
    input SLAVE_n,
    input mybus,
    input MASTER_n,
    input SCSI_AS_n,
    input SCSI_DS_n,
    input READ,
    input Z_FCS_n,
    input DTACK_n,
    input [1:0] ADDRL,
    input [1:0] SIZ,
    output reg efcs = 0,
    output dma_aboel,
    output reg dma_aboeh = 0,
    output reg dma_doe = 0,
    output reg [3:0] ds_n
);

    localparam STATE_IDLE   = 3'b000,       // IDLE, Bus is free
               STATE_ABOEH  = 3'b001,       // ABOEL, ABOEH active
               STATE_FCS    = 3'b011,       // ABOEL, ABOEH, FCS active
               STATE_GAP    = 3'b010,       // ABOEL, FCS active
               STATE_DOE    = 3'b110,       // ABOEL, FCS, DOE active
               STATE_DS     = 3'b100;       // ABOEL, FCS, DOE, DS active, wait for DTACK active

    (* fsm_encoding = "User" *)reg [2:0] dmamaster = STATE_IDLE, dmamaster_next = STATE_IDLE;

    reg dma_ds = 0;
    reg scsi_as_sig = 0;
    reg scsi_ds_sig = 0;
    wire busfree;

    assign busfree = Z_FCS_n && DTACK_n && SLAVE_n;

    // ds_n based on A1,A0 and SIZ from NCR
    always @ (*) begin
        if (dma_ds) begin
            ds_n[0] <= !(READ || (ADDRL[0] && SIZ == 2'b11) || SIZ == 2'b00 || ADDRL == 2'b11 || (ADDRL[1] && SIZ[1]));
            ds_n[1] <= !(READ || (!ADDRL[1] && SIZ == 2'b00) || (!ADDRL[1] && SIZ == 2'b11) || (ADDRL == 2'b01 && !SIZ[0]) || ADDRL == 2'b10);
            ds_n[2] <= !(READ || (!ADDRL[1] && !SIZ[0]) || ADDRL == 2'b01 || (!ADDRL[1] && SIZ[1]));
            ds_n[3] <= !(READ || ADDRL == 2'b00);
        end else begin
            ds_n <= 4'b1111;
        end
    end

    // sample AS and DS aut BCLK rising edge
    always @ (negedge IORST_n, posedge bclk) begin
        if (!IORST_n) begin
            scsi_as_sig <= 0;
            scsi_ds_sig <= 0;
        end else begin
            scsi_as_sig <= !SCSI_AS_n;
            scsi_ds_sig <= !SCSI_DS_n;
        end
    end

    // state register block
    always @ (negedge IORST_n, posedge clk) begin
        if (!IORST_n) begin
            dmamaster <= STATE_IDLE;
        end else begin
            dmamaster <= dmamaster_next;
        end
    end

    // input signal processing
    always @ (*) begin
        if (!mybus || !IORST_n) begin
            dmamaster_next = STATE_IDLE;
        end else begin
            dmamaster_next = dmamaster;
            case (dmamaster)
                STATE_IDLE: begin
                    // Start dma cycle if bus is free and NCR AS is active
                    if (busfree && scsi_as_sig) begin
                        dmamaster_next = STATE_ABOEH;
                    end
                end
                STATE_ABOEH: begin
                    // addressbuffer active
                    dmamaster_next = STATE_FCS;
                end
                STATE_FCS: begin
                    // data and addressbuffer inactive
                    dmamaster_next = STATE_GAP;
                end
                STATE_GAP: begin
                    // databuffers active
                    dmamaster_next = STATE_DOE;
                end
                STATE_DOE: begin
                    // Datastrobe active
                    dmamaster_next = STATE_DS;
                end
                STATE_DS: begin
                    // wait for NCR cycle end
                    if (SCSI_AS_n) begin
                        dmamaster_next = STATE_IDLE;
                    end
                end
                default: dmamaster_next = STATE_IDLE;
            endcase
        end
    end

    // Async signals to Zorro Bus
    // always drive dma_aboel when Card is ZIII Master
    assign dma_aboel = mybus;

    // efcs
    always @ (*) begin
        case (dmamaster_next)
            STATE_FCS,
            STATE_GAP,
            STATE_DOE,
            STATE_DS: efcs <= 1;
        default: efcs <= 0;
        endcase
    end

    // dma_aboeh
    always @ (*) begin
        case (dmamaster_next)
            STATE_ABOEH,
            STATE_FCS: dma_aboeh <= 1;
        default: dma_aboeh <= 0;
        endcase
    end

    // dma_doe
    always @ (*) begin
        case (dmamaster_next)
            STATE_DOE,
            STATE_DS: dma_doe <= 1;
        default: dma_doe <= 0;
        endcase
    end

    // dma_ds
    always @ (*) begin
        case (dmamaster_next)
            STATE_DS: dma_ds <= 1;
        default: dma_ds <= 0;
        endcase
    end

endmodule
