/**********************************************************************************
 *
 * Parallel flash ROM support
 *
 **********************************************************************************/
module parallelrom (
    input CLK,
    input romcycle,
    input DOE,
    input [3:0] DS_n,
    input READ,
    input FC2,
    output reg dtack = 0,
    output ROM_CE_n,
    output ROM_OE_n,
    output ROM_WE_n
);

    // ############################################################################
    localparam waitstates = 3;
    // ############################################################################

    reg [2:0] delay = waitstates;

    assign ROM_CE_n = !romcycle;
    assign ROM_OE_n = !(romcycle && READ);
    assign ROM_WE_n = !(romcycle && !READ && DOE && !DS_n[1] && !DS_n[3]); // write always a full byte
    // For supervisor-only write control, you would use this line instead:
    // assign ROM_WE_n = !(romcycle && !READ && DOE && !DS_n[1] && !DS_n[3] && FC2); // write always a full byte

   always @(negedge romcycle, posedge CLK) begin
        if (!romcycle) begin
            delay <= waitstates;
            dtack <= 0;
        end else begin
            dtack <= 0;
            if (READ || (DOE && !(DS_n[1] && DS_n[3]))) begin
                if (delay > 0) begin
                    delay <= delay - 1;
                end else begin
                    dtack <= 1;
                end
            end
        end
    end

endmodule
