`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date:    20:21:44 08/03/2025
// Design Name:
// Module Name:    spirom
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
module spirom(
    input clk,
    input IORST_n,
    input romcycle,
    input [22:2] addr,
    input DOE,
    input [3:0] DS_n,
    input READ,
    input FC2,
    output reg dtack = 0,
    output reg spi_read = 0,
    output reg [7:0] spi_dataout = 0,
    input [7:0] spi_datain,
    output reg SPI_CLK = 0,
    output reg SPI_CS_n = 1,
    output reg SPI_MOSI = 0,
    input SPI_MISO
    );

    localparam  SPI_IDLE    =  2'b00,
                SPI_N       =  2'b01,
                SPI_P       =  2'b11,
                SPI_DTACK   =  2'b10;

    (* fsm_encoding = "user" *)reg [1:0] spi_state = SPI_IDLE;
    reg [5:0] cnt = 40;
    wire [39:0] readcmd;
    reg close = 1;

    assign readcmd = {8'h03, 3'h000, addr[22:2], spi_datain};

    always @ (negedge IORST_n, negedge clk) begin
        if (!IORST_n) begin
            cnt <= 40;
            spi_read <= 0;
            dtack <= 0;
            SPI_CLK <= 0;
            SPI_CS_n <= 0;
            SPI_MOSI <= 0;
            close = 1;
            spi_state <= SPI_IDLE;
        end else begin
            spi_read <= 0;
            dtack <= 0;
            SPI_CLK <= 0;
            SPI_MOSI <= 0;
            if (romcycle) begin
                close = 1;
                case (spi_state)
                SPI_IDLE : begin
                    cnt <= 40;
                    if (&addr[22:3] && ~&DS_n) begin
                        close = addr[2];
                        cnt <= 8;
                        SPI_CS_n <= 0;
                        spi_state <= SPI_N;
                    end else if (READ) begin
                        SPI_CS_n <= 0;
                        spi_state <= SPI_N;
                    end else if (romcycle) begin
                        spi_state <= SPI_DTACK;
                    end
                end
                SPI_N : begin
                    if (cnt == 0) begin
                        spi_state <= SPI_DTACK;
                    end else begin
                        if (cnt <= 8 && READ) begin
                            SPI_MOSI <= 0;
                        end else begin
                            SPI_MOSI <= readcmd[cnt-1];
                        end
                        spi_state <= SPI_P;
                    end
                end
                SPI_P : begin
                    SPI_CLK <= 1;
                    spi_dataout <= {spi_dataout[6:0], SPI_MISO};
                    cnt <= cnt -1;
                    spi_state <= SPI_N;
                end
                SPI_DTACK : begin
                    SPI_CS_n <= close;
                    spi_read <= READ;
                    dtack <= 1;
                    spi_state <= SPI_DTACK;
                end
                endcase
            end else begin
                SPI_CS_n <= close;
                spi_state <= SPI_IDLE;
            end
        end
     end

endmodule
