/**********************************************************************************
 *
 * DMA arbiter
 *
 **********************************************************************************/
module dmaarbiter (
    input clk7m,
    input clk,
    input IORST_n,
    input MASTER_n,
    input SBR_n,
    input SC0,
    input EBG_n,
    input FCS_n,
    input DTACK_n,
    output reg mybus = 0,
    output reg SBG_n = 1,
    output reg EBR_n = 1
);

    reg RCHNG = 0;
    reg DMASTER_n = 1;					// 1 cycle delayed smaster_n
    reg SMASTER_n = 1;					// MASTER_n synced to 7m clock
    reg REGED = 0;						// registration indicator
    reg SSBR_n = 1;						// SBR_n synced to 7m clock
    reg BLOCKBG = 0;

    // until now 1:1 from A4091 U303 should converted to FSM for readability

    // The SCSI chip can be given the A4091 bus as soon as there's no activity on it.
    // Hold onto it until the SCSI becomes master. */
    // Not really if granted early the chip will have as asserted then fcs will
    // assert and when the z bus is granted fcs and addr will assert a the same time
	 // Dorken: set when FCS, DTACK_n, IORST_n BLOCKBG inactive and SBR_n EBG_n active
	 // Dorken: hold until SBR_n and MASTER_N inactive or IORST_n active or BLOCKBG active
    always @(*) begin
        if (!IORST_n) begin
            SBG_n = 1;
        end else if (FCS_n && DTACK_n && IORST_n && !BLOCKBG && !SBR_n && !EBG_n) begin
            SBG_n = 0;
        end else if ((SBR_n && MASTER_n) || BLOCKBG) begin
            SBG_n = 1;
        end
    end

    // after 1st sbg must block any further till unregistered and ebg deasserts
	 // Dorken: set if Master_n active
	 // Dorken: hold until REGED and EBG_n inactive
    always @(*) begin
        if (!MASTER_n) begin
            BLOCKBG <= 1;
        end else if (!REGED && EBG_n) begin
            BLOCKBG <= 0;
        end
    end

    // The Zorro III bus request is driven out on C7M high, for one C7M cycle, to
    // register for bus mastership.  When done, the same sequence relinquishes
    // registration.  The RCHNG signal indicated when a change is necessary.
    always @(negedge IORST_n, posedge clk7m) begin
        if (!IORST_n) begin
            EBR_n = 1;
        end else begin
            if (RCHNG && EBR_n) begin
                EBR_n = 0;
            end else begin
                EBR_n = 1;
            end
        end
    end

    // A change of registration is necessary whenever a SCSI request comes in
    // and we're unregistered, or when the MASTER line is dropped and we are
    // registered. dmaster is used to block regd & not master period at beginning
    always @(posedge clk7m) begin
        if ((!REGED && EBR_n && !SSBR_n) ||
            ( REGED && EBR_n && SMASTER_n && !DMASTER_n)) begin
            RCHNG <= 1;
        end else begin
            RCHNG <= 0;
        end
    end


    // Here's the actual registration indicator.  We're registered when EBR toggles,
    // unregistered the next time it toggles.  This can only change while EBR is low,
    // or in response to an error or reset condition.
	 always @(negedge IORST_n, negedge clk7m) begin
        if (!IORST_n) begin
            REGED <= 0;
        end else if (!EBR_n) begin
            REGED <= !REGED;
        end
    end

    // The A4091 has the Zorro III bus only if its registered and it receives a grant.
    // It holds the bus until the grant is removed and the cycle ends.
	 // dorken: set if REGED and EBG_n active
	 // dorken: hold until FCS_n inactive or IORST active
    always @(*) begin
        if (!IORST_n) begin
            mybus <= 0;
        end else if (REGED && !EBG_n) begin
            mybus <= 1;
		end else if (FCS_n) begin
            mybus <= 0;
        end
    end

    // MASTER_n is synch to 33M clock we must synch to 7m to avoid metastability.
    // DMASTER ist SMASTER delayed by 1 7MHz Clock
    // SBR is synch to 33M clock we must synch to 7m to avoid metastability.
    // SBG is not resynched to 33m since we use asynch fast arbitration mode
	always @(negedge IORST_n, posedge clk7m) begin
        if (!IORST_n) begin
            SMASTER_n <= 1;
            DMASTER_n <= 1;
			SSBR_n <= 1;
        end else begin
            SMASTER_n <= MASTER_n;
            DMASTER_n <= SMASTER_n;
            SSBR_n <= SBR_n;
        end		
    end

endmodule
