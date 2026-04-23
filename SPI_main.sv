///////////////////////////////////////////////////////////////////////////////              
// SPI controller; transfers 16 bits full-duplex (MSB-first) with           //
// CPOL=1, CPHA=1 timing. A 5-bit clock divider generates SCLK from the    //
// system clock. The module manages front-porch and back-porch            //
// timing around SS_n so that the slave has time to set up before the    //
// first SCLK edge and the final bit is captured before SS_n de-asserts.//
/////////////////////////////////////////////////////////////////////////


module SPI_main(
    input  logic        clk,        // system clock
    input  logic        rst_n,      // active-low asynchronous reset
    input  logic        wrt,        // pulse to begin a 16-bit SPI transaction
    input  logic [15:0] wt_data,    // data to shift out on MOSI
    output logic        SS_n,       // active-low slave select
    output logic        SCLK,       // serial clock to slave
    output logic        MOSI,       // master-out / slave-in (MSB of shift reg)
    input  logic        MISO,       // master-in / slave-out
    output logic        done,       // asserted for one cycle when transfer completes
    output logic [15:0] rd_data     // data shifted in from MISO
);

    logic [15:0] shft_reg;          // bidirectional shift register (TX & RX)
    logic [4:0]  sclk_div;          // free-running divider → SCLK is bit [4]
    logic        active;            // high while a transaction is in progress
    logic        first_fall_seen;   // suppresses shifting on the very first falling edge (front porch)
    logic [4:0]  sample_cnt;        // counts MISO samples (0–15 → 16 bits)
    logic        miso_smpl;         // latched MISO value awaiting the next shift
    logic        finish_pending;    // set after 16th sample; transfer ends on next fall


    // smpl asserts one system-clock cycle before SCLK's rising edge
    // shft_imm asserts one system-clock cycle before SCLK's falling edge
    wire smpl = active && (sclk_div == 5'b01111);
    wire shft_imm = active && (sclk_div == 5'b11111);

    assign SS_n = ~active;
    assign SCLK = active ? sclk_div[4] : 1'b1;   // CPOL=1: idle high
    assign MOSI = shft_reg[15];                   // MSB-first output
    assign rd_data = shft_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shft_reg        <= 16'h0000;
            sclk_div        <= 5'b10111;   // preset near rollover for initial front porch
            active          <= 1'b0;
            first_fall_seen <= 1'b0;
            sample_cnt      <= 5'd0;
            miso_smpl       <= 1'b0;
            finish_pending  <= 1'b0;
            done            <= 1'b0;
        end else begin
            // Transaction kick-off
            if (wrt) begin
                shft_reg        <= wt_data;
                sclk_div        <= 5'b10111;   // front-porch setup
                active          <= 1'b1;
                first_fall_seen <= 1'b0;
                sample_cnt      <= 5'd0;
                miso_smpl       <= 1'b0;
                finish_pending  <= 1'b0;
                done            <= 1'b0;
            end
            else if (active) begin
                sclk_div <= sclk_div + 5'd1;  // free-run the divider

                // MISO sampling (just before rising SCLK edge)
                if (smpl) begin
                    miso_smpl  <= MISO;
                    sample_cnt <= sample_cnt + 5'd1;

                    // 16th sample taken — schedule end-of-transfer on next fall
                    if (sample_cnt == 5'd15) begin
                        finish_pending <= 1'b1;
                    end
                end

                // Shift register update (just before falling SCLK edge)
                if (shft_imm) begin
                    if (!first_fall_seen) begin
                        // Suppress shifting on the first fall (front porch)
                        first_fall_seen <= 1'b1;
                    end
                    else begin
                        shft_reg <= {shft_reg[14:0], miso_smpl};

                        // Back-porch: deassert active after the final shift
                        if (finish_pending) begin
                            active         <= 1'b0;
                            done           <= 1'b1;
                            finish_pending <= 1'b0;
                        end
                    end
                end
            end
        end
    end

endmodule
