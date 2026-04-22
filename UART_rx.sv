// UART_rx.sv
// Byte-oriented UART receiver (19200 baud at 50 MHz clock).
// Detects start bit, samples incoming bits, and raises rdy when a byte is valid.
module UART_rx (
    input clk, // 50 MHz clock
    input rst_n, // active low reset
    input RX, // serial data input
    input clr_rdy, // knocks down rdy when asserted
    output [7:0] rx_data, // received byte
    output rdy // asserted when byte received, stays high until clr_rdy or next start bit
);


logic start; // asserted for 1 clock cycle when start bit is detected
logic receiving; // asserted when bits are being received
logic shift; // asserted when a bit should be sampled (middle of baud period)
logic set_rdy; // asserted when all bits have been received


logic RX_ff1, RX_ff2; // 2-flop synchronizer for metastability on RX input

logic [3:0] bit_cnt; // counts bits received
logic [8:0] rx_shft_reg; // 9-bit shift register to hold received bits
logic [11:0] baud_cnt; // 12-bit baud counter, counts down
logic rdy_ff; // SR flip-flop for rdy output


typedef enum logic { IDLE, RECEIVING } state_t; // state machine states
state_t state, nxt_state; // current and next state variables

// handles metastability on RX input
always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        RX_ff1  <= 1'b1; // reset to idle high
        RX_ff2 <= 1'b1; // reset to idle high
    end
    else begin
        RX_ff1  <= RX; // first flop
        RX_ff2 <= RX_ff1; // second flop - synchronized RX signal
    end
end

// State machine register logic
always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE; // reset to IDLE state
    end
    else begin
        state <= nxt_state; // state transition
    end
end

// State machine combinational logic
always_comb begin
    // default values
    start = 1'b0; // default to not starting
    receiving = 1'b0; // default to not receiving
    nxt_state = state; // default to no state change

    case (state)
        IDLE: begin
            if (!RX_ff2) begin // falling edge of RX indicates start bit
                start = 1'b1; // initialize counters and shift register
                nxt_state = RECEIVING; // transition to RECEIVING state
            end
        end
        RECEIVING: begin
            receiving = 1'b1; // indicate that reception is in progress
            if (set_rdy) begin // if all bits have been received
                nxt_state = IDLE;
            end
        end
        default: nxt_state = IDLE; // default to IDLE state
    endcase
end

// shift logic when baud_cnt reaches 0
assign shift   = receiving && (baud_cnt == 12'd0);
// set_rdy is asserted when the last bit is received
assign set_rdy = shift && (bit_cnt == 4'd9);

// bit counter - reset on start, increment on each shift
always_ff @(posedge clk) begin
    if (start) begin
        bit_cnt <= 4'h0; // reset bit counter when start bit detected
    end
    else if (shift) begin
        bit_cnt <= bit_cnt + 1'b1; // increment each baud period
    end
    else if (!start && !shift) begin
        bit_cnt <= bit_cnt; // hold value when not shifting
    end
end

// baud counter - counts down
// always_ff @(posedge clk) begin
//     if (start || shift) begin
//         baud_cnt <= 12'd1301; // half baud period offset to sample in middle of start bit
//     end
//     else if ((!start && !shift) && receiving) begin
//         baud_cnt <= baud_cnt - 1'b1; // count down each clock cycle
//     end
//     else if ((!start && !shift) && !receiving) begin
//         baud_cnt <= baud_cnt; // hold value when not receiving
//     end
// end
always_ff @(posedge clk) begin
    if (start) begin
        baud_cnt <= 12'd1302; // half period - to sample middle of start bit
    end
    else if (shift) begin
        baud_cnt <= 12'd2604; // full period - for all subsequent bits
    end
    else if ((!start && !shift) && receiving) begin
        baud_cnt <= baud_cnt - 1'b1; // count down
    end
    else if ((!start && !shift) && !receiving) begin
        baud_cnt <= baud_cnt; // hold value when not receiving
    end
end
// shift register - shift right on each shift, fill MSB with RX_ff2
always_ff @(posedge clk) begin
    if (start) begin
        rx_shft_reg <= 9'h1FF; // reset shift register to idle high on start bit
    end
    else if (shift) begin
        rx_shft_reg <= {RX_ff2, rx_shft_reg[8:1]}; // shift right, fill MSB with RX_ff2
    end
    else if (!start && !shift) begin
        rx_shft_reg <= rx_shft_reg; // hold value when not shifting
    end
end
// always_ff @(posedge clk, negedge rst_n) begin
//     if (!rst_n) begin
//         rx_shft_reg <= 9'h1FF; // reset to idle high
//     end
//     else if (shift) begin
//         rx_shft_reg <= {RX_ff2, rx_shft_reg[8:1]}; // shift right, fill MSB with RX
//     end
// end

assign rx_data = rx_shft_reg[7:0]; // D7 down to D0 after all data bits shifted in

// rdy SR flip-flop - set when all bits received, reset on new start bit or clr_rdy
always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        rdy_ff <= 1'b0; // reset rdy flip-flop
    end
    else if (start || clr_rdy) begin
        rdy_ff <= 1'b0; // clear rdy on new transmission or clr_rdy
    end
    else if (set_rdy) begin
        rdy_ff <= 1'b1; // set rdy when byte is fully received
    end
end

assign rdy = rdy_ff;


endmodule
