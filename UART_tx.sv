module UART_tx (
    input clk, // 50 MHz clock
    input rst_n, // active low reset
    input [7:0] tx_data, //byte to transmit
    input trmt, // asserted for 1 clock cycle to initiate transmission
    output TX, // serial data output
    output tx_done // asserted when byte is done transmitting. Stays high until next trmt signal
);


logic init; // asserted for 1 clock cycle to initialize counters
logic transmitting; // asserted when bits are being transmitting
logic shift; // asserted when shifting bits
logic set_done; // asserted when shifting is done and tx_done should be set


logic [3:0] bit_cnt; // counts bits being transmitted
logic [8:0] tx_shift_reg; // holds the bits being shifted
logic [11:0] baud_cnt; // 12 bit baud counter for 19200 baud at 50 MHz
logic tx_done_ff; // flip-flop to hold tx_done signal until next transmission


typedef enum logic { IDLE, TRANSMIT } state_t; // state machine states
state_t state, nxt_state; // current and next state variables

// State machine register logic
always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n)  begin
        state <= IDLE; // reset to IDLE state
    end
    else begin        
        state <= nxt_state; // state transition
    end
end

// State machine combinational logic
always_comb begin
    // default values for outputs and control signals
    init = 1'b0; // default to initializing counters
    transmitting = 1'b0; // default to not transmitting
    nxt_state = state; // default to no state change

    case (state)
        IDLE: begin
            init = 1'b0; // deassert init in IDLE state
            transmitting = 1'b0; // deassert transmitting in IDLE state
            if (trmt) begin // if trmt is asserted, start transmission
                init = 1'b1; // initialize counters and shift register
                nxt_state = TRANSMIT; // transistion to TRANSMIT state
            end
        end
        TRANSMIT: begin
            init = 1'b0; // deassert init after first cycle in TRANSMIT state
            transmitting = 1'b1; // indicate that transmission is in progress
            if (set_done) begin // if all bits have been transmitted (10 bits total)
                nxt_state = IDLE;
            end
        end
        default : nxt_state = IDLE; // default to IDLE state
    endcase
end

assign shift = transmitting && (baud_cnt == 12'd2603); // shift bits at the correct baud rate (19200 baud at 50 MHz)
assign set_done = shift && (bit_cnt == 4'd9); // set tx_done when the last bit (stop bit) is being shifted out


// bit counter
always_ff @(posedge clk) begin
    if (init) begin
        bit_cnt <= 4'h0; // reset bit counter   
    end
    else if (!init && shift) begin
        bit_cnt <= bit_cnt + 1'b1; // increment bit counter
    end
    else if (!init && !shift) begin
        bit_cnt <= bit_cnt; // hold bit counter value
    end

end

// baud counter for 19200 baud at 50 MHz, 12 bit counter counts to 2604
always_ff @(posedge clk) begin
    if (init||shift) begin
        baud_cnt <= 12'h000; // reset baud counter
    end
    else if ((!init && !shift) && transmitting) begin
        baud_cnt <= baud_cnt + 1'b1; // increment baud counter
    end
    else if ((!init && !shift) && !transmitting) begin
        baud_cnt <= baud_cnt; // hold baud count value
    end
end

// shift register for transmitting bits
always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        tx_shift_reg <= 9'h1FF; // reset shift register
    end
    else if (init) begin
        tx_shift_reg <= {tx_data, 1'b0}; // load shift register with data bits and stop bit
    end
    else if (!init && shift) begin
        tx_shift_reg <= {1'b1, tx_shift_reg[8:1]}; // shift bits to the right, filling in 1's on the left for stop bit
    end
    else if (!init && !shift) begin
        tx_shift_reg <= tx_shift_reg; // hold shift register value
    end
end
assign TX = tx_shift_reg[0]; // output the least significant bit of the shift register as the serial data output

// tx_done flip-flop logic
always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        tx_done_ff <= 1'b0; // reset tx_done flip-flop
    end
    else if (set_done) begin
        tx_done_ff <= 1'b1; // set tx_done when transmission is complete
    end
    else if (init) begin
        tx_done_ff <= 1'b0; // clear tx_done when not set_done
    end
end
assign tx_done = tx_done_ff; // output the value of the tx_done flip-flop
  



endmodule
