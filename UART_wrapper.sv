// UART_wrapper.sv
// Protocol wrapper around UART TX/RX.
// Packs two received bytes into a 16-bit command and transmits 8-bit responses.
module UART_wrapper (
    input clk, rst_n, // 50 MHz clock and asynch active low reset
    input RX, // receive line (19200 baud)
    output cmd_rdy, // When cmd_rdy is asserted, cmd is the 16-bit command received
    output [15:0] cmd, // When cmd_rdy is asserted, cmd is the 16-bit command received
    input clr_cmd_rdy, // Used to knock down cmd_rdy. Bookkeeping for consumer.
    input trmt, //resp is sent to Bluetooth module upon a pulse on trmt
    input [7:0] resp, // resp is sent to Bluetooth module upon a pulse on trmt
    output tx_done, // Asserted when resp has been sent
    output TX // transmit line (19200 baud)
);


    // Internal signals for connecting UART_rx and UART_tx using UART
    logic rx_rdy;       
    logic clr_rdy;      
    logic [7:0] rx_data;      
 
    logic [7:0] data_high_bits;
    logic set_high_bits;    
    logic set_cmd_rdy;  
    logic cmd_rdy_ff;  

    // Instantiate UART
    UART iUART(.clk(clk), .rst_n(rst_n), .RX(RX), .TX(TX), .rx_rdy(rx_rdy),
                .clr_rx_rdy(clr_rdy), .rx_data(rx_data), .trmt(trmt),
                .tx_data(resp), .tx_done(tx_done));


    // Output logic for cmd
    always_ff @(posedge clk) begin
        if (set_high_bits) begin
            data_high_bits <= rx_data; // capture high bits of command
        end
        else begin
            data_high_bits <= data_high_bits; // hold previous value
        end
    end
    assign cmd = {data_high_bits, rx_data}; // concatenate high and low bits for full command

    // Logic for cmd_rdy signal
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            cmd_rdy_ff <= 1'b0; // reset cmd_rdy_ff to 0
        end
        else if (clr_cmd_rdy) begin
            cmd_rdy_ff <= 1'b0; // clear cmd_rdy_ff when clr_cmd_rdy is asserted
        end
        else if (set_cmd_rdy) begin
            cmd_rdy_ff <= 1'b1; // set cmd_rdy_ff when set_cmd_rdy is asserted
        end
        else begin
            cmd_rdy_ff <= cmd_rdy_ff; // hold previous value
        end
    end
    assign cmd_rdy = cmd_rdy_ff; // output cmd_rdy signal    
    
    typedef enum logic {IDLE, WAIT_LOW} state_t; // state machine states
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
    
    always_comb begin
        // default outputs
        set_high_bits = 1'b0; // default to not capturing high bits
        clr_rdy = 1'b0; // default to not clearing rx_rdy
        set_cmd_rdy = 1'b0; // default to not setting cmd_rdy
        nxt_state = state; // default to no state change

        case(state) 
            IDLE: begin
                if (rx_rdy) begin // if a byte is received
                    set_high_bits = 1'b1; // capture high bits of command
                    clr_rdy = 1'b1; // clear rx_rdy for next byte
                    nxt_state = WAIT_LOW; // wait for low bits of command
                end   
            end
            WAIT_LOW: begin
                if (rx_rdy) begin // if low bits of command are received
                    set_cmd_rdy = 1'b1; // set cmd_rdy to indicate full command is ready
                    clr_rdy = 1'b1; // clear rx_rdy for next command
                    nxt_state = IDLE; // go back to IDLE state
                end
            end 

            default: nxt_state = IDLE; // default to IDLE state
        endcase


    end

endmodule
