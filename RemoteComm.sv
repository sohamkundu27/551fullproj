module RemoteComm (
    input clk, rst_n,   // 50 MHz clock and asynch active low reset
    input [15:0] cmd,   // 16-bit command to send
    input snd_cmd,        // pulse to initiate transmission
    output cmd_snt,    // asserted when full 16-bit command has been sent
    output TX,   // UART transmit line to UART_wrapper
    input RX,    // UART receive line from UART_wrapper
    output [7:0] resp,     // byte received from UART_wrapper
    output resp_rdy    // asserted when resp has been received
);


    // Internal signals
    logic [7:0] tx_data; // byte fed into UART tx_data
    logic [7:0] low_byte;   // holding register for cmd[7:0]
    logic trmt;     // start transmission pulse to UART
    logic tx_done;      // UART finished transmitting
    logic sel_high;    // select high byte of cmd to load into tx_data
    logic set_cmd_snt;    // pulse to set cmd_snt when full command has been sent
    logic cmd_snt_ff;   // indicates full command has been sent

    // Instantiate UART
    UART iUART(.clk(clk), .rst_n(rst_n), .RX(RX), .TX(TX), .rx_rdy(resp_rdy),
                .clr_rx_rdy(1'b0), // rx_rdy will be cleared by new start bit, so tie clr_rx_rdy to 0
                .rx_data(resp), .trmt(trmt),
                .tx_data(tx_data), .tx_done(tx_done));
    
    // Holding register for high byte of command
    always_ff @(posedge clk) begin
        if (snd_cmd) begin
            low_byte <= cmd[7:0]; // capture low byte of command on snd_cmd pulse
        end 
        else begin
            low_byte <= low_byte; // hold previous value
        end
    end

    // select between high and low byte of cmd to load into tx_data
    assign tx_data = sel_high ? cmd[15:8] : low_byte;

    // logic for cmd_snt signal
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            cmd_snt_ff <= 1'b0; // reset cmd_snt to 0
        end
        else if (snd_cmd) begin
            cmd_snt_ff <= 1'b0; // clear cmd_snt on new snd_cmd pulse
        end 
        else if (set_cmd_snt) begin
            cmd_snt_ff <= 1'b1; // set cmd_snt when set_cmd_snt is asserted
        end
        else begin
            cmd_snt_ff <= cmd_snt_ff; // hold previous value
        end
    end 
    assign cmd_snt = cmd_snt_ff; // output cmd_snt signal

    // State definitions and state logic
    typedef enum logic [1:0] {IDLE, SEND_LOW, WAIT_DONE} state_t;
    state_t state, nxt_state;

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
        // Default values for control signals and next state
        sel_high = 1'b0; // default to selecting low byte
        trmt = 1'b0; // default to not transmitting
        set_cmd_snt = 1'b0; // default to not setting cmd_snt
        nxt_state = state; // default to no state change

        case(state)

            IDLE: begin
                if (snd_cmd) begin // on snd_cmd pulse, start sending low byte
                    sel_high = 1'b1; // select high byte for transmission
                    trmt = 1'b1; // initiate transmission
                    nxt_state = SEND_LOW; // transition to SEND_LOW state
                end
            end

            SEND_LOW: begin
                sel_high = 1'b0; // select low byte for transmission
                if (tx_done) begin // when low byte is done transmitting
                    trmt = 1'b1; // initiate transmission of high byte
                    nxt_state = WAIT_DONE; // transition to WAIT_DONE state
                end
            end

            WAIT_DONE: begin
                if (tx_done) begin // when high byte is done transmitting
                    set_cmd_snt = 1'b1; // set cmd_snt to indicate full command has been sent
                    nxt_state = IDLE; // transition back to IDLE state
                end
            end

            default: nxt_state = IDLE; // default to IDLE state

        endcase
    end


endmodule


