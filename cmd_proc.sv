// cmd_proc.sv
// Decodes BLE/UART commands and issues high-level control actions:
// calibration, heading changes, forward moves, and response handshakes.
module cmd_proc (
    input logic clk,
    input logic rst_n,

    input logic [15:0] cmd,
    input logic cmd_rdy,
    output logic clr_cmd_rdy,
    output logic send_resp,

    output logic strt_cal,
    input logic cal_done,
    output logic in_cal,

    output logic strt_hdng,
    output logic strt_mv,
    output logic stp_lft,
    output logic stp_rght,
    output logic [11:0] dsrd_hdng,
    input logic mv_cmplt,

    output logic cmd_md,
    input logic sol_cmplt
);

    typedef enum logic [2:0] {
        IDLE = 3'd0,
        CAL = 3'd1,
        HDNG = 3'd2,
        MOVE = 3'd3,
        SOLVE = 3'd4,
        RESP = 3'd5
    } state_t;

    state_t state, nxt_state;

    logic [11:0] dsrd_hdng_nxt;
    logic stp_lft_nxt, stp_rght_nxt;

    // Next-state logic
    always_comb begin
        nxt_state = state;

        case (state)
            IDLE: begin
                if (cmd_rdy) begin
                    case (cmd[15:13])
                        3'b000: nxt_state = CAL;
                        3'b001: nxt_state = HDNG;
                        3'b010: nxt_state = MOVE;
                        3'b011: nxt_state = SOLVE;
                        default: nxt_state = IDLE;
                    endcase
                end
            end

            CAL: begin
                if (cal_done)
                    nxt_state = RESP;
            end

            HDNG: begin
                if (mv_cmplt)
                    nxt_state = RESP;
            end

            MOVE: begin
                if (mv_cmplt)
                    nxt_state = RESP;
            end

            SOLVE: begin
                if (sol_cmplt)
                    nxt_state = RESP;
            end

            RESP: begin
                nxt_state = IDLE;
            end

            default: begin
                nxt_state = IDLE;
            end
        endcase
    end

    // Registered data path
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            dsrd_hdng <= 12'h000;
            stp_lft <= 1'b0;
            stp_rght <= 1'b0;
        end
        else begin
            state <= nxt_state;

            // Latch command information when command is accepted
            if (state == IDLE && cmd_rdy) begin
                case (cmd[15:13])
                    3'b001: begin
                        dsrd_hdng <= cmd[11:0];
                        stp_lft <= 1'b0;
                        stp_rght <= 1'b0;
                    end

                    3'b010: begin
                        stp_lft <= cmd[1];
                        stp_rght <= cmd[0];
                    end

                    default: begin
                        stp_lft <= 1'b0;
                        stp_rght <= 1'b0;
                    end
                endcase
            end
            else if (state != MOVE) begin
                stp_lft <= 1'b0;
                stp_rght <= 1'b0;
            end
        end
    end

    // Output logic
    always_comb begin
        clr_cmd_rdy = 1'b0;
        send_resp = 1'b0;
        strt_cal = 1'b0;
        strt_hdng = 1'b0;
        strt_mv = 1'b0;
        in_cal = 1'b0;
        cmd_md = 1'b1;

        // Accept command in IDLE
        if (state == IDLE && cmd_rdy)
            clr_cmd_rdy = 1'b1;

        // 1-cycle start pulses on state transition out of IDLE
        if (state == IDLE && nxt_state == CAL)
            strt_cal = 1'b1;

        if (state == IDLE && nxt_state == HDNG)
            strt_hdng = 1'b1;

        if (state == IDLE && nxt_state == MOVE)
            strt_mv = 1'b1;

        // Stay in calibration mode while waiting
        if (state == CAL)
            in_cal = 1'b1;

        // Solve mode drives cmd_md low
        if (state == SOLVE)
            cmd_md = 1'b0;

        // Send response for one cycle
        if (state == RESP)
            send_resp = 1'b1;
    end

endmodule