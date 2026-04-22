// maze_solve_tb.sv
// Unit testbench for maze_solve autonomous-state behavior.
// Checks start-move/start-heading outputs for representative openings/events.
module maze_solve_tb();

    // Signal declarations
    logic clk, rst_n;
    logic cmd_md, cmd0;
    logic lft_opn, rght_opn;
    logic sol_cmplt;
    logic mv_cmplt;

    logic strt_hdng, strt_mv;
    logic stp_lft, stp_rght;
    logic [11:0] dsrd_hdng;

    // Shift register for mv_cmplt delay (Slide 8 architecture)
    logic [2:0] delay_regs;
    

    // Instantiate the Design Under Test (DUT)
    maze_solve iDUT (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_md(cmd_md),
        .cmd0(cmd0),
        .lft_opn(lft_opn),
        .rght_opn(rght_opn),
        .mv_cmplt(mv_cmplt),
        .sol_cmplt(sol_cmplt),
        .strt_hdng(strt_hdng),
        .dsrd_hdng(dsrd_hdng),
        .strt_mv(strt_mv),
        .stp_lft(stp_lft),
        .stp_rght(stp_rght)
    );

    // Clock generation (50MHz nominal)
    always #5 clk = ~clk;

    // Feedback loop logic to generate mv_cmplt
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            delay_regs <= 3'b000;
        else
            // OR gate feeding into 3 chained D-Flip-Flops
            delay_regs <= {delay_regs[1:0], (strt_hdng | strt_mv)};
    end
    
    // The output of the 3rd flip-flop connects back to mv_cmplt
    assign mv_cmplt = delay_regs[2];

    initial begin
        // 1. Initialize inputs
        clk = 0;
        rst_n = 0;
        cmd_md = 1; // Keep high to stay in IDLE
        cmd0 = 1;   // 1 configures left affinity
        lft_opn = 0;
        rght_opn = 0;
        sol_cmplt = 0;

        // 2. De-assert reset
        #15 rst_n = 1;

        // 3. Kick off the state machine
        // cmd_md going low kicks the SM out of IDLE
        #10 cmd_md = 0;

        // --- Test Case 1: Initial Forward Movement ---
        @(posedge strt_mv);
        $display("PASS: State machine initiated movement (strt_mv asserted).");

        // Set up the environment for when the current move completes.
        // We will assert lft_opn to test the left affinity branch.
        #10 lft_opn = 1; 
        
        // Wait for the state machine to command a turn
        @(posedge strt_hdng);
        $display("PASS: Left opening detected, heading turn initiated (strt_hdng asserted).");

        // --- Test Case 2: Discovering the Solution ---
        // Wait for the turn to complete (mv_cmplt will go low, then high, then low)
        @(negedge mv_cmplt);
        lft_opn = 0;
        sol_cmplt = 1; // Simulate the Hall effect sensor detecting the magnet

        // Wait a few cycles to ensure the SM processes the solution and enters DONE
        repeat(10) @(posedge clk);
        
        // In the DONE state, the system should stop issuing movement commands
        if (!strt_mv && !strt_hdng)
            $display("PASS: Movement stopped successfully after solution found.");
        else
            $error("FAIL: State machine continued to assert movement commands.");

        $display("Simulation complete.");
        $stop;
    end

endmodule