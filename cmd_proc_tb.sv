module cmd_proc_tb;

    logic clk;
    logic rst_n;

    logic [15:0] cmd;
    logic cmd_rdy;
    logic clr_cmd_rdy;
    logic send_resp;

    logic strt_cal;
    logic cal_done;
    logic in_cal;

    logic strt_hdng;
    logic strt_mv;
    logic stp_lft;
    logic stp_rght;
    logic [11:0] dsrd_hdng;
    logic mv_cmplt;

    logic cmd_md;
    logic sol_cmplt;

    // DUT
    cmd_proc dut (
        .clk(clk),
        .rst_n(rst_n),
        .cmd(cmd),
        .cmd_rdy(cmd_rdy),
        .clr_cmd_rdy(clr_cmd_rdy),
        .send_resp(send_resp),
        .strt_cal(strt_cal),
        .cal_done(cal_done),
        .in_cal(in_cal),
        .strt_hdng(strt_hdng),
        .strt_mv(strt_mv),
        .stp_lft(stp_lft),
        .stp_rght(stp_rght),
        .dsrd_hdng(dsrd_hdng),
        .mv_cmplt(mv_cmplt),
        .cmd_md(cmd_md),
        .sol_cmplt(sol_cmplt)
    );


    // 10 ns clock period
    initial clk = 1'b0;
    always #5 clk = ~clk;


    integer timeout_cnt;


    // Waits for a signal to go high, checking once per clock cycle. Stops the simulation with an error if the signal does not assert within max_clks clock cycles.
    task automatic wait_for_signal(input string name, ref logic sig, input int max_clks = 100);
        timeout_cnt = 0;
        while (!sig && timeout_cnt < max_clks) begin
            @(posedge clk);
            timeout_cnt++;
        end

        if (!sig) begin
            $display("ERROR: Timed out waiting for %s", name);
            $stop;
        end
    endtask


    initial begin
        rst_n = 1'b0;
        cmd = 16'h0000;
        cmd_rdy = 1'b0;
        cal_done = 1'b0;
        mv_cmplt = 1'b0;
        sol_cmplt = 1'b0;

        repeat (2) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);


        // Test 1: Reset behavior
        $display("Test 1: Checking reset state...");
        if (cmd_md !== 1'b1) begin
            $display("ERROR: cmd_md should be high after reset");
            $stop;
        end

        if (send_resp !== 1'b0 || strt_cal !== 1'b0 || strt_hdng !== 1'b0 || strt_mv !== 1'b0) begin
            $display("ERROR: outputs should be deasserted after reset");
            $stop;
        end
        $display("  PASS");


        // Test 2: Calibrate command
        $display("Test 2: Calibrate command...");

        @(negedge clk);
        cmd = 16'h0000;
        cmd_rdy = 1'b1;

        #1;
        if (clr_cmd_rdy !== 1'b1) begin
            $display("ERROR: clr_cmd_rdy not asserted on cmd acceptance");
            $stop;
        end
        if (strt_cal !== 1'b1) begin
            $display("ERROR: strt_cal not asserted");
            $stop;
        end

        @(posedge clk);

        @(negedge clk);
        cmd_rdy = 1'b0;

        #1;
        if (in_cal !== 1'b1) begin
            $display("ERROR: in_cal should be high during calibration");
            $stop;
        end
        if (strt_cal !== 1'b0) begin
            $display("ERROR: strt_cal should be deasserted after 1 clk");
            $stop;
        end

        repeat (3) @(negedge clk);
        cal_done = 1'b1;

        @(negedge clk);
        cal_done = 1'b0;

        wait_for_signal("send_resp (cal)", send_resp);
        $display("  PASS");

        repeat (2) @(negedge clk);


        // Test 3: Heading command
        $display("Test 3: Heading command (West)...");

        @(negedge clk);
        cmd = 16'h23FF;
        cmd_rdy = 1'b1;

        #1;
        if (clr_cmd_rdy !== 1'b1) begin
            $display("ERROR: clr_cmd_rdy not asserted on heading cmd acceptance");
            $stop;
        end
        if (strt_hdng !== 1'b1) begin
            $display("ERROR: strt_hdng not asserted");
            $stop;
        end

        @(posedge clk);

        @(negedge clk);
        cmd_rdy = 1'b0;

        #1;
        if (dsrd_hdng !== 12'h3FF) begin
            $display("ERROR: dsrd_hdng = %h, expected 3FF", dsrd_hdng);
            $stop;
        end
        if (strt_hdng !== 1'b0) begin
            $display("ERROR: strt_hdng should be deasserted after 1 clk");
            $stop;
        end

        repeat (3) @(negedge clk);
        mv_cmplt = 1'b1;

        @(negedge clk);
        mv_cmplt = 1'b0;

        wait_for_signal("send_resp (hdng)", send_resp);
        $display("  PASS");

        repeat (2) @(negedge clk);


        // Test 4: Move command
        $display("Test 4: Move command (stop left)...");

        @(negedge clk);
        cmd = 16'h4002;
        cmd_rdy = 1'b1;

        #1;
        if (clr_cmd_rdy !== 1'b1) begin
            $display("ERROR: clr_cmd_rdy not asserted on move cmd acceptance");
            $stop;
        end
        if (strt_mv !== 1'b1) begin
            $display("ERROR: strt_mv not asserted");
            $stop;
        end

        @(posedge clk);

        @(negedge clk);
        cmd_rdy = 1'b0;

        #1;
        if (stp_lft !== 1'b1) begin
            $display("ERROR: stp_lft should be 1");
            $stop;
        end
        if (stp_rght !== 1'b0) begin
            $display("ERROR: stp_rght should be 0");
            $stop;
        end

        repeat (3) @(negedge clk);
        mv_cmplt = 1'b1;

        @(negedge clk);
        mv_cmplt = 1'b0;

        wait_for_signal("send_resp (move)", send_resp);
        $display("  PASS");

        repeat (2) @(negedge clk);


        // Test 5: Solve command
        $display("Test 5: Solve command...");

        @(negedge clk);
        cmd = 16'h6000;
        cmd_rdy = 1'b1;

        #1;
        if (clr_cmd_rdy !== 1'b1) begin
            $display("ERROR: clr_cmd_rdy not asserted for solve cmd");
            $stop;
        end

        @(posedge clk);

        @(negedge clk);
        cmd_rdy = 1'b0;

        #1;
        if (cmd_md !== 1'b0) begin
            $display("ERROR: cmd_md should be low during solve");
            $stop;
        end

        repeat (5) @(negedge clk);
        sol_cmplt = 1'b1;

        @(negedge clk);
        sol_cmplt = 1'b0;

        wait_for_signal("send_resp (solve)", send_resp);

        repeat (2) @(posedge clk);
        #1;
        if (cmd_md !== 1'b1) begin
            $display("ERROR: cmd_md should return high after solve completes");
            $stop;
        end
        $display("  PASS");


        $display("========================================");
        $display("All cmd_proc tests PASSED");
        $display("========================================");
        $display("");
        $display("Group Members: Bhanu Kankanala, Arnav Mohanty, Soham Kundu, Mohnish Nanthakumar");
        $stop;
    end

endmodule