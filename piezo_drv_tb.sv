// piezo_drv_tb.sv
// Unit testbench for piezo_drv tone/fanfare state sequencing.
// Validates response to fanfare and battery-low inputs.
module piezo_drv_tb();

    // Inputs
    logic clk;
    logic rst_n;
    logic batt_low;
    logic fanfare;

    // Outputs
    logic piezo;
    logic piezo_n;

    // Instantiate the DUT
    piezo_drv #(.FAST_SIM(1)) iDUT (.clk(clk), .rst_n(rst_n), .batt_low(batt_low), .fanfare(fanfare), .piezo(piezo), .piezo_n(piezo_n));


    // Test sequence
    initial begin
        // Initialize inputs
        clk = 0;
        rst_n = 0;
        batt_low = 0;
        fanfare = 0;
        // Deassert reset after some time
        repeat(4) @(negedge clk);
        rst_n = 1'b1; // deassert reset
        repeat(2) @(negedge clk);

        // Test 1: Piezo should be silent before batt_low or fanfare is asserted
        repeat(10) @(negedge clk);
        if (piezo !== 1'b0 || piezo_n !== 1'b0) begin
            $display("ERROR: piezo/piezo_n should be silent in IDLE before fanfare.");
            $stop();
        end
        $display("Test 1 passed: piezo silent before fanfare or batt_low.");

        // Test 2: Assert fanfare for 1 clk 
        repeat(10) @(negedge clk);
        fanfare = 1'b1;
        @(negedge clk);
        fanfare = 1'b0;

        // Check if piezo is toggling
        repeat(2000) @(negedge clk);
        if (piezo === 1'b0 && piezo_n === 1'b0) begin
            $display("ERROR: piezo/piezo_n should be toggling during fanfare.");
            $stop();
        end
        $display("Piezo toggling during fanfare.");

        // Also check that piezo and piezo_n are always inversed
        repeat(2000) @(negedge clk);
        if (piezo === piezo_n) begin
            $display("ERROR: piezo and piezo_n should be inversed.");
            $stop();
        end
        $display("Test 2 passed: piezo toggling and piezo/piezo_n are inversed during fanfare.");

        // Test 3: Wait until fanfare finishes and check that piezo goes silent again
        repeat(4000000) @(negedge clk);
        if (piezo !== 1'b0 || piezo_n !== 1'b0) begin
            $display("ERROR: piezo/piezo_n should be silent after fanfare finishes.");
            $stop();
        end
        $display("Test 3 passed: piezo silent after fanfare finishes.");

        // Test 4: Assert batt_low for 1 clk and check that piezo starts toggling
        repeat(10) @(negedge clk);
        batt_low = 1'b1;
        
        repeat(2000) @(negedge clk);
        if (piezo === 1'b0 && piezo_n === 1'b0) begin
            $display("ERROR: piezo/piezo_n should be toggling during batt_low.");
            $stop();
        end

        // Also check that piezo and piezo_n are always inversed
        repeat(2000) @(negedge clk);
        if (piezo === piezo_n) begin
            $display("ERROR: piezo and piezo_n should be inversed.");
            $stop();
        end

        // Check that piezo is toggling for a while (batt_low_run should be 1)
        repeat(4000000) @(negedge clk);
        if (piezo === 1'b0 && piezo_n === 1'b0) begin
            $display("ERROR: piezo/piezo_n should still be toggling during batt_low_run.");
            $stop();
        end

        $display("Test 4 passed: piezo toggling during batt_low and continues toggling indefinitely.");

        // Test 5: Wait until batt_low finishes and check that piezo goes silent again
        repeat(10) @(negedge clk);
        batt_low = 1'b0;
        repeat(4000000) @(negedge clk);
        if (piezo !== 1'b0 || piezo_n !== 1'b0) begin
            $display("ERROR: piezo/piezo_n should be silent after batt_low finishes.");
            $stop();
        end
        $display("Test 5 passed: piezo silent after batt_low finishes.");

        

        // All tests passed
        $display("");
        $display("========================================");
        $display("Yahoo, all tests passed!");
        $display("========================================");
        $display("");
        $display("Group Members: Bhanu Kankanala, Arnav Mohanty, Soham Kundu, Mohnish Nanthakumar");
        $stop();

    end



    // clock generation
    always #10 clk = ~clk;

endmodule