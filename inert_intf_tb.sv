`timescale 1ns/1ps

// inert_intf_tb.sv
// Unit testbench for inert_intf gyro/SPI bring-up and heading-valid handshakes.
// Drives calibration and emulated sensor interrupt traffic.
module inert_intf_tb;

  logic clk;
  logic rst_n;
  logic strt_cal;
  logic cal_done;
  logic signed [11:0] heading;
  logic rdy;
  logic [8:0] IR_Dtrm;
  logic SS_n;
  logic SCLK;
  logic MOSI;
  logic MISO;
  logic INT;
  logic moving;
  logic en_fusion;

  int rdy_count;
  logic check_run;
  logic saw_heading_change;
  logic signed [11:0] prev_heading;

  // DUT is run in FAST_SIM so calibration finishes in a reasonable time
  inert_intf #(.FAST_SIM(1)) DUT(
    .clk(clk),
    .rst_n(rst_n),
    .strt_cal(strt_cal),
    .cal_done(cal_done),
    .heading(heading),
    .rdy(rdy),
    .IR_Dtrm(IR_Dtrm),
    .SS_n(SS_n),
    .SCLK(SCLK),
    .MOSI(MOSI),
    .MISO(MISO),
    .INT(INT),
    .moving(moving),
    .en_fusion(en_fusion)
  );

  // SPI pins
  SPI_iNEMO2 iNEMO(
    .SS_n(SS_n),
    .SCLK(SCLK),
    .MISO(MISO),
    .MOSI(MOSI),
    .INT(INT)
  );

  // 50 MHz clock, so one full period is 20 ns
  initial clk = 1'b0;
  always #10 clk = ~clk;

  // Wait for the sensor model to say setup is complete
  // The timeout keeps the test from hanging forever if init fails
  task automatic wait_for_nemo_setup(input int max_clks);
    bit got_it;
    begin
      got_it = 1'b0;

      fork
        begin
          wait (iNEMO.NEMO_setup === 1'b1);
          got_it = 1'b1;
        end

        begin
          repeat (max_clks) @(posedge clk);
        end
      join_any

      disable fork;

      if (!got_it)
        $fatal(1, "Timeout waiting for NEMO_setup");
    end
  endtask

  // Wait for calibration to finish
  // This checks that the DUT is reading enough gyro samples for the integrator
  task automatic wait_for_cal_done(input int max_clks);
    bit got_it;
    begin
      got_it = 1'b0;

      fork
        begin
          wait (cal_done === 1'b1);
          got_it = 1'b1;
        end

        begin
          repeat (max_clks) @(posedge clk);
        end
      join_any

      disable fork;

      if (!got_it)
        $fatal(1, "Timeout waiting for cal_done");
    end
  endtask

  // Count ready pulses after calibration
  // Also check that heading is real data, not X or Z
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rdy_count <= 0;
      saw_heading_change <= 1'b0;
      prev_heading <= 12'sh000;
    end else if (!check_run) begin
      rdy_count <= 0;
      saw_heading_change <= 1'b0;
      prev_heading <= heading;
    end else if (rdy) begin
      rdy_count <= rdy_count + 1;

      assert (!$isunknown(heading))
        else $fatal(1, "heading went X/Z when rdy was high");

      if (heading !== prev_heading)
        saw_heading_change <= 1'b1;

      prev_heading <= heading;
    end
  end

  initial begin
    $display("Running inert_intf_tb for akmohanty");

    // Start with everything in a quiet known state
    rst_n = 1'b0;
    strt_cal = 1'b0;
    moving = 1'b1;
    en_fusion = 1'b0;
    IR_Dtrm = 9'h000;
    check_run = 1'b0;

    // Hold reset for a few clocks before letting the DUT run
    repeat (5) @(negedge clk);
    rst_n = 1'b1;

    // The DUT should write the setup registers inside the sensor model
    wait_for_nemo_setup(200000);
    $display("NEMO setup complete at %0t", $time);

    // Pulse strt_cal for exactly one clock cycle.
    @(negedge clk);
    strt_cal = 1'b1;
    @(negedge clk);
    strt_cal = 1'b0;

    // Calibration should complete after the integrator sees enough samples
    wait_for_cal_done(1000000);
    $display("Calibration complete at %0t", $time);

    // Now start checking normal post-calibration behavior
    check_run = 1'b1;

    // Let the interface run long enough to get many gyro readings
    repeat (8000000) @(posedge clk);

    // The DUT should keep producing new completed heading updates
    assert (rdy_count > 10)
      else $fatal(1, "DUT did not produce enough rdy pulses after calibration");

    // Heading should not stay stuck forever after calibration
    assert (saw_heading_change)
      else $fatal(1, "heading never changed after calibration");

    $display("Self-checks passed: rdy_count=%0d, final heading=%0d", rdy_count, heading);
    $display("Finished post-calibration run at %0t", $time);

    $stop;
  end

endmodule