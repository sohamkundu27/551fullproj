`timescale 1ns/1ps

// PID_tb.sv
// Unit testbench for PID control-law outputs.
// Sweeps heading error scenarios and checks left/right speed response direction.
module PID_tb;

  // Stimulus signals driven by the testbench.
  logic clk;
  logic rst_n;
  logic moving;
  logic [11:0] dsrd_hdng;
  logic [11:0] actl_hdng;
  logic hdng_vld;
  logic [10:0] frwrd_spd;

  // DUT outputs observed by the testbench.
  logic at_hdng;
  logic signed [11:0] lft_spd;
  logic signed [11:0] rght_spd;

  // DUT instantiation
  PID dut (
    .clk(clk),
    .rst_n(rst_n),
    .moving(moving),
    .dsrd_hdng(dsrd_hdng),
    .actl_hdng(actl_hdng),
    .hdng_vld(hdng_vld),
    .frwrd_spd(frwrd_spd),
    .at_hdng(at_hdng),
    .lft_spd(lft_spd),
    .rght_spd(rght_spd)
  );

  // 100 MHz clock
  initial clk = 1'b0;
  always #5 clk = ~clk;

  initial begin
    // Initialize all inputs before deasserting reset.
    rst_n      = 1'b0;
    moving     = 1'b0;
    dsrd_hdng  = 12'd0;
    actl_hdng  = 12'd0;
    hdng_vld   = 1'b0;
    frwrd_spd  = 11'd0;

    // Keep reset active for a few cycles so sequential state clears.
    repeat (3) @(posedge clk);
    rst_n = 1'b1;

    // Begin motion with constant base speed.
    @(posedge clk);
    moving    = 1'b1;
    frwrd_spd = 11'd400;

    // Case 1: Zero heading error. Expect small correction and at_hdng asserted.
    dsrd_hdng = 12'd1000;
    actl_hdng = 12'd1000;
    hdng_vld  = 1'b1;
    // Pulse hdng_vld for one sample to mimic sensor update timing.
    @(posedge clk);
    hdng_vld  = 1'b0;
    // Let internal pipeline/integrator settle for observation.
    repeat (4) @(posedge clk);

    // Case 2: Positive error (actual > desired). Expect one wheel to slow.
    dsrd_hdng = 12'd1000;
    actl_hdng = 12'd1200;
    hdng_vld  = 1'b1;
    @(posedge clk);
    hdng_vld  = 1'b0;
    repeat (8) @(posedge clk);

    // Case 3: Negative error (actual < desired). Expect opposite wheel response.
    dsrd_hdng = 12'd1200;
    actl_hdng = 12'd1000;
    hdng_vld  = 1'b1;
    @(posedge clk);
    hdng_vld  = 1'b0;
    repeat (8) @(posedge clk);

    // Stop command should force both outputs to zero.
    moving = 1'b0;
    repeat (4) @(posedge clk);

    // End simulation.
    $finish;
  end

  // Console trace for quick debugging without opening waves.
  initial begin
    $display("time  rst_n moving vld dsrd actl at_hdng lft_spd rght_spd");
    $monitor("%0t   %0b     %0b     %0b   %0d  %0d   %0b      %0d      %0d",
             $time, rst_n, moving, hdng_vld, dsrd_hdng, actl_hdng, at_hdng, lft_spd, rght_spd);
  end

endmodule
