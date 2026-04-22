`timescale 1ns/1ps

module maze_solve_tb();

  logic clk, rst_n;
  logic cmd_md, cmd0;
  logic lft_opn, rght_opn;
  logic mv_cmplt, sol_cmplt;
  logic strt_hdng;
  logic [11:0] dsrd_hdng;
  logic strt_mv;
  logic stp_lft, stp_rght;

  maze_solve iDUT (
    .clk(clk), .rst_n(rst_n),
    .cmd_md(cmd_md), .cmd0(cmd0),
    .lft_opn(lft_opn), .rght_opn(rght_opn),
    .mv_cmplt(mv_cmplt), .sol_cmplt(sol_cmplt),
    .strt_hdng(strt_hdng), .dsrd_hdng(dsrd_hdng),
    .strt_mv(strt_mv), .stp_lft(stp_lft), .stp_rght(stp_rght)
  );

  initial clk = 0;
  always #5 clk = ~clk;

  initial begin
    rst_n    = 0; cmd_md = 1; cmd0 = 1;
    lft_opn  = 0; rght_opn = 0;
    mv_cmplt = 0; sol_cmplt = 0;
    repeat(2) @(negedge clk);
    rst_n = 1;

    // Start solve mode
    @(negedge clk); cmd_md = 0;

    // Let it issue a move
    @(negedge clk);
    if (!strt_mv) begin
      $display("FAIL: strt_mv not asserted"); $stop;
    end
    $display("PASS: strt_mv asserted");

    // Complete the move with left opening available
    repeat(3) @(negedge clk);
    mv_cmplt = 1; lft_opn = 1;
    @(negedge clk); mv_cmplt = 0; lft_opn = 0;

    // Should issue a left turn (strt_hdng)
    @(negedge clk);
    if (!strt_hdng) begin
      $display("FAIL: strt_hdng not asserted after left opening"); $stop;
    end
    $display("PASS: strt_hdng asserted for left turn");

    // Complete the turn
    repeat(3) @(negedge clk);
    mv_cmplt = 1;
    @(negedge clk); mv_cmplt = 0;

    // Trigger sol_cmplt
    repeat(3) @(negedge clk);
    mv_cmplt = 1; sol_cmplt = 1;
    @(negedge clk); mv_cmplt = 0; sol_cmplt = 0;

    $display("PASS: maze_solve_tb all tests passed");
    $display("Group Members: Bhanu Kankanala, Arnav Mohanty, Soham Kundu, Mohnish Nanthakumar");
    $stop;
  end

endmodule
