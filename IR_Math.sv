// IR_Math.sv
// Computes heading correction from left/right IR wall measurements and IR_Dtrm.
// Produces adjusted desired heading used by PID when fusion is enabled.
// Computes heading adjustment from IR sensor readings
module IR_math #(
    parameter logic [11:0] NOM_IR = 12'h900
)(
    input logic lft_opn,
    input logic rght_opn,
    input logic [11:0] lft_IR,
    input logic [11:0] rght_IR,
    input logic signed [8:0] IR_Dtrm,
    input logic en_fusion,
    input logic signed [11:0] dsrd_hdng,
    output logic [11:0] dsrd_hdng_adj
);


  logic signed [12:0] lft13, rght13, nom13;
  assign lft13 = {1'b0, lft_IR};
  assign rght13 = {1'b0, rght_IR};
  assign nom13 = {1'b0, NOM_IR};

  // Collapsed 4-way IR_raw: two 2:1 muxes feeding a single 13-bit subtract.
  //   lft_opn=0, rght_opn=0: lft13 - rght13
  //   lft_opn=0, rght_opn=1: lft13 - nom13
  //   lft_opn=1, rght_opn=0: nom13 - rght13
  //   lft_opn=1, rght_opn=1: 0
  logic signed [12:0] a_op, b_op;
  assign a_op = lft_opn  ? nom13 : lft13;
  assign b_op = rght_opn ? nom13 : rght13;

  logic signed [12:0] IR_raw;
  assign IR_raw = (lft_opn && rght_opn) ? 13'sd0 : (a_op - b_op);

  logic signed [12:0] IR_for_P;
  assign IR_for_P = (!lft_opn && !rght_opn) ? (IR_raw >>> 1) : IR_raw;

  logic signed [12:0] P_part;
  assign P_part = IR_for_P >>> 5;

  logic signed [12:0] D_part;
  assign D_part = {{2{IR_Dtrm[8]}}, IR_Dtrm, 2'b00};

  logic signed [12:0] corr13;
  assign corr13 = (P_part + D_part) >>> 1;

  // Heading is a wrapping 12-bit angle; the previous [11:0] truncation of a
  // 13-bit sign-extended sum is equivalent to a plain 12-bit modular add.
  // Using a 12-bit adder instead of 13-bit saves cells with no behavior change.
  logic [11:0] dsrd_hdng_adj_pre;
  assign dsrd_hdng_adj_pre = en_fusion ? (dsrd_hdng + corr13[11:0]) : dsrd_hdng;

  // Hold-fix helper:
  // Preserve a tiny two-inverter buffer chain on this output path so synthesis
  // keeps non-zero min delay on very short IR_math->consumer routes.
  (* keep = "true", syn_keep = 1 *) logic [11:0] dsrd_hdng_adj_buf1;
  (* keep = "true", syn_keep = 1 *) logic [11:0] dsrd_hdng_adj_buf2;
  assign dsrd_hdng_adj_buf1 = ~dsrd_hdng_adj_pre;
  assign dsrd_hdng_adj_buf2 = ~dsrd_hdng_adj_buf1;
  assign dsrd_hdng_adj = dsrd_hdng_adj_buf2;

endmodule
