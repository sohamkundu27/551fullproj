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
  assign nom13  = {1'b0, NOM_IR};

  // IR difference based on which sensors are open
  logic signed [12:0] IR_raw;
  always_comb begin
    if (!lft_opn && !rght_opn)       IR_raw = lft13 - rght13;
    else if (!lft_opn && rght_opn)   IR_raw = lft13 - nom13;
    else if (lft_opn && !rght_opn)   IR_raw = nom13 - rght13;
    else                             IR_raw = 13'sd0;
  end

  logic signed [12:0] IR_for_P;
  assign IR_for_P = (!lft_opn && !rght_opn) ? (IR_raw >>> 1) : IR_raw;

  logic signed [12:0] P_part;
  assign P_part = IR_for_P >>> 5;

  logic signed [12:0] D_part;
  assign D_part = {{2{IR_Dtrm[8]}}, IR_Dtrm, 2'b00};

  logic signed [12:0] corr13;
  assign corr13 = (P_part + D_part) >>> 1;

  logic signed [12:0] hdng_ext;
  assign hdng_ext = {dsrd_hdng[11], dsrd_hdng};

  // Apply correction to desired heading when fusion is enabled
  logic signed [12:0] adj13;
  assign adj13 = en_fusion ? (hdng_ext + corr13) : hdng_ext;

  assign dsrd_hdng_adj = adj13[11:0];

endmodule
