module PID (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        moving,
    input  logic [11:0] dsrd_hdng,
    input  logic [11:0] actl_hdng,
    input  logic        hdng_vld,
    input  logic [10:0] frwrd_spd,
    output logic        at_hdng,
    output logic signed [11:0] lft_spd,
    output logic signed [11:0] rght_spd
);

  // Controller gains for proportional and derivative terms.
  localparam signed [3:0] P_COEFF = 4'h3;
  localparam signed [4:0] D_COEFF = 5'h0E;

  ///////////////////////////////////////////
  // Error computation and 10-bit saturate //
  /////////////////////////////////////////
  logic signed [11:0] error;
  logic signed [9:0] err_sat;

  // Signed heading error (actual - desired).
  assign error = actl_hdng - dsrd_hdng;

  // Clamp the 12-bit error into a 10-bit range before PID math.
  assign err_sat = (!error[11] && |error[10:9])  ? 10'sb0111111111 :
                   ( error[11] && ~&error[10:9]) ? 10'sb1000000000 :
                   error[9:0];

  // "At heading" when heading error is within a small deadband.
  assign at_hdng = (err_sat < 10'sd30) && (err_sat > -10'sd30);

  /////////////
  // P term //
  ///////////
  logic signed [13:0] P_term;

  assign P_term = err_sat * P_COEFF;

  /////////////
  // I term //
  ///////////
  logic signed [15:0] integrator, nxt_integrator;
  logic signed [15:0] err_ext, accum;
  logic ov;
  logic signed [11:0] I_term;

  // Sign-extend saturated error to integrator width.
  assign err_ext = {{6{err_sat[9]}}, err_sat};
  assign accum = err_ext + integrator;

  // Two's-complement overflow detect for accumulator add.
  assign ov = (err_ext[15] & integrator[15] & ~accum[15]) |
              (~err_ext[15] & ~integrator[15] & accum[15]);

  // Reset integral state when not moving, otherwise integrate on valid samples.
  assign nxt_integrator = (!moving)          ? 16'h0000 :
                          (hdng_vld && !ov)  ? accum    :
                          integrator;

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      integrator <= 16'h0000;
    else
      integrator <= nxt_integrator;

  assign I_term = integrator[15:4];

  /////////////
  // D term //
  ///////////
  logic signed [9:0] ff1, ff2;
  logic signed [9:0] diff;
  logic signed [7:0] diff_sat;
  logic signed [12:0] D_term;

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      ff1 <= 10'h000;
      ff2 <= 10'h000;
    end else if (hdng_vld) begin
      // Two-sample pipeline used to approximate derivative.
      ff1 <= err_sat;
      ff2 <= ff1;
    end

  // First-order discrete derivative of heading error.
  assign diff = ff1 - ff2;

  // Limit derivative magnitude to keep D-term from dominating.
  assign diff_sat = (!diff[9] && |diff[8:7])  ? 8'sh7F :
                    ( diff[9] && ~&diff[8:7]) ? 8'sh80 :
                    diff[7:0];

  assign D_term = diff_sat * D_COEFF;

  ///////////////////////////////////////////////
  // PID sum, divide by 8, motor speed output //
  /////////////////////////////////////////////
  logic signed [14:0] PID_sum;
  logic signed [11:0] PID_div8;
  logic signed [11:0] frwrd_ext;

  // Total control effort, then scale down by 8 for motor command range.
  assign PID_sum = P_term + I_term + D_term;
  assign PID_div8 = PID_sum[14:3];

  // Extend unsigned forward speed to signed 12-bit domain.
  assign frwrd_ext = {1'b0, frwrd_spd};

  // Differential steering: add/subtract correction around forward speed.
  assign lft_spd  = moving ? (frwrd_ext + PID_div8) : 12'h000;
  assign rght_spd = moving ? (frwrd_ext - PID_div8) : 12'h000;

endmodule
