///////////////////////////////////////////////////////////////////////////
// Closed-loop heading controller used by MazeRunner.                   //
// Computes P/I/D correction from heading error, then outputs signed   //
// left/right wheel speed commands around the requested forward speed.//
///////////////////////////////////////////////////////////////////////
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

  // Controller gains for proportional and derivative terms
  localparam signed [3:0] P_COEFF = 4'h3;
  localparam signed [4:0] D_COEFF = 5'h0E;

  // Input pipeline plus 10-bit error clamp
  logic signed [11:0] dsrd_hdng_q, actl_hdng_q;
  logic signed [11:0] error;
  logic signed [9:0] err_sat;

  // Pipeline desired/actual heading before PID combinational math
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      dsrd_hdng_q <= 12'h000;
      actl_hdng_q <= 12'h000;
    end else begin
      dsrd_hdng_q <= dsrd_hdng;
      actl_hdng_q <= actl_hdng;
    end

  // Signed heading error (actual - desired)
  assign error = actl_hdng_q - dsrd_hdng_q;

  // Clamp the 12-bit error into a 10-bit range before PID math
  assign err_sat = (!error[11] && |error[10:9])  ? 10'sb0111111111 :
                   ( error[11] && ~&error[10:9]) ? 10'sb1000000000 :
                   error[9:0];

  logic signed [9:0] err_sat_q;

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      err_sat_q <= 10'h000;
    else if (hdng_vld)
      err_sat_q <= err_sat;

  // "At heading" when heading error is within a small deadband
  assign at_hdng = (err_sat < 10'sd30) && (err_sat > -10'sd30);

  // Proportional term
  // P_COEFF = 3 => err_sat*3 = (err_sat<<1) + err_sat
  logic signed [11:0] P_term;

  assign P_term = {{1{err_sat[9]}}, err_sat, 1'b0}
                + {{2{err_sat[9]}}, err_sat};

  // Integral term
  logic signed [15:0] integrator, nxt_integrator;
  logic signed [15:0] err_ext, accum;
  logic ov;
  logic signed [11:0] I_term;


  // Sign-extend saturated error to integrator width
  assign err_ext = {{6{err_sat_q[9]}}, err_sat_q};
  assign accum = err_ext + integrator;

  // Two's-complement overflow detect for accumulator add
  assign ov = (err_ext[15] & integrator[15] & ~accum[15]) |
              (~err_ext[15] & ~integrator[15] & accum[15]);

  // Reset integral state when not moving, otherwise integrate on valid samples
  assign nxt_integrator = (!moving)          ? 16'h0000 :
                          (hdng_vld && !ov)  ? accum    :
                          integrator;

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      integrator <= 16'h0000;
    else
      integrator <= nxt_integrator;

  assign I_term = integrator[15:4];

  // Derivative term
  logic signed [9:0] ff1, ff2;
  logic signed [9:0] diff;
  logic signed [7:0] diff_sat;
  logic signed [11:0] D_term;

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      ff1 <= 10'h000;
      ff2 <= 10'h000;
    end else if (hdng_vld) begin
      // Two-sample pipeline used to approximate derivative.
      ff1 <= err_sat;
      ff2 <= ff1;
    end

  // First-order discrete derivative of heading error
  assign diff = ff1 - ff2;

  // Limit derivative magnitude to keep D-term from dominating
  assign diff_sat = (!diff[9] && |diff[8:7])  ? 8'sh7F :
                    ( diff[9] && ~&diff[8:7]) ? 8'sh80 :
                    diff[7:0];

  // D_COEFF = 14 = 8 + 4 + 2 => (diff_sat<<3) + (diff_sat<<2) + (diff_sat<<1)
  assign D_term = {{1{diff_sat[7]}}, diff_sat, 3'b000}
                + {{2{diff_sat[7]}}, diff_sat, 2'b00}
                + {{3{diff_sat[7]}}, diff_sat, 1'b0};

  // PID sum, divide-by-8, and motor speed outputs
  logic signed [13:0] PID_sum;
  logic signed [11:0] PID_corr_nxt, PID_corr_q;
  logic signed [11:0] frwrd_ext;

  // Total control effort, then scale down by 8 for motor command range
  assign PID_sum = {{2{P_term[11]}}, P_term}
                 + {{2{I_term[11]}}, I_term}
                 + {{2{D_term[11]}}, D_term};
  assign PID_corr_nxt = PID_sum >>> 3;

  // Additional pipeline cut: register correction term before speed outputs
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      PID_corr_q <= 12'h000;
    else
      PID_corr_q <= PID_corr_nxt;

  // Extend unsigned forward speed to signed 12-bit domain
  assign frwrd_ext = {1'b0, frwrd_spd};

  // Differential steering: add/subtract correction around forward speed.
  // Pipeline the motor-speed outputs to break the long IR to MtrDrv to PWM
  // combinational chain (critical-path fix).
  logic signed [11:0] lft_spd_nxt, rght_spd_nxt;
  assign lft_spd_nxt = moving ? (frwrd_ext + PID_corr_q) : 12'h000;
  assign rght_spd_nxt = moving ? (frwrd_ext - PID_corr_q) : 12'h000;

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      lft_spd  <= 12'h000;
      rght_spd <= 12'h000;
    end else begin
      lft_spd  <= lft_spd_nxt;
      rght_spd <= rght_spd_nxt;
    end

endmodule
