// MtrDrv.sv
// Converts signed wheel speed commands into PWM duties for left/right motors.
// Includes battery-voltage scaling, saturation, and duty pipelining before PWM12.
module MtrDrv(
  input  logic              clk,
  input  logic              rst_n,
  input  logic [11:0]       vbatt,
  input  logic signed [11:0] lft_spd,
  input  logic signed [11:0] rght_spd,
  output logic              lftPWM1,
  output logic              lftPWM2,
  output logic              rghtPWM1,
  output logic              rghtPWM2
);

  // DutyScaleROM output is 13-bit (MSB is always 0; we only use the low 12).
  logic [12:0] scale_factor;

  logic signed [12:0] lft_spd_ext;
  logic signed [12:0] rght_spd_ext;
  logic signed [12:0] scale_ext;

  // 13-bit signed spd x 13-bit signed scale_ext (MSB always 0) -> 25-bit signed.
  logic signed [24:0] lft_prod;
  logic signed [24:0] rght_prod;

  logic signed [11:0] lft_scaled;
  logic signed [11:0] rght_scaled;

  logic signed [12:0] lft_scaled_ext;
  logic signed [12:0] rght_scaled_ext;

  // Pre-flop duty values (combinational) and registered outputs to PWM12.
  logic [11:0] lft_duty_nxt;
  logic [11:0] rght_duty_nxt;
  logic [11:0] lft_duty;
  logic [11:0] rght_duty;

  DutyScaleROM iDUTY_SCALE_ROM(
    .clk(clk),
    .batt_level(vbatt[9:4]),
    .scale(scale_factor)
  );

  assign lft_spd_ext  = {lft_spd[11],  lft_spd};
  assign rght_spd_ext = {rght_spd[11], rght_spd};

  // scale_factor MSB is always 0 per the ROM; take only the low 12 bits and
  // re-extend to 13-bit signed. This lets DC infer the narrower multiplier
  // regardless of how the ROM port is declared.
  assign scale_ext = {1'b0, scale_factor[11:0]};

  assign lft_prod  = lft_spd_ext  * scale_ext;
  assign rght_prod = rght_spd_ext * scale_ext;

  assign lft_scaled  = sat12(lft_prod);
  assign rght_scaled = sat12(rght_prod);

  assign lft_scaled_ext  = {lft_scaled[11],  lft_scaled};
  assign rght_scaled_ext = {rght_scaled[11], rght_scaled};

  // left motor uses normal signed-to-unsigned conversion
  assign lft_duty_nxt  = duty_sat(13'sd2048 + lft_scaled_ext);

  // right motor is physically flipped, so its command is inverted
  assign rght_duty_nxt = duty_sat(13'sd2048 - rght_scaled_ext);

  // Pipeline register between the duty arithmetic and PWM12 inputs.
  // This is the stage that breaks the long IR->PID->MtrDrv->PWM path.
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      lft_duty  <= 12'h800;     // midscale = motor off
      rght_duty <= 12'h800;
    end else begin
      lft_duty  <= lft_duty_nxt;
      rght_duty <= rght_duty_nxt;
    end

  PWM12 iLFT_PWM(
    .clk(clk),
    .rst_n(rst_n),
    .duty(lft_duty),
    .PWM1(lftPWM1),
    .PWM2(lftPWM2)
  );

  PWM12 iRGHT_PWM(
    .clk(clk),
    .rst_n(rst_n),
    .duty(rght_duty),
    .PWM1(rghtPWM1),
    .PWM2(rghtPWM2)
  );

  // Saturate the >>>11 arithmetic-shift of the product to 12-bit signed.
  // Only bits [24:11] of the 25-bit product matter for the final range check.
  function automatic logic signed [11:0] sat12(input logic signed [24:0] prod);
    logic signed [13:0] shifted;
    begin
      shifted = prod[24:11];                   // sign-preserving >>> 11
      if (shifted > 14'sd2047)
        sat12 = 12'sh7FF;
      else if (shifted < -14'sd2048)
        sat12 = 12'sh800;
      else
        sat12 = shifted[11:0];
    end
  endfunction

  function automatic logic [11:0] duty_sat(input logic signed [12:0] duty_in);
    begin
      if (duty_in < 13'sd0)
        duty_sat = 12'h000;
      else if (duty_in > 13'sd4095)
        duty_sat = 12'hFFF;
      else
        duty_sat = duty_in[11:0];
    end
  endfunction

endmodule
