`timescale 1ns/1ps

// MtrDrv_tb.sv
// Unit testbench for motor driver scaling/saturation and PWM duty generation.
// Exercises battery scaling with signed speed inputs.
module MtrDrv_tb();

  logic clk;
  logic rst_n;

  logic [11:0] vbatt;
  logic signed [11:0] lft_spd;
  logic signed [11:0] rght_spd;

  logic lftPWM1;
  logic lftPWM2;
  logic rghtPWM1;
  logic rghtPWM2;

  localparam int TOL = 3;

  MtrDrv iDUT(
    .clk(clk),
    .rst_n(rst_n),
    .vbatt(vbatt),
    .lft_spd(lft_spd),
    .rght_spd(rght_spd),
    .lftPWM1(lftPWM1),
    .lftPWM2(lftPWM2),
    .rghtPWM1(rghtPWM1),
    .rghtPWM2(rghtPWM2)
  );

  initial begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  initial begin
    $display("Running MtrDrv_tb for akmohanty");

    rst_n = 1'b0;
    vbatt = 12'hDB0;
    lft_spd = 12'h000;
    rght_spd = 12'h000;

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (4) @(posedge clk);

    check_case("zero speed at vbatt DB",
               12'hDB0, 12'h000, 12'h000,
               12'h800, 12'h800);

    check_case("positive speed at unity scaling",
               12'hDB0, 12'h3FF, 12'h3FF,
               12'hBFD, 12'h403);

    check_case("positive speed at low battery",
               12'hD00, 12'h3FF, 12'h3FF,
               12'hC32, 12'h3CE);

    check_case("negative speed at fresh battery",
               12'hFF0, 12'hC00, 12'hC00,
               12'h492, 12'hB6E);

    check_case("left positive right negative",
               12'hDB0, 12'h3FF, 12'hC00,
               12'hBFD, 12'hBFE);

    check_case("left negative right positive",
               12'hDB0, 12'hC00, 12'h3FF,
               12'h402, 12'h403);

    $display("All MtrDrv tests passed");
    $stop;
  end

  task automatic check_case(
    input string name,
    input logic [11:0] vbatt_val,
    input logic signed [11:0] lft_val,
    input logic signed [11:0] rght_val,
    input logic [11:0] exp_lft_duty,
    input logic [11:0] exp_rght_duty
  );
    begin
      vbatt = vbatt_val;
      lft_spd = lft_val;
      rght_spd = rght_val;

      // wait for the ROM output to update
      repeat (4) @(posedge clk);

      check_duty({name, " left duty"}, iDUT.lft_duty, exp_lft_duty);
      check_duty({name, " right duty"}, iDUT.rght_duty, exp_rght_duty);

      check_no_overlap(name);

      $display("Passed: %s", name);
    end
  endtask

  task automatic check_duty(
    input string label,
    input logic [11:0] got,
    input logic [11:0] exp
  );
    int got_int;
    int exp_int;

    begin
      got_int = got;
      exp_int = exp;

      if ((got_int < exp_int - TOL) || (got_int > exp_int + TOL)) begin
        $display("ERROR: %s expected about %h but got %h", label, exp, got);
        $stop;
      end
    end
  endtask

  task automatic check_no_overlap(
    input string name
  );
    int i;

    begin
      for (i = 0; i < 100; i++) begin
        @(posedge clk);

        if (lftPWM1 && lftPWM2) begin
          $display("ERROR: left PWM signals overlap during %s", name);
          $stop;
        end

        if (rghtPWM1 && rghtPWM2) begin
          $display("ERROR: right PWM signals overlap during %s", name);
          $stop;
        end
      end
    end
  endtask

endmodule
