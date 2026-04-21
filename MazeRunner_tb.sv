// MazeRunner_tb.sv
// Full-system testbench for MazeRunner project.
// Uses RemoteComm to send 16-bit commands and RunnerPhysics to model the
// physical world (wheels, maze walls, gyro).
//
// Test sequence:
//   1. Reset + PWM midrail check
//   2. Wait for NEMO setup (gyro initialized)
//   3. Calibrate command (0x0000) -> cal_done + pos ack (0xA5)
//   4. Change heading to WEST (0x23FF) -> wheels respond, heading converges, pos ack
//   5. Change heading to SOUTH (0x27FF) -> facing open direction
//   6. Move forward (0x4000) -> omega_sum ramps up
//
// Group Members: Bhanu Kankanala, Arnav Mohanty, Soham Kundu, Mohnish Nanthakumar

`timescale 1ns/1ps

module MazeRunner_tb();

  reg clk, RST_n;
  reg send_cmd;
  reg [15:0] cmd;
  reg [11:0] batt;

  logic cmd_sent;
  logic resp_rdy;
  logic [7:0] resp;

  wire TX_RX, RX_TX;
  wire INRT_SS_n, INRT_SCLK, INRT_MOSI, INRT_MISO, INRT_INT;
  wire lftPWM1, lftPWM2, rghtPWM1, rghtPWM2;
  wire A2D_SS_n, A2D_SCLK, A2D_MOSI, A2D_MISO;
  wire IR_lft_en, IR_cntr_en, IR_rght_en;
  wire piezo, piezo_n;
  wire hall_n;

  MazeRunner iDUT(
    .clk(clk), .RST_n(RST_n),
    .INRT_SS_n(INRT_SS_n), .INRT_SCLK(INRT_SCLK),
    .INRT_MOSI(INRT_MOSI), .INRT_MISO(INRT_MISO), .INRT_INT(INRT_INT),
    .A2D_SS_n(A2D_SS_n), .A2D_SCLK(A2D_SCLK),
    .A2D_MOSI(A2D_MOSI), .A2D_MISO(A2D_MISO),
    .lftPWM1(lftPWM1), .lftPWM2(lftPWM2),
    .rghtPWM1(rghtPWM1), .rghtPWM2(rghtPWM2),
    .RX(RX_TX), .TX(TX_RX),
    .hall_n(hall_n), .piezo(piezo), .piezo_n(piezo_n),
    .IR_lft_en(IR_lft_en), .IR_cntr_en(IR_cntr_en), .IR_rght_en(IR_rght_en),
    .LED()
  );

  RemoteComm iCMD(
    .clk(clk), .rst_n(RST_n),
    .RX(TX_RX), .TX(RX_TX),
    .cmd(cmd), .snd_cmd(send_cmd),
    .cmd_snt(cmd_sent),
    .resp_rdy(resp_rdy), .resp(resp)
  );

  RunnerPhysics iPHYS(
    .clk(clk), .RST_n(RST_n),
    .SS_n(INRT_SS_n), .SCLK(INRT_SCLK),
    .MISO(INRT_MISO), .MOSI(INRT_MOSI), .INT(INRT_INT),
    .lftPWM1(lftPWM1), .lftPWM2(lftPWM2),
    .rghtPWM1(rghtPWM1), .rghtPWM2(rghtPWM2),
    .IR_lft_en(IR_lft_en), .IR_cntr_en(IR_cntr_en), .IR_rght_en(IR_rght_en),
    .A2D_SS_n(A2D_SS_n), .A2D_SCLK(A2D_SCLK),
    .A2D_MOSI(A2D_MOSI), .A2D_MISO(A2D_MISO),
    .hall_n(hall_n), .batt(batt)
  );

  always #10 clk = ~clk;

  task automatic Initialize();
    begin
      clk      = 1'b0;
      RST_n    = 1'b0;
      send_cmd = 1'b0;
      cmd      = 16'h0000;
      batt     = 12'hD80;
      @(negedge clk);
      @(negedge clk);
      RST_n = 1'b1;
      @(negedge clk);
      $display("[%0t] Reset released", $time);
    end
  endtask

  task automatic SendCmd(input [15:0] c);
    int timeout_cnt;
    begin
      @(negedge clk);
      cmd      = c;
      send_cmd = 1'b1;
      @(negedge clk);
      send_cmd = 1'b0;
      timeout_cnt = 0;
      while (!cmd_sent && timeout_cnt < 200_000) begin
        @(posedge clk);
        timeout_cnt++;
      end
      if (!cmd_sent) begin
        $display("FAIL: cmd_sent never asserted for cmd 0x%04h", c);
        $stop;
      end
      $display("[%0t] Command 0x%04h sent", $time, c);
    end
  endtask

  task automatic CheckPosAck();
    int timeout_cnt;
    begin
      timeout_cnt = 0;
      while (!resp_rdy && timeout_cnt < 500_000) begin
        @(posedge clk);
        timeout_cnt++;
      end
      if (!resp_rdy) begin
        $display("FAIL: resp_rdy never asserted (timeout)");
        $stop;
      end
      if (resp !== 8'hA5) begin
        $display("FAIL: expected 0xA5, got 0x%02h", resp);
        $stop;
      end
      $display("[%0t] PASS: received pos ack 0xA5", $time);
    end
  endtask

  task automatic WaitForNemoSetup();
    int timeout_cnt;
    begin
      timeout_cnt = 0;
      while (!iPHYS.iNEMO.NEMO_setup && timeout_cnt < 2_000_000) begin
        @(posedge clk);
        timeout_cnt++;
      end
      if (!iPHYS.iNEMO.NEMO_setup) begin
        $display("FAIL: NEMO_setup never asserted");
        $stop;
      end
      $display("[%0t] PASS: NEMO_setup complete", $time);
    end
  endtask

  task automatic WaitForCalDone();
    int timeout_cnt;
    begin
      timeout_cnt = 0;
      while (!iDUT.cal_done && timeout_cnt < 2_000_000) begin
        @(posedge clk);
        timeout_cnt++;
      end
      if (!iDUT.cal_done) begin
        $display("FAIL: cal_done never asserted");
        $stop;
      end
      $display("[%0t] PASS: cal_done asserted", $time);
    end
  endtask

  task automatic CheckHeadingNear(input logic signed [11:0] target, input int tol);
    logic signed [11:0] actual;
    int diff;
    begin
      actual = iDUT.actl_hdng;
      diff = actual - target;
      if (diff < 0) diff = -diff;
      if (diff > tol) begin
        $display("FAIL: heading 0x%03h not within %0d of target 0x%03h", actual, tol, target);
        $stop;
      end
      $display("[%0t] PASS: heading 0x%03h near target 0x%03h", $time, actual, target);
    end
  endtask

  task automatic CheckPWMsMidrail();
    logic [11:0] lft_d, rght_d;
    begin
      lft_d  = iDUT.iMTR.lft_duty;
      rght_d = iDUT.iMTR.rght_duty;
      if ((lft_d < 12'h7E0) || (lft_d > 12'h820)) begin
        $display("FAIL: left duty not midrail: 0x%03h", lft_d);
        $stop;
      end
      if ((rght_d < 12'h7E0) || (rght_d > 12'h820)) begin
        $display("FAIL: right duty not midrail: 0x%03h", rght_d);
        $stop;
      end
      $display("[%0t] PASS: PWMs at midrail (lft=0x%03h rght=0x%03h)", $time, lft_d, rght_d);
    end
  endtask

  task automatic CheckRightGreaterThanLeft();
    int i, seen_diff;
    logic [11:0] lft_d, rght_d;
    begin
      seen_diff = 0;
      for (i = 0; i < 100; i++) begin
        @(posedge clk);
        lft_d  = iDUT.iMTR.lft_duty;
        rght_d = iDUT.iMTR.rght_duty;
        if (rght_d > lft_d + 12'h020) seen_diff++;
      end
      if (seen_diff == 0) begin
        $display("FAIL: right duty never > left during heading turn");
        $stop;
      end
      $display("[%0t] PASS: right duty exceeded left (%0d samples)", $time, seen_diff);
    end
  endtask

  task automatic WaitOmegaSumRamp();
    int timeout_cnt;
    begin
      timeout_cnt = 0;
      while ((iPHYS.omega_sum < $signed(17'd5000)) && (timeout_cnt < 1_000_000)) begin
        @(posedge clk);
        timeout_cnt++;
      end
      if (iPHYS.omega_sum < $signed(17'd5000)) begin
        $display("FAIL: omega_sum never ramped (got %0d)", $signed(iPHYS.omega_sum));
        $stop;
      end
      $display("[%0t] PASS: omega_sum ramped to %0d", $time, $signed(iPHYS.omega_sum));
    end
  endtask

  initial begin
    $display("========================================");
    $display("Starting MazeRunner_tb full system test");
    $display("========================================");

    Initialize();

    $display("\n--- Test 1: PWMs at midrail ---");
    repeat (20) @(posedge clk);
    CheckPWMsMidrail();

    $display("\n--- Test 2: NEMO setup ---");
    WaitForNemoSetup();

    $display("\n--- Test 3: Calibrate ---");
    SendCmd(16'h0000);
    WaitForCalDone();
    CheckPosAck();

    $display("\n--- Test 4: Heading West (0x23FF) ---");
    SendCmd(16'h23FF);
    repeat (100) @(posedge clk);
    CheckRightGreaterThanLeft();
    CheckPosAck();
    CheckHeadingNear(12'h3FF, 64);

    $display("\n--- Test 5: Heading South (0x27FF) ---");
    SendCmd(16'h27FF);
    CheckPosAck();
    CheckHeadingNear(12'h7FF, 64);

    $display("\n--- Test 6: Move forward (0x4000) ---");
    SendCmd(16'h4000);
    WaitOmegaSumRamp();

    $display("\n========================================");
    $display("ALL MAZERUNNER TESTS PASSED");
    $display("========================================");
    $display("Group Members: Bhanu Kankanala, Arnav Mohanty, Soham Kundu, Mohnish Nanthakumar");
    $stop();
  end

  initial begin
    #100_000_000;
    $display("FAIL: global simulation timeout");
    $stop;
  end

endmodule
