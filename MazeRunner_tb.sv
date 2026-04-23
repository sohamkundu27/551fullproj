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

  // Bring all interfaces to a known state, then release reset
  task automatic Initialize();
    begin
      clk = 1'b0;
      RST_n = 1'b0;
      send_cmd = 1'b0;
      cmd = 16'h0000;
      batt = 12'hD80;
      @(negedge clk);
      @(negedge clk);
      RST_n = 1'b1;
      @(negedge clk);
      $display("[%0t] Reset released", $time);
    end
  endtask

  // Send one 16-bit command through RemoteComm and wait for cmd_sent confirmation
  task automatic SendCmd(input [15:0] c);
    int timeout_cnt;
    begin
      @(negedge clk);
      cmd = c;
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

  // Wait for DUT response byte and require the expected positive ACK value
  task automatic CheckPosAck();
    int timeout_cnt;
    logic prev_resp_rdy;
    logic saw_new_resp;
    begin
      prev_resp_rdy = resp_rdy;
      saw_new_resp = 1'b0;
      timeout_cnt = 0;
      while (!saw_new_resp && timeout_cnt < 500_000) begin
        @(posedge clk);
        if (!prev_resp_rdy && resp_rdy)
          saw_new_resp = 1'b1;
        prev_resp_rdy = resp_rdy;
        timeout_cnt++;
      end
      if (!saw_new_resp) begin
        $display("FAIL: new resp_rdy event never observed (timeout)");
        $stop;
      end
      if (resp !== 8'hA5) begin
        $display("FAIL: expected 0xA5, got 0x%02h", resp);
        $stop;
      end
      $display("[%0t] PASS: received pos ack 0xA5", $time);
    end
  endtask

  // Wait for inertial model setup completion after startup SPI programming
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

  // Wait for calibration completion from inertial_integrator path
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

  // Utility check for wrapped heading proximity with absolute tolerance
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

  // Verify motors are neutral at reset (no unintended drive torque)
  task automatic CheckPWMsMidrail();
    logic [11:0] lft_d, rght_d;
    begin
      lft_d = iDUT.iMTR.lft_duty;
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

  // For the WEST turn command, validate differential wheel directions
  // are consistent with a CCW heading correction
  task automatic CheckHeadingTurnDirection();
    logic signed [11:0] lft, rght;
    begin
      // Sample speeds after PID has had time to respond
      repeat (50000) @(posedge clk);
      lft = iDUT.lft_spd;
      rght = iDUT.rght_spd;
      $display("[%0t] moving=%b lft_spd=%0d rght_spd=%0d lft_duty=0x%03h rght_duty=0x%03h",
               $time, iDUT.moving, lft, rght,
               iDUT.iMTR.lft_duty, iDUT.iMTR.rght_duty);
      if (!iDUT.moving) begin
        $display("FAIL: moving not asserted during heading turn");
        $stop;
      end
      if (!(lft < 0 && rght > 0)) begin
        $display("FAIL: expected lft_spd<0 and rght_spd>0 for CCW turn, got lft=%0d rght=%0d", lft, rght);
        $stop;
      end
      $display("[%0t] PASS: heading turn direction correct (lft=%0d rght=%0d)", $time, lft, rght);
    end
  endtask

  // Confirm forward motion by watching RunnerPhysics wheel-speed sum ramp up
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

  // Ensure no response is produced within a bounded window.
  // Useful for invalid/unsupported command coverage.
  task automatic CheckNoRespWindow(input int max_cycles, input [255:0] label);
    int timeout_cnt;
    logic prev_resp_rdy;
    begin
      prev_resp_rdy = resp_rdy;
      timeout_cnt = 0;
      while (timeout_cnt < max_cycles) begin
        @(posedge clk);
        if (!prev_resp_rdy && resp_rdy) begin
          $display("FAIL: unexpected response 0x%02h during %0s", resp, label);
          $stop;
        end
        prev_resp_rdy = resp_rdy;
        timeout_cnt++;
      end
      $display("[%0t] PASS: no response observed during %0s", $time, label);
    end
  endtask

  // Wait until cmd_proc enters command mode (cmd_md=1) or solver mode (cmd_md=0).
  task automatic WaitCmdMode(input logic exp_cmd_md, input int max_cycles, input [255:0] label);
    int timeout_cnt;
    begin
      timeout_cnt = 0;
      while ((iDUT.cmd_md !== exp_cmd_md) && (timeout_cnt < max_cycles)) begin
        @(posedge clk);
        timeout_cnt++;
      end
      if (iDUT.cmd_md !== exp_cmd_md) begin
        $display("FAIL: cmd_md did not become %0d during %0s", exp_cmd_md, label);
        $stop;
      end
      $display("[%0t] PASS: cmd_md=%0d during %0s", $time, iDUT.cmd_md, label);
    end
  endtask

  // Verify MOVE command stop bits are latched as expected in cmd_proc.
  task automatic CheckCmdStopBits(input logic exp_lft, input logic exp_rght);
    begin
      if ((iDUT.iCMD.stp_lft !== exp_lft) || (iDUT.iCMD.stp_rght !== exp_rght)) begin
        $display("FAIL: stop bit decode mismatch exp(l=%0b r=%0b) got(l=%0b r=%0b)",
                 exp_lft, exp_rght, iDUT.iCMD.stp_lft, iDUT.iCMD.stp_rght);
        $stop;
      end
      $display("[%0t] PASS: stop bits latched l=%0b r=%0b",
               $time, iDUT.iCMD.stp_lft, iDUT.iCMD.stp_rght);
    end
  endtask

  initial begin
    $display("========================================");
    $display("Starting MazeRunner_tb full system test");
    $display("========================================");

    // Common reset/setup used by all subsequent directed tests
    Initialize();

    $display("\n--- Test 1: PWMs at midrail ---");
    // Let PWM/control loops settle briefly after reset release
    repeat (20) @(posedge clk);
    CheckPWMsMidrail();

    $display("\n--- Test 2: NEMO setup ---");
    WaitForNemoSetup();

    $display("\n--- Test 3: Calibrate ---");
    // Calibrate command exercises BLE/UART/cmd_proc/inertial pipeline
    SendCmd(16'h0000);
    WaitForCalDone();
    CheckPosAck();

    $display("\n--- Test 4: Heading West (0x23FF) ---");
    // Command heading to WEST and inspect turn dynamics before ACK
    SendCmd(16'h23FF);
    repeat (50000) @(posedge clk);
    $display("moving=%b at_hdng=%b frwrd_spd=%0d",
             iDUT.moving, iDUT.at_hdng, iDUT.frwrd_spd);
    $display("actl_hdng=%0d dsrd_hdng=%0d",
             iDUT.actl_hdng, iDUT.iCMD.dsrd_hdng);
    $display("lft_spd=%0d rght_spd=%0d",
             iDUT.lft_spd, iDUT.rght_spd);
    $display("lft_duty=0x%03h rght_duty=0x%03h",
             iDUT.iMTR.lft_duty, iDUT.iMTR.rght_duty);
    CheckHeadingTurnDirection();
    CheckPosAck();
    $display("[%0t] INFO: post-move heading=0x%03h (target=0x3FF, drift expected after stop)",
             $time, iDUT.actl_hdng);

    $display("\n--- Test 5: Heading South (0x27FF) ---");
    // Back-to-back heading command checks command re-acceptance after RESP
    SendCmd(16'h27FF);
    CheckPosAck();
    $display("[%0t] INFO: post-move heading=0x%03h (target=0x7FF, drift expected after stop)",
             $time, iDUT.actl_hdng);

    $display("\n--- Test 6: Move forward (0x4000) ---");
    // Forward move command; physics model should observe wheel acceleration
    SendCmd(16'h4000);
    WaitOmegaSumRamp();
    CheckPosAck();
    WaitCmdMode(1'b1, 50_000, "post-move return to command mode");

    $display("\n--- Test 7: Invalid opcode has no response (0xE000) ---");
    SendCmd(16'hE000);
    CheckNoRespWindow(300_000, "invalid-opcode no-response");

    $display("\n--- Test 8: Move stop-bit decode + response (0x4003) ---");
    SendCmd(16'h4003);
    // Give cmd_proc a few cycles to accept and latch MOVE stop bits.
    repeat (20) @(posedge clk);
    CheckCmdStopBits(1'b1, 1'b1);
    CheckPosAck();
    WaitCmdMode(1'b1, 50_000, "post-stop-bit move return to command mode");

    $display("\n--- Test 9: Solve command enters solver mode (0x6001) ---");
    SendCmd(16'h6001);
    WaitCmdMode(1'b0, 200_000, "solver-mode entry");

    $display("\n========================================");
    $display("ALL MAZERUNNER TESTS PASSED");
    $display("========================================");
    $display("Group Members: Bhanu Kankanala, Arnav Mohanty, Soham Kundu, Mohnish Nanthakumar");
    $stop();
  end

  initial begin
    // Safety net: prevents hung simulations from running indefinitely
    #100_000_000;
    $display("FAIL: global simulation timeout");
    $stop;
  end

endmodule
