`timescale 1ns/1ps

// CommTB.sv
// UART communication integration testbench.
// Verifies RemoteComm <-> UART_wrapper command/response path with timeouts.
module CommTB;

  // 50 MHz clock and conservative UART transaction timeout.
  localparam int unsigned CLK_HALF_PERIOD_NS = 10;
  localparam int unsigned UART_TIMEOUT_CYCLES = 200000;
  localparam int unsigned RANDOM_TESTS = 3;

  logic        clk;
  logic        rst_n;

  logic        rem_snd_cmd;
  logic [15:0] rem_cmd;
  logic        rem_cmd_snt;
  logic        rem_tx;
  logic        rem_rx;
  logic [7:0]  rem_resp;
  logic        rem_resp_rdy;

  logic        wrp_tx;
  logic        wrp_rx;
  logic        wrp_cmd_rdy;
  logic [15:0] wrp_cmd;
  logic        wrp_clr_cmd_rdy;
  logic        wrp_trmt;
  logic [7:0]  wrp_resp;
  logic        wrp_tx_done;

  // Cross-couple serial links to emulate BLE <-> wrapper wiring.
  assign wrp_rx = rem_tx;
  assign rem_rx = wrp_tx;

  RemoteComm iRemoteComm (
      .clk     (clk),
      .rst_n   (rst_n),
      .snd_cmd (rem_snd_cmd),
      .cmd     (rem_cmd),
      .cmd_snt (rem_cmd_snt),
      .TX      (rem_tx),
      .RX      (rem_rx),
      .resp    (rem_resp),
      .resp_rdy(rem_resp_rdy)
  );

  UART_wrapper iUART_wrapper (
      .clk        (clk),
      .rst_n      (rst_n),
      .RX         (wrp_rx),
      .TX         (wrp_tx),
      .cmd_rdy    (wrp_cmd_rdy),
      .cmd        (wrp_cmd),
      .clr_cmd_rdy(wrp_clr_cmd_rdy),
      .trmt       (wrp_trmt),
      .resp       (wrp_resp),
      .tx_done    (wrp_tx_done)
  );

  // Free-running 50 MHz clock.
  initial begin
    clk = 1'b0;
    forever #(CLK_HALF_PERIOD_NS) clk = ~clk;
  end

  // One-cycle send pulse carrying a 16-bit command.
  task automatic pulse_snd_cmd(input logic [15:0] cmd_in);
    begin
      rem_cmd     = cmd_in;
      rem_snd_cmd = 1'b1;
      @(posedge clk);
      rem_snd_cmd = 1'b0;
    end
  endtask

  // Wait for wrapper command-ready with timeout protection.
  task automatic wait_cmd_rdy_with_timeout;
    int unsigned cycles;
    begin
      cycles = 0;
      while (!wrp_cmd_rdy && (cycles < UART_TIMEOUT_CYCLES)) begin
        @(posedge clk);
        cycles++;
      end
      if (!wrp_cmd_rdy) begin
        $display("ERROR: Timeout waiting for wrp_cmd_rdy after %0d cycles", UART_TIMEOUT_CYCLES);
        $stop;
      end
    end
  endtask

  // Wait for RemoteComm response-ready with timeout protection.
  task automatic wait_resp_rdy_with_timeout;
    int unsigned cycles;
    begin
      cycles = 0;
      while (!rem_resp_rdy && (cycles < UART_TIMEOUT_CYCLES)) begin
        @(posedge clk);
        cycles++;
      end
      if (!rem_resp_rdy) begin
        $display("ERROR: Timeout waiting for rem_resp_rdy after %0d cycles", UART_TIMEOUT_CYCLES);
        $stop;
      end
    end
  endtask

  // Consumer side handshake to clear wrapper cmd_rdy.
  task automatic clear_cmd_rdy;
    begin
      wrp_clr_cmd_rdy = 1'b1;
      @(posedge clk);
      wrp_clr_cmd_rdy = 1'b0;
    end
  endtask

  // Wrapper transmits one-byte response back to RemoteComm.
  task automatic send_ack(input logic [7:0] ack_in);
    begin
      wrp_resp = ack_in;
      wrp_trmt = 1'b1;
      @(posedge clk);
      wrp_trmt = 1'b0;
    end
  endtask

  // Full round-trip check: command to wrapper, ack back to remote.
  task automatic run_case(
      input logic [15:0] cmd_in,
      input logic [7:0]  ack_in
  );
    begin
      pulse_snd_cmd(cmd_in);
      wait_cmd_rdy_with_timeout();

      if (wrp_cmd !== cmd_in) begin
        $display("ERROR: cmd mismatch exp=%h got=%h", cmd_in, wrp_cmd);
        $stop;
      end

      clear_cmd_rdy();

      send_ack(ack_in);
      wait_resp_rdy_with_timeout();

      if (rem_resp !== ack_in) begin
        $display("ERROR: resp mismatch exp=%h got=%h", ack_in, rem_resp);
        $stop;
      end
    end
  endtask

  // Directed + random regression sequence.
  initial begin : stimulus
    int unsigned idx;
    logic [15:0] rand_cmd;
    logic [7:0]  rand_ack;

    rst_n            = 1'b0;
    rem_snd_cmd      = 1'b0;
    rem_cmd          = 16'h0000;
    wrp_clr_cmd_rdy  = 1'b0;
    wrp_trmt         = 1'b0;
    wrp_resp         = 8'h00;

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    run_case(16'hABCD, 8'hA5);

    for (idx = 0; idx < RANDOM_TESTS; idx++) begin
      rand_cmd = $urandom;
      rand_ack = $urandom;
      run_case(rand_cmd, rand_ack);
    end

    $display("YAHOO! All tests passed!");
    $stop;
  end

endmodule
