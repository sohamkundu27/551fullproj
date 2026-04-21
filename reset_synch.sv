module reset_synch(
  input  logic RST_n,
  input  logic clk,
  output logic rst_n
);

  logic ff1, ff2;

  // Asynchronously assert reset from push button, deassert on negedge clk.
  always_ff @(negedge clk, negedge RST_n) begin
    if (!RST_n) begin
      ff1 <= 1'b0;
      ff2 <= 1'b0;
    end else begin
      ff1 <= 1'b1;
      ff2 <= ff1;
    end
  end

  assign rst_n = ff2;

endmodule
