// SPI_mnrch.sv
// Alias module — SPI_mnrch is identical to SPI_main.
// inert_intf.sv instantiates SPI_mnrch; this simply wraps SPI_main.
// Soham Kundu

module SPI_mnrch(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        wrt,
  input  logic [15:0] wt_data,
  output logic        SS_n,
  output logic        SCLK,
  output logic        MOSI,
  input  logic        MISO,
  output logic        done,
  output logic [15:0] rd_data
);

  SPI_main iSPI(.*);

endmodule