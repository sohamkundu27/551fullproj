///////////////////////////////////////////////////////////////////////////////////////////
// Top-level UART wrapper combining UART_tx and UART_rx submodules.                     //
// Provides byte-level TX/RX handshake signals for higher-level logic.                 //
////////////////////////////////////////////////////////////////////////////////////////
module UART(clk, rst_n, RX, TX, rx_rdy, clr_rx_rdy, rx_data, trmt, tx_data, tx_done)

input clk, rst_n;               // clock and active-low reset
input RX, trmt;                 // trmt starts TX with tx_data
input clr_rx_rdy;               // clears rx_rdy
input [7:0] tx_data;            // byte to transmit
output TX, rx_rdy, tx_done;     // RX ready and TX complete flags
output [7:0] rx_data;           // received byte

// Transmitter
UART_tx iTX(
  .clk(clk),
  .rst_n(rst_n),
  .TX(TX),
  .trmt(trmt),
  .tx_data(tx_data),
  .tx_done(tx_done)
);

// Receiver
UART_rx iRX(
  .clk(clk),
  .rst_n(rst_n),
  .RX(RX),
  .rdy(rx_rdy),
  .clr_rdy(clr_rx_rdy),
  .rx_data(rx_data)
);

endmodule
