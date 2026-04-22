// piezo_drv_test.sv
// FPGA/top-level bring-up wrapper for piezo_drv.
// Maps board inputs/outputs for interactive tone testing.
module piezo_drv_test(clk, RST_n, fanfare, batt_low, piezo, piezo_n, LED);
 
    // signal declarations
    input        clk;       // 50MHz clock (PIN_R8)
    input        RST_n;     // Push button reset, active low (PIN_J15)
    input        fanfare;   // DIP switch SW0: assert to play Charge! (PIN_M1)
    input        batt_low;  // DIP switch SW1: assert for battery low tone (PIN_T8)
    output       piezo;     // Piezo differential drive + (PIN_D11)
    output       piezo_n;   // Piezo differential drive - (PIN_B12)
    output [7:0] LED;       // LEDs for visual debug feedback
 
    // Internal synchronized reset
    wire rst_n;
 
   
   // reset_synch module to synchronize asynchronous reset input RST_n to the clock domain of the piezo driver
    reset_synch iRST(
        .clk(clk),
        .RST_n(RST_n),
        .rst_n(rst_n)
    );
 
    // Instantiate piezo driver — FAST_SIM=0 for real hardware timing
    piezo_drv #(.FAST_SIM(0)) iDUT(.clk(clk), .rst_n(rst_n), .batt_low(batt_low), .fanfare(fanfare), .piezo(piezo), .piezo_n(piezo_n));
 
    // // LED feedback for visual debugging on the board
    // assign LED[0] = fanfare;    // lit when fanfare switch is on
    // assign LED[1] = batt_low;   // lit when batt_low switch is on
    // assign LED[2] = piezo;      // toggles with piezo when playing
    // assign LED[3] = rst_n;      // lit when out of reset
    // assign LED[7:4] = 4'h0;     // unused LEDs off
 
endmodule
 