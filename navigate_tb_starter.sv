// navigate_tb_starter.sv
// Unit testbench for the navigate FSM.
// Drives heading/move commands and checks movement-state behavior.
module navigate_tb();

  //// Declare stimulus as type reg ////
  reg clk,rst_n;			// 50MHz clock and asynch active low reset
  reg strt_hdng;			// indicates to start a new heading sequence
  reg strt_mv;				// indicates a new forward movement occurring
  reg stp_lft;				// indicates move should stop at a left opening
  reg stp_rght;				// indicates move should stop at a right opening
  reg hdng_rdy;				// used to pace frwrd_spd increments
  reg at_hdng;				// asserted by PID when new heading is close enough
  reg lft_opn;				// from IR sensor....indicates opening in maze to left
  reg rght_opn;				// from IR sensor....indicates opening in maze to right
  reg frwrd_opn;			// from IR sensor....indicates opening in front
  
  //// declare outputs monitored of type wire ////
  wire mv_cmplt;			// should be asserted at end of move
  wire moving;				// should be asserted at all times not in IDLE
  wire en_fusion;			// should be asserted whenever frwrd_spd>MAX_FRWRD/2
  wire [10:0] frwrd_spd;	// the primary output...forward motor speed

  localparam FAST_SIM = 1;	// we always simulate with FAST_SIM on
  localparam MIN_FRWRD = 11'h0D0;		// minimum duty at which wheels will turn
  localparam MAX_FRWRD = 11'h2A0;		// max forward speed
  
  //////////////////////
  // Instantiate DUT //
  ////////////////////
  navigate #(FAST_SIM) iDUT(.clk(clk),.rst_n(rst_n),.strt_hdng(strt_hdng),.strt_mv(strt_mv),
                .stp_lft(stp_lft),.stp_rght(stp_rght),.mv_cmplt(mv_cmplt),.hdng_rdy(hdng_rdy),
				.moving(moving),.en_fusion(en_fusion),.at_hdng(at_hdng),.lft_opn(lft_opn),
				.rght_opn(rght_opn),.frwrd_opn(frwrd_opn),.frwrd_spd(frwrd_spd));
  
  initial begin
    clk = 0;
	rst_n = 0;
	strt_hdng = 0;
	strt_mv = 0;
	stp_lft = 1;
	stp_rght = 0;
	hdng_rdy = 1;		// allow increments of frwrd_spd initially
	at_hdng = 0;
	lft_opn = 0;
	rght_opn = 0;
	frwrd_opn = 1;		// no wall in front of us
	
	@(negedge clk);		// after negege clk
	rst_n = 1;			// deassert reset
	
	assert (!moving) $display("GOOD0: moving should not be asserted when IDLE");
	else $error("ERR0: why is moving asserted now?");	
	//////////////////////////////////////////////
	// First testcase will be a heading change //
	////////////////////////////////////////////
	strt_hdng = 1;
	@(negedge clk);
	strt_hdng = 0;
	assert (moving) $display("GOOD1: moving asserted during heading change");
	else $error("ERR1: expecting moving asserted during heading change");
	repeat(5) @(negedge clk);
	at_hdng = 1;				// end the heading
	#1;							// give DUT time to respond
	assert (mv_cmplt) $display("GOOD2: mv_cmplt should be asserted when at_hdng");
	else $error("ERR2: expecting mv_cmplt to be asserted at time %t",$time);
	@(negedge clk);
	at_hdng = 0;
	
	///////////////////////////////////////////////////////////////////////////////////
	// Second testcase will be move forward looking for lft_opn, but hit wall first //
	/////////////////////////////////////////////////////////////////////////////////
	strt_mv = 1;
	@(negedge clk);
	strt_mv = 0;
	assert (moving) $display("GOOD3: moving asserted during forward move");
	else $error("ERR3: expecting moving asserted during forward move");
	assert (frwrd_spd===MIN_FRWRD) $display("GOOD4: frwrd spd should have changed to MIN_FRWRD");
	else $error("ERR4: expecting frwrd_spd to have loaded MIN_FRWRD at time %t",$time);

	@(negedge clk);
	assert (frwrd_spd===MIN_FRWRD+11'h018) $display("GOOD5: frwrd spd should have incrementd to MIN_FRWRD+0x018");
	else $error("ERR5: expecting frwrd_spd to have incremented by 0x018 at time %t",$time);	
	
	/// Now lower hdng_rdy to ensure frwrd_spd does not increment ////
	hdng_rdy = 0;
	@(negedge clk);
	assert (frwrd_spd===MIN_FRWRD+11'h018) $display("GOOD6: frwrd spd should still be MIN_FRWRD+0x018");
	else $error("ERR6: expecting frwrd_spd to have maintained at MIN_FRWRD+0x018");	
	
	/// Now raise hdng_rdy back up
	hdng_rdy = 1;
	@(negedge clk);
	assert (moving) $display("GOOD7: moving should still be asserted");
	else $error("ERR7: why is moving not still asserted?");
	assert (frwrd_spd===MIN_FRWRD+11'h030) $display("GOOD8: frwrd spd should have incremented to MIN_FRWRD+0x030");
	else $error("ERR8: expecting frwrd_spd to have incremented to MIN_FRWRD+0x030 at time %t",$time);
	
	/// Now let it increment 6 more times (so 9 in total) ////
	repeat(6) @(negedge clk);
	
	/// Now let it know it has an obstacle in front ////
	frwrd_opn = 0;
	repeat(2) @(negedge clk);
	assert (frwrd_spd===MIN_FRWRD+11'h018) $display("GOOD9: frwrd spd should have decremented fast to MIN_FRWRD+0x018");
	else $error("ERR9: expecting a fast decrement of frwrd_spd at time %t",$time);	
	
	/// Now check that it properly decrements to zero ////
	repeat(2) @(negedge clk);
	assert (frwrd_spd===11'h000) $display("GOOD10: frwrd spd should be zero now");
	else $error("ERR10: expecting frwrd_spd to have decremented to zero by time %t",$time);	
	assert (mv_cmplt) $display("GOOD11: mv_cmplt should be asserted when speed hits zero");
	else $error("ERR11: expecting mv_cmplt to be asserted at time %t",$time);	
	
	///////////////////////////////////////////////////////////////////////////
	// Third testcase: move with left opening stop (normal decel)           //
	//   Also checks en_fusion                                             //
	/////////////////////////////////////////////////////////////////////////
	@(negedge clk);
	frwrd_opn = 1;
	stp_lft = 1;
	stp_rght = 0;
	lft_opn = 0;
	rght_opn = 0;

	strt_mv = 1;
	@(negedge clk);
	strt_mv = 0;
	assert (moving) $display("GOOD12: moving asserted for left opening move");
	else $error("ERR12: expecting moving at time %t", $time);
	assert (frwrd_spd === MIN_FRWRD) $display("GOOD13: frwrd_spd loaded MIN_FRWRD for left opening test");
	else $error("ERR13: expecting MIN_FRWRD at time %t", $time);

	// After 5 increments: speed = 0x0D0+5*0x018 = 0x148 < 0x150 → en_fusion=0
	repeat(5) @(negedge clk);
	assert (!en_fusion) $display("GOOD14: en_fusion not yet asserted at speed 0x%h", frwrd_spd);
	else $error("ERR14: en_fusion should not be asserted yet at speed 0x%h", frwrd_spd);

	// After 6th increment: speed = 0x160 > 0x150 → en_fusion=1
	@(negedge clk);
	assert (en_fusion) $display("GOOD15: en_fusion asserted at speed 0x%h (above half MAX_FRWRD)", frwrd_spd);
	else $error("ERR15: en_fusion should be asserted at speed 0x%h", frwrd_spd);

	// Two more increments → 8 total: speed = 0x190
	repeat(2) @(negedge clk);

	// Trigger left opening rising edge → transitions to DECEL
	// 1 transition cycle + 9 decel cycles (@0x030 per) = 10 cycles to speed=0
	lft_opn = 1;
	repeat(10) @(negedge clk);
	assert (frwrd_spd === 11'h000) $display("GOOD16: frwrd_spd reached zero after normal decel (left opening)");
	else $error("ERR16: expecting frwrd_spd zero, got 0x%h at time %t", frwrd_spd, $time);
	assert (mv_cmplt) $display("GOOD17: mv_cmplt asserted after left opening stop");
	else $error("ERR17: expecting mv_cmplt at time %t", $time);
	assert (!en_fusion) $display("GOOD18: en_fusion de-asserted at zero speed");
	else $error("ERR18: en_fusion should be off at zero speed");

	///////////////////////////////////////////////////////////////////////////
	// Fourth testcase: move with right opening stop (normal decel)         //
	/////////////////////////////////////////////////////////////////////////
	@(negedge clk);
	frwrd_opn = 1;
	stp_lft = 0;
	stp_rght = 1;
	lft_opn = 0;
	rght_opn = 0;

	strt_mv = 1;
	@(negedge clk);
	strt_mv = 0;
	assert (moving) $display("GOOD19: moving asserted for right opening move");
	else $error("ERR19: expecting moving at time %t", $time);

	// 8 increments: speed = 0x190
	repeat(8) @(negedge clk);

	// Trigger right opening → DECEL, same 10-cycle pattern as left
	rght_opn = 1;
	repeat(10) @(negedge clk);
	assert (frwrd_spd === 11'h000) $display("GOOD20: frwrd_spd reached zero after normal decel (right opening)");
	else $error("ERR20: expecting frwrd_spd zero, got 0x%h at time %t", frwrd_spd, $time);
	assert (mv_cmplt) $display("GOOD21: mv_cmplt asserted after right opening stop");
	else $error("ERR21: expecting mv_cmplt at time %t", $time);

	///////////////////////////////////////////////////////////////////////////
	// Fifth testcase: opening already asserted before move starts          //
	//   Verify rising edge detector prevents immediate stop                //
	/////////////////////////////////////////////////////////////////////////
	@(negedge clk);
	frwrd_opn = 1;
	stp_lft = 1;
	stp_rght = 0;
	lft_opn = 1;				// pre-assert opening BEFORE move
	rght_opn = 0;
	@(negedge clk);				// allow lft_opn_ff to latch to 1

	strt_mv = 1;
	@(negedge clk);
	strt_mv = 0;
	assert (moving) $display("GOOD22: moving asserted with pre-asserted opening");
	else $error("ERR22: expecting moving at time %t", $time);

	// Robot should NOT stop — lft_opn was already high, no rising edge
	repeat(3) @(negedge clk);
	assert (moving) $display("GOOD23: still moving — pre-asserted opening did not trigger stop");
	else $error("ERR23: robot should not have stopped on pre-asserted opening at time %t", $time);
	assert (frwrd_spd > MIN_FRWRD) $display("GOOD24: speed increasing (ACCEL), frwrd_spd=0x%h", frwrd_spd);
	else $error("ERR24: speed should be > MIN_FRWRD, got 0x%h at time %t", frwrd_spd, $time);

	// Create a new rising edge: deassert then reassert lft_opn
	lft_opn = 0;
	@(negedge clk);				// lft_opn_ff updates to 0 at posedge
	lft_opn = 1;				// rising edge on next posedge

	// Speed was 0x130 at this point (4 increments + 1 inc during deassert cycle)
	// 1 transition cycle + 7 decel cycles = 8 cycles to zero
	repeat(8) @(negedge clk);
	assert (frwrd_spd === 11'h000) $display("GOOD25: frwrd_spd reached zero after rising-edge stop");
	else $error("ERR25: expecting frwrd_spd zero, got 0x%h at time %t", frwrd_spd, $time);
	assert (mv_cmplt) $display("GOOD26: mv_cmplt asserted after edge-detect stop");
	else $error("ERR26: expecting mv_cmplt at time %t", $time);

	///////////////////////////////////////////////////////////////////////////
	// Sixth testcase: back-to-back commands                                //
	//   Heading change immediately followed by a forward move              //
	/////////////////////////////////////////////////////////////////////////
	@(negedge clk);
	lft_opn = 0;

	strt_hdng = 1;
	@(negedge clk);
	strt_hdng = 0;
	assert (moving) $display("GOOD27: moving asserted during back-to-back heading change");
	else $error("ERR27: expecting moving at time %t", $time);

	repeat(3) @(negedge clk);
	at_hdng = 1;
	#1;
	assert (mv_cmplt) $display("GOOD28: mv_cmplt for back-to-back heading");
	else $error("ERR28: expecting mv_cmplt at time %t", $time);
	@(negedge clk);
	at_hdng = 0;

	// Immediately follow with a forward move that hits a wall
	frwrd_opn = 1;
	stp_lft = 0;
	stp_rght = 0;
	strt_mv = 1;
	@(negedge clk);
	strt_mv = 0;
	assert (moving) $display("GOOD29: moving asserted for back-to-back forward move");
	else $error("ERR29: expecting moving at time %t", $time);

	// 4 increments: speed = 0x0D0+4*0x018 = 0x130
	repeat(4) @(negedge clk);
	frwrd_opn = 0;
	// 1 transition + 2 fast decel cycles (0x130→0x070→0x000) = 3 cycles
	repeat(3) @(negedge clk);
	assert (frwrd_spd === 11'h000) $display("GOOD30: frwrd_spd zero after back-to-back wall stop");
	else $error("ERR30: expecting frwrd_spd zero, got 0x%h at time %t", frwrd_spd, $time);
	assert (mv_cmplt) $display("GOOD31: mv_cmplt for back-to-back wall stop");
	else $error("ERR31: expecting mv_cmplt at time %t", $time);

	@(negedge clk);
	assert (!moving) $display("GOOD32: back in IDLE, moving de-asserted");
	else $error("ERR32: expecting not moving at time %t", $time);

	$display("All tests completed...did all pass?");
	$stop();
	
  end
  
  always
    #5 clk = ~clk;
	
endmodule
