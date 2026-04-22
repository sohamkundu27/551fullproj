// piezo_drv.sv
// Audio-pattern generator for the piezo buzzer.
// Plays a fanfare sequence and battery-low alert tones.
module piezo_drv(clk, rst_n, batt_low, fanfare, piezo, piezo_n);

// inputs and outputs
input logic clk, rst_n, batt_low, fanfare;
output logic piezo, piezo_n;

// state machine states
typedef enum logic [2:0] {IDLE, N1, N2, N3, N4, N5, N6} state_t;
state_t state, nxt_state;

//parameter for clk speed used in calculating note frequencies and durations
localparam clk_speed = 50000000;
// parameter for fast simulation - increases note duration by 16x to make it easier to see the notes being played
parameter FAST_SIM = 0;

// hard coded note durations for each note
localparam note_dur = 8388608;
localparam note_dur2 = 12582912;
localparam note_dur3 = 4194304;
localparam note_dur4 = 16777216;
logic [4:0] note_dur_inc;

// hard coded note frequencies
localparam G6_HALF = 50000000 / (2 * 1568);  // ~15943
localparam C7_HALF = 50000000 / (2 * 2093);  // ~11944
localparam E7_HALF = 50000000 / (2 * 2637);  //  ~9482
localparam G7_HALF = 50000000 / (2 * 3136);  //  ~7972

// registers for current note's half period and duration
logic [14:0] curr_half_period;

// adjust note duration for fast simulation
generate if (FAST_SIM) begin
		assign note_dur_inc = 16;
	end else begin
		assign note_dur_inc = 1;
	end
endgenerate


// register to track if we are currently in a batt_low sequence
logic batt_low_run;
// register for current note duration, used in note duration counter
logic [24:0] curr_note_dur;


// note frequency counter
logic [24:0] freq_cnt;
logic freq_done;
always_ff @(posedge clk, negedge rst_n) begin
	if (!rst_n) // reset frequency counter on reset
		freq_cnt <= 0;
	else if (state == IDLE || freq_done) // reset frequency counter at end of each note or in IDLE
		freq_cnt <= 0;
	else // increment frequency counter every clock cycle to track when to toggle piezo output for current note's frequency
		freq_cnt <= freq_cnt + 1;
end

// frequency done when counter reaches current note's half period
assign freq_done = (freq_cnt >= curr_half_period);

// note count for tracking note duration, increments every clock cycle and resets at end of note
logic [24:0] note_cnt;
// note done when note count reaches current note duration
logic note_done;
// note duration counter
always_ff @(posedge clk, negedge rst_n) begin
	if (!rst_n) // reset note counter on reset
		note_cnt <= 0;
	else if (note_cnt >= curr_note_dur) // reset note counter at end of note duration to prepare for next note
		note_cnt <= 0;
	else if (!note_done) // increments note count if not is not done
		note_cnt <= note_cnt + note_dur_inc;
	else // hold note count at current value if note is done until state machine updates to next note
		note_cnt <= note_cnt;
end

// flip flop to track when current note is done based on note count reaching current note duration
always_ff @(posedge clk, negedge rst_n) begin
	if (!rst_n) // reset note done on reset
		note_done <= 0;
	else if (note_cnt >= curr_note_dur) begin // set note done when note count reaches current note duration
		note_done <= 1;
	end else note_done <= 0; // reset note done when note count is less than current note duration to prepare for next note
end

// batt_low_run FF
always_ff @(posedge clk, negedge rst_n) begin
	if (!rst_n) // reset batt_low_run on reset
		batt_low_run <= 0;
	else if (state == IDLE) // if in IDLE, batt_low_run depends on batt_low
		batt_low_run <= batt_low;
end

// FSM update
always_ff @(posedge clk, negedge rst_n) begin
	if (!rst_n) // reset to IDLE on reset	
		state <= IDLE;
	else // update state to next state every clock cycle
		state <= nxt_state;
end

// FSM comb logic
always_comb begin
	// default outputs and next state
	curr_half_period = 0;
	curr_note_dur = 0;
	nxt_state = state;
	
	case (state)
		IDLE: begin // in IDLE, wait for batt_low or fanfare to start sequence
			if (batt_low) begin
				nxt_state = N1; // if batt_low is high, start batt_low sequence at N1
			end else if (fanfare) begin
				nxt_state = N1; // if fanfare is high, start fanfare sequence at N1
			end
		end
		N1: begin // in N1, play G6 note for batt_low or fanfare sequence
			curr_note_dur = note_dur; // set current note duration
			curr_half_period = G6_HALF; // set current note half period for G6
			if (note_done) nxt_state = N2; // when note is done, move to N2 to play next note in sequence
		end
		N2: begin // in N2, play C7 note for batt_low or fanfare sequence
			curr_note_dur = note_dur; // set current note duration
			curr_half_period = C7_HALF; // set current note half period for C7
			if (note_done) nxt_state = N3; // when note is done, move to N3 to play next note in sequence
		end
		N3: begin // in N3, play E7 note for batt_low or fanfare sequence
			curr_note_dur = note_dur; // set current note duration
			curr_half_period = E7_HALF; // set current note half period for E7
			if (note_done) begin // check if note is done to determine next state based on 
								 // if we are in batt_low sequence or fanfare sequence
				if (batt_low_run && batt_low) nxt_state = N1; // if in batt_low sequence and batt_low is still high, 
															  // loop back to N1 to repeat sequence
				else if (batt_low_run) nxt_state = IDLE; // batt_low ended during note, go back to IDLE after note finishes
				else nxt_state = N4; // if in fanfare sequence, move to N4 to play next note in fanfare sequence after note is done
			end
		end
		N4: begin // in N4, play G7 note for fanfare sequence
			curr_note_dur = note_dur2; // set current note duration for G7 in fanfare sequence
			curr_half_period = G7_HALF; // set current note half period for G7
			if (note_done) nxt_state = N5; // when note is done, move to N5 to play next note in fanfare sequence
		end
		N5: begin // in N5, play E7 note again for fanfare sequence
			curr_note_dur = note_dur3; // set current note duration for E7 in fanfare sequence
			curr_half_period = E7_HALF; // set current note half period for E7
			if (note_done) nxt_state = N6; // when note is done, move to N6 to play next note in fanfare sequence
		end
		N6: begin // in N6, play G7 note again for fanfare sequence
			curr_note_dur = note_dur4; // set current note duration for G7 in fanfare sequence
			curr_half_period = G7_HALF; // set current note half period for G7
			if (note_done) nxt_state = IDLE; // when note is done, go back to IDLE to end fanfare sequence
		end
		default: nxt_state = IDLE; // default to IDLE for safety
	endcase
end

// piezo output logic, toggles at current note frequency when not in IDLE to generate sound on piezo
logic piezo_out;
// piezo_out toggles at current note frequency when not in IDLE, holds 0 in IDLE
always_ff @(posedge clk, negedge rst_n) begin
	if (!rst_n) // reset piezo_out on reset
		piezo_out <= 0;
	else if (state == IDLE) // hold piezo_out low in IDLE to keep piezo silent
		piezo_out <= 0;
	else if (freq_done)	// toggle piezo_out at current note frequency to generate sound on piezo
		piezo_out <= ~piezo_out;
	else // hold piezo_out at current value between frequency toggles to maintain correct note frequency
		piezo_out <= piezo_out;	
end

// assign piezo outputs, hold low in IDLE and toggle at current note frequency when not in IDLE to generate sound on piezo
assign piezo = (state == IDLE) ? 1'b0 : piezo_out;
assign piezo_n = (state == IDLE) ? 1'b0 : ~piezo_out;

endmodule