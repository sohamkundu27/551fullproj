//////////////////////////////////////////////////////////////////////////////////////////
// Maze-solving state machine for autonomous mode.                                     //
// Chooses move/turn commands using opening sensors and move-complete feedback.       //
///////////////////////////////////////////////////////////////////////////////////////
module maze_solve(clk, rst_n, cmd_md, cmd0, lft_opn, rght_opn, mv_cmplt, sol_cmplt, strt_hdng, dsrd_hdng, strt_mv, stp_lft, stp_rght);

// Inputs and outputs
input logic clk, rst_n;
input logic cmd_md, cmd0;
input logic lft_opn, rght_opn;
input logic mv_cmplt, sol_cmplt;
output logic strt_hdng;
output logic [11:0] dsrd_hdng;
output logic strt_mv;
output logic stp_lft, stp_rght;

// Direction values
localparam NORTH = 12'h000;
localparam WEST = 12'h3FF;
localparam SOUTH = 12'h7FF;
localparam EAST = 12'hC00;

// Internal control signals
logic lft_affn;          // 1: left-affinity policy, 0: right-affinity policy
logic sensor1_opn;       // preferred-side opening
logic sensor2_opn;       // non-preferred-side opening
logic [11:0] nxt_hdng;   // next heading to latch on strt_hdng

// Affinity select from command bit
assign lft_affn = cmd0;

// Stop condition selects for navigate
assign stp_lft = cmd0;   // left-affinity watches left openings
assign stp_rght = ~cmd0;   // right-affinity watches right openings

// Preferred and secondary openings based on affinity
assign sensor1_opn = (lft_affn) ? lft_opn : rght_opn;
assign sensor2_opn = (lft_affn) ? rght_opn : lft_opn;

// State declarations
typedef enum logic [2:0] {IDLE, MV_FRWD, WAIT_MV, TURN_S1, TURN_S2, TURN_180, WAIT_TURN, DONE} state_t;
state_t state, nxt_state;

// State register
always_ff @(posedge clk, negedge rst_n) begin
  if (!rst_n)
    state <= IDLE;
  else
    state <= nxt_state;
end

// State machine logic
always_comb begin
  // Default outputs
  strt_hdng = 1'b0;
  strt_mv = 1'b0;
  nxt_hdng = dsrd_hdng;
  nxt_state = state;

  case (state)
    IDLE: begin
      // Start autonomous motion when command mode is dropped
      // cmd_md is held high during manual command mode from cmd_proc
      if (!cmd_md)
        nxt_state = MV_FRWD;
    end

    MV_FRWD: begin
      // Fire a one-cycle move pulse, then wait for navigate to finish the segment
      strt_mv = 1'b1;
      nxt_state = WAIT_MV;
    end

    WAIT_MV: begin
      // Decision point after each forward segment:
      // First, stop if solution is found, otherwise, prefer affinity-side branch, then secondary branch, else do a 180 at dead ends
      if (mv_cmplt) begin
        if (sol_cmplt)
          nxt_state = DONE;                 // Magnet found
        else if (sensor1_opn)
          nxt_state = TURN_S1;              // Preferred opening
        else if (sensor2_opn)
          nxt_state = TURN_S2;              // Secondary opening
        else
          nxt_state = TURN_180;             // Dead end
      end
    end

    TURN_S1: begin
      // Turn toward the preferred (affinity-side) branch
      strt_hdng = 1'b1;
      if (lft_affn) begin
        // Left-affinity: rotate 90 deg CCW in wrapped heading space
        if (dsrd_hdng == NORTH) nxt_hdng = WEST;
        else if (dsrd_hdng == WEST) nxt_hdng = SOUTH;
        else if (dsrd_hdng == SOUTH) nxt_hdng = EAST;
        else if (dsrd_hdng == EAST) nxt_hdng = NORTH;
      end else begin
        // Right-affinity: rotate 90 deg CW in wrapped heading space
        if (dsrd_hdng == NORTH) nxt_hdng = EAST;
        else if (dsrd_hdng == EAST) nxt_hdng = SOUTH;
        else if (dsrd_hdng == SOUTH) nxt_hdng = WEST;
        else if (dsrd_hdng == WEST) nxt_hdng = NORTH;
      end
      // navigate asserts mv_cmplt when heading settle completes
      nxt_state = WAIT_TURN;
    end

    TURN_S2: begin
      // Turn toward the non-preferred branch when preferred is unavailable
      strt_hdng = 1'b1;
      if (lft_affn) begin
        // Left-affinity fallback is a 90 deg CW turn
        if (dsrd_hdng == NORTH) nxt_hdng = EAST;
        else if (dsrd_hdng == EAST) nxt_hdng = SOUTH;
        else if (dsrd_hdng == SOUTH) nxt_hdng = WEST;
        else if (dsrd_hdng == WEST) nxt_hdng = NORTH;
      end else begin
        // Right-affinity fallback is a 90 deg CCW turn
        if (dsrd_hdng == NORTH) nxt_hdng = WEST;
        else if (dsrd_hdng == WEST) nxt_hdng = SOUTH;
        else if (dsrd_hdng == SOUTH) nxt_hdng = EAST;
        else if (dsrd_hdng == EAST) nxt_hdng = NORTH;
      end
      nxt_state = WAIT_TURN;
    end

    TURN_180: begin
      // Dead-end recovery: reverse heading and continue wall-following
      strt_hdng = 1'b1;
      if (dsrd_hdng == NORTH) nxt_hdng = SOUTH;
      else if (dsrd_hdng == SOUTH) nxt_hdng = NORTH;
      else if (dsrd_hdng == EAST) nxt_hdng = WEST;
      else if (dsrd_hdng == WEST) nxt_hdng = EAST;
      nxt_state = WAIT_TURN;
    end

    WAIT_TURN: begin
      // Hold until navigate reports heading move complete
      if (mv_cmplt)
        nxt_state = MV_FRWD;
    end

    DONE: begin
      // Terminal state; cmd_proc handles mode return/response externally
      nxt_state = DONE;
    end

    default: begin
      nxt_state = IDLE;
    end
  endcase
end

// Update heading register on turn command
always_ff @(posedge clk, negedge rst_n) begin
  if (!rst_n)
    dsrd_hdng <= NORTH;
  else if (strt_hdng)
    dsrd_hdng <= nxt_hdng;
end
endmodule