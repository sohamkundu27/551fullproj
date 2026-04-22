module maze_solve(clk, rst_n, cmd_md, cmd0, lft_opn, rght_opn, mv_cmplt, sol_cmplt, strt_hdng, dsrd_hdng, strt_mv, stp_lft, stp_rght);

// inputs and outputs
input logic clk, rst_n;
input logic cmd_md, cmd0;
input logic lft_opn, rght_opn;
input logic mv_cmplt, sol_cmplt;
output logic strt_hdng; 
output logic [11:0] dsrd_hdng;
output logic strt_mv;
output logic stp_lft, stp_rght;

//localparam clk_speed = 50000000;

// direction values
localparam NORTH = 12'h000;
localparam WEST = 12'h3FF;
localparam SOUTH = 12'h7FF;
localparam EAST = 12'hC00;

// logic statements
logic lft_affn; // tells if system is left or right affinity
logic sensor1_opn; // if left affinity, checks if left is open, otherwise checks if right is open
logic sensor2_opn; // if left affinity, checks if right is open, otherwise checks if left is open
logic [11:0] nxt_hdng; // next heading direction, used to update desird heading


// if cmd[0] is high system is left affinty otherwise right affinity
assign lft_affn = cmd0;
// stp_lft/stp_rght tell navigate which kind of opening to stop at
assign stp_lft     =  cmd0;   // left-affinity means watch for left openings
assign stp_rght    = ~cmd0;   // right-affinity means watch for right openings

// sensor 1 and sesor 2 are based on if it is left or right affinity
assign sensor1_opn = (lft_affn) ? lft_opn : rght_opn;
assign sensor2_opn = (lft_affn) ? rght_opn : lft_opn;


// state declaraions
typedef enum logic [2:0] {IDLE, MV_FRWD, WAIT_MV, TURN_S1, TURN_S2, TURN_180, WAIT_TURN, DONE} state_t;
state_t state, nxt_state;

// state machine register
always_ff @(posedge clk, negedge rst_n) begin
	if (!rst_n) state <= IDLE;
	else state <= nxt_state;
end

// state machine logic
always_comb begin
	// default outputs
    strt_hdng = 0;
    strt_mv = 0;
    nxt_hdng = dsrd_hdng;
    nxt_state = state;
	
    
    case (state)
        IDLE: begin
            // if ((!cmd_md) && (mv_cmplt || !lft_opn || !rght_opn))begin
            //     nxt_state = MV_FRWD;
            // end


            // wait until cmd_md low to signify that a cmd was recieved
            if (!cmd_md)
                nxt_state = MV_FRWD;
        end

        MV_FRWD: begin
            // if (sol_cmplt) begin
            //     nxt_state = DONE;
            // end
            // else if ((!sol_cmplt) && (sensor1_opn)) begin
                
            //     strt_hdng = 1;
            //     nxt_state = TURN_S1;
            // end
            // else if ((!sol_cmplt) && (!sensor1_opn) && (sensor2_opn)) begin
            //     strt_hdng = 1;
            //     nxt_state = TURN_S2;
            // end
            // else if ((!sol_cmplt) && (!sensor1_opn) && (!sensor2_opn)) begin
            //     // strt_hdng = 1;
            //     nxt_state = TURN_180;
            // end
            // else nxt_state = MV_FRWD;

            strt_mv   = 1'b1;
            nxt_state = WAIT_MV;
        end


        WAIT_MV: begin
            if (mv_cmplt) begin // if previous move is complete
                if (sol_cmplt)
                    nxt_state = DONE;           // magnet found, at solution
                else if ((!sol_cmplt) && (sensor1_opn))
                    nxt_state = TURN_S1;       // preferred opening available
                else if ((!sol_cmplt) && (!sensor1_opn) && (sensor2_opn))
                    nxt_state = TURN_S2;       // secondary opening available
                else if ((!sol_cmplt) && (!sensor1_opn) && (!sensor2_opn))
                    nxt_state = TURN_180;       // dead end, turn 180 deg
                else nxt_state = MV_FRWD; // shouldn't happen but just in case
            end

        end


        TURN_S1: begin // Turn towards the affinity side
            strt_hdng = 1'b1; // assert strt_hdng
            if (lft_affn) begin // turn left bc left affinity is true
                if (dsrd_hdng == NORTH) nxt_hdng = WEST;
                else if (dsrd_hdng == WEST) nxt_hdng = SOUTH;
                else if (dsrd_hdng == SOUTH) nxt_hdng = EAST;
                else if (dsrd_hdng == EAST) nxt_hdng = NORTH;
            end
            else begin // turn right bc right affinity is true
                if (dsrd_hdng == NORTH) nxt_hdng = EAST;
                else if (dsrd_hdng == EAST) nxt_hdng = SOUTH;
                else if (dsrd_hdng == SOUTH) nxt_hdng = WEST;
                else if (dsrd_hdng == WEST) nxt_hdng = NORTH;
            end
            nxt_state = WAIT_TURN; // State that Waits for turn to complete
        end

        TURN_S2: begin // Turn towards the non affinity side
            strt_hdng = 1'b1; // assert strt_hdng
            if (lft_affn) begin // turn right bc left affinity true
                if (dsrd_hdng == NORTH) nxt_hdng = EAST;
                else if (dsrd_hdng == EAST) nxt_hdng = SOUTH;
                else if (dsrd_hdng == SOUTH) nxt_hdng = WEST;
                else if (dsrd_hdng == WEST) nxt_hdng = NORTH;
            end
            else begin // turn left bc right affinity true
                if (dsrd_hdng == NORTH) nxt_hdng = WEST;
                else if (dsrd_hdng == WEST) nxt_hdng = SOUTH;
                else if (dsrd_hdng == SOUTH) nxt_hdng = EAST;
                else if (dsrd_hdng == EAST) nxt_hdng = NORTH;   
            end
            nxt_state = WAIT_TURN; // State that Waits for turn to complete
        end

        TURN_180: begin // turn 180 deg bc dead end
            strt_hdng = 1'b1; // assert strt_hdng
            if (dsrd_hdng == NORTH) nxt_hdng = SOUTH;
            else if (dsrd_hdng == SOUTH) nxt_hdng = NORTH;
            else if (dsrd_hdng == EAST) nxt_hdng = WEST;
            else if (dsrd_hdng == WEST) nxt_hdng = EAST; 
            nxt_state = WAIT_TURN; // State that Waits for turn to complete
        end


        WAIT_TURN: begin // Wait for each turn to finish
            if (mv_cmplt)
                nxt_state = MV_FRWD; // go back to MV_FRWD to decide what move to do
        end

        DONE: nxt_state = DONE; // Stay in DONE bc solution was found
		
        default: nxt_state = IDLE; // default state is IDLE
	endcase
end



// updates dsrd_hdng with nxt_hdg based on FSM outputs
always_ff @(posedge clk, negedge rst_n) begin
	if (!rst_n) dsrd_hdng <= NORTH; // defaults to NORTH
	else if (strt_hdng) dsrd_hdng <= nxt_hdng;
	else dsrd_hdng <= dsrd_hdng;
end

	



endmodule