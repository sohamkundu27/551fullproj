// navigate_shell.sv
// Navigation/motion-control FSM.
// Sequences heading alignment, acceleration/cruise/decel, and move completion.
module navigate(clk,rst_n,strt_hdng,strt_mv,stp_lft,stp_rght,mv_cmplt,hdng_rdy,moving,
                en_fusion,at_hdng,lft_opn,rght_opn,frwrd_opn,frwrd_spd);

  parameter FAST_SIM = 1;

  input clk,rst_n;
  input strt_hdng;
  input strt_mv;
  input stp_lft;
  input stp_rght;
  input hdng_rdy;
  output logic mv_cmplt;
  output logic moving;
  output en_fusion;
  input at_hdng;
  input lft_opn,rght_opn,frwrd_opn;
  output reg [10:0] frwrd_spd;

  typedef enum logic [2:0] {IDLE, HEADING, ACCEL, DECEL, DECEL_FAST} state_t;
  state_t state, nxt_state;

  logic init_frwrd, inc_frwrd, dec_frwrd, dec_frwrd_fast;
  logic lft_opn_ff, rght_opn_ff;
  wire lft_opn_rise, rght_opn_rise;

  localparam MAX_FRWRD = 11'h2A0;
  localparam MIN_FRWRD = 11'h0D0;

  ////////////////////////////////////////
  // frwrd_inc based on FAST_SIM param //
  //////////////////////////////////////
  wire [5:0] frwrd_inc;

  generate if (FAST_SIM) begin
    assign frwrd_inc = 6'h18;
  end else begin
    assign frwrd_inc = 6'h02;
  end endgenerate

  ///////////////////////////////
  // Rising edge detectors    //
  /////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      lft_opn_ff <= 1'b0;
      rght_opn_ff <= 1'b0;
    end else begin
      lft_opn_ff <= lft_opn;
      rght_opn_ff <= rght_opn;
    end

  assign lft_opn_rise = lft_opn & ~lft_opn_ff;
  assign rght_opn_rise = rght_opn & ~rght_opn_ff;

  ////////////////////////
  // en_fusion signal  //
  //////////////////////
  assign en_fusion = (frwrd_spd > (MAX_FRWRD >> 1));

  ////////////////////////////////
  // Now form forward register //
  //////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  frwrd_spd <= 11'h000;
	else if (init_frwrd)
	  frwrd_spd <= MIN_FRWRD;
	else if (hdng_rdy && inc_frwrd && (frwrd_spd<MAX_FRWRD))
	  frwrd_spd <= frwrd_spd + {5'h00,frwrd_inc};
	else if (hdng_rdy && (frwrd_spd>11'h000) && (dec_frwrd | dec_frwrd_fast))
	  frwrd_spd <= ((dec_frwrd_fast) && (frwrd_spd>{2'h0,frwrd_inc,3'b000})) ? frwrd_spd - {2'h0,frwrd_inc,3'b000} :
                    (dec_frwrd_fast) ? 11'h000 :
	                (frwrd_spd>{4'h0,frwrd_inc,1'b0}) ? frwrd_spd - {4'h0,frwrd_inc,1'b0} :
					11'h000;

  ///////////////////////
  // State register   //
  /////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      state <= IDLE;
    else
      state <= nxt_state;

  ////////////////////////////////////////
  // Next state and output logic       //
  //////////////////////////////////////
  always_comb begin
    nxt_state = state;
    moving = 1'b0;
    mv_cmplt = 1'b0;
    init_frwrd = 1'b0;
    inc_frwrd = 1'b0;
    dec_frwrd = 1'b0;
    dec_frwrd_fast = 1'b0;

    case (state)
      IDLE: begin
        if (strt_hdng)
          nxt_state = HEADING;
        else if (strt_mv) begin
          nxt_state = ACCEL;
          init_frwrd = 1'b1;
        end
      end

      HEADING: begin
        moving = 1'b1;
        if (at_hdng) begin
          nxt_state = IDLE;
          mv_cmplt = 1'b1;
        end
      end

      ACCEL: begin
        moving = 1'b1;
        if (~frwrd_opn) begin
          nxt_state = DECEL_FAST;
        end else if ((stp_lft & lft_opn_rise) | (stp_rght & rght_opn_rise)) begin
          nxt_state = DECEL;
        end else begin
          inc_frwrd = 1'b1;
        end
      end

      DECEL: begin
        moving = 1'b1;
        if (frwrd_spd == 11'h000) begin
          nxt_state = IDLE;
          mv_cmplt = 1'b1;
        end else begin
          dec_frwrd = 1'b1;
        end
      end

      DECEL_FAST: begin
        moving = 1'b1;
        if (frwrd_spd == 11'h000) begin
          nxt_state = IDLE;
          mv_cmplt = 1'b1;
        end else begin
          dec_frwrd_fast = 1'b1;
        end
      end

      default: nxt_state = IDLE;
    endcase
  end

endmodule
