/////////////////////////////////////////////////////
// Interfaces with ST 6-axis inertial sensor.  In  //
// this application we only use Z-axis gyro for   //
// heading of mazeRunner.  Fusion correction     //
// comes from IR_Dtrm when en_fusion is high.   //
/////////////////////////////////////////////////
module inert_intf(clk,rst_n,strt_cal,cal_done,heading,rdy,IR_Dtrm,
                  SS_n,SCLK,MOSI,MISO,INT,moving,en_fusion);

  parameter FAST_SIM = 1;	// used to speed up simulation
  
  input clk, rst_n;
  input MISO;							// SPI input from inertial sensor
  input INT;							// goes high when measurement ready
  input strt_cal;						// initiate claibration of yaw readings
  input moving;							// Only integrate yaw when going
  input en_fusion;						// do fusion corr only when forward at decent clip
  input [8:0] IR_Dtrm;					// derivative term of IR sensors (used for fusion)
  
  output cal_done;				// pulses high for 1 clock when calibration done
  output signed [11:0] heading;	// heading of robot.  000 = Orig dir 3FF = 90 CCW 7FF = 180 CCW
  output rdy;					// goes high for 1 clock when new outputs ready (from inertial_integrator)
  output SS_n,SCLK,MOSI;		// SPI outputs
 

  // Internal registers.
  logic [15:0] init_tmr;		// waits before the first setup write
  logic INT_ff1, INT_ff2;		// double flop for the sensor interrupt
  logic [7:0] yawL, yawH;		// stores the two bytes from the gyro
  
  // State-machine outputs.
  logic wrt;					// starts an SPI transfer
  logic vld;					// pulses when yaw_rt is ready
  logic [15:0] cmd;				// command sent to the SPI block
  logic clr_init_tmr, en_init_tmr;
  logic ld_yawL, ld_yawH;		// load controls for the yaw byte registers

  // Internal signals connecting sub-blocks.
  wire done;
  wire [15:0] inert_data;		// Data back from inertial sensor (only lower 8-bits used)
  wire signed [15:0] yaw_rt;
  wire init_tmr_full;
  wire INT_synched;
  
  
  // FSM state encoding.
  typedef enum logic [3:0] {
    RST_WAIT,
    INIT1, INIT1_WAIT,
    INIT2, INIT2_WAIT,
    INIT3, INIT3_WAIT,
    WAIT_INT,
    READ_YAWL, WAIT_YAWL,
    READ_YAWH, WAIT_YAWH,
    VALID_YAW
  } state_t;
  state_t state, nxt_state;
  
  // SPI interface to inertial sensor
  SPI_mnrch iSPI(.clk(clk),.rst_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),
                 .MISO(MISO),.MOSI(MOSI),.wrt(wrt),.done(done),
				 .rd_data(inert_data),.wt_data(cmd));
				  
  // Angle engine combines gyro + fusion correction into heading
  inertial_integrator #(FAST_SIM) iINT(.clk(clk), .rst_n(rst_n), .strt_cal(strt_cal),
                        .vld(vld),.rdy(rdy),.cal_done(cal_done), .yaw_rt(yaw_rt),.moving(moving),
						.en_fusion(en_fusion),.IR_Dtrm(IR_Dtrm),.heading(heading));
	

  // The gyro sends low byte first and high byte second
  assign yaw_rt = {yawH, yawL};

  // Reduction AND makes this high when the whole timer is full
  assign init_tmr_full = &init_tmr;

  // The FSM only uses the synchronized version of INT
  assign INT_synched = INT_ff2;

  // State register
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n)
      state <= RST_WAIT;
    else
      state <= nxt_state;

  // Registered datapath for values that must be remembered
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      init_tmr <= 16'h0000;
      INT_ff1 <= 1'b0;
      INT_ff2 <= 1'b0;
      yawL <= 8'h00;
      yawH <= 8'h00;
    end else begin
      // INT is external to this clock domain, so sync it first
      INT_ff1 <= INT;
      INT_ff2 <= INT_ff1;

      // Run the reset wait timer only when the FSM asks for it
      if (clr_init_tmr)
        init_tmr <= 16'h0000;
      else if (en_init_tmr && !init_tmr_full)
        init_tmr <= init_tmr + 1'b1;

      // Capture the low byte after the yawL SPI read finishes
      if (ld_yawL)
        yawL <= inert_data[7:0];

      // Capture the high byte after the yawH SPI read finishes
      if (ld_yawH)
        yawH <= inert_data[7:0];
    end
  end

  // Next-state and output logic.
  always_comb begin
    // Default outputs.
    nxt_state = state;
    wrt = 1'b0;
    vld = 1'b0;
    cmd = 16'h0000;
    clr_init_tmr = 1'b0;
    en_init_tmr = 1'b0;
    ld_yawL = 1'b0;
    ld_yawH = 1'b0;

    case (state)
      RST_WAIT : begin
        // Give the sensor model time to finish its reset
        en_init_tmr = 1'b1;
        if (init_tmr_full)
          nxt_state = INIT1;
      end

      INIT1 : begin
        // Enable data-ready interrupts
        wrt = 1'b1;
        cmd = 16'h0D02;
        nxt_state = INIT1_WAIT;
      end

      INIT1_WAIT : begin
        // Wait until the interrupt setup write is done
        cmd = 16'h0D02;
        if (done)
          nxt_state = INIT2;
      end

      INIT2 : begin
        // Set gyro data rate and range
        wrt = 1'b1;
        cmd = 16'h1160;
        nxt_state = INIT2_WAIT;
      end

      INIT2_WAIT : begin
        // Wait until the gyro setup write is done
        cmd = 16'h1160;
        if (done)
          nxt_state = INIT3;
      end

      INIT3 : begin
        // Enable wraparound reads
        wrt = 1'b1;
        cmd = 16'h1440;
        nxt_state = INIT3_WAIT;
      end

      INIT3_WAIT : begin
        // After this write, the sensor is ready for normal reads
        cmd = 16'h1440;
        if (done)
          nxt_state = WAIT_INT;
      end

      WAIT_INT : begin
        // Wait here until the sensor has a fresh sample
        if (INT_synched)
          nxt_state = READ_YAWL;
      end

      READ_YAWL : begin
        // Read yaw low byte from address 0x26
        wrt = 1'b1;
        cmd = 16'hA600;
        nxt_state = WAIT_YAWL;
      end

      WAIT_YAWL : begin
        // Store yaw low after the SPI read completes
        cmd = 16'hA600;
        if (done) begin
          ld_yawL = 1'b1;
          nxt_state = READ_YAWH;
        end
      end

      READ_YAWH : begin
        // Read yaw high byte from address 0x27
        wrt = 1'b1;
        cmd = 16'hA700;
        nxt_state = WAIT_YAWH;
      end

      WAIT_YAWH : begin
        // Store yaw high after the SPI read completes
        cmd = 16'hA700;
        if (done) begin
          ld_yawH = 1'b1;
          nxt_state = VALID_YAW;
        end
      end

      VALID_YAW : begin
        // Both bytes are ready, so pulse vld for the integrator
        vld = 1'b1;
        nxt_state = WAIT_INT;
      end

      default : begin
        // Recover safely if the FSM ever lands in a bad state
        clr_init_tmr = 1'b1;
        nxt_state = RST_WAIT;
      end
    endcase
  end
endmodule