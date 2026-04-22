// PWM12.sv
// 12-bit PWM generator with non-overlap between complementary outputs.
// Used by MtrDrv for H-bridge motor drive timing.
module PWM12(
 input clk, // 50 MHz system clock
 input rst_n, // asynchronous active low reset for system
 input [11:0] duty, // unsigned 12 bit value for duty cycle
 output PWM1, // PWM signal, does not overlap w/ PWM2
 output PWM2 // PWM signal, does not over lap w/ PWM1
);

    localparam NONOVERLAP = 12'h02C; // overlap time (44)
    reg [11:0] cnt; // 12 bit unsigned counter for PWM generation, 4095 max value
    reg pwm1_reg, pwm2_reg; // registers for PWM outputs

    // countinous counter for PWM generation
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 12'b0; // reset counter to 0
        end 
        else begin
            cnt <= cnt + 1; // increment counter on each clock cycle
        end
    end

    //PWM1 generation logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm1_reg <= 1'b0; // reset PWM1 to 0
        end 
        else if (duty <= NONOVERLAP) begin
            pwm1_reg <= 1'b0; // duty cycle is 0 bc duty <= non-overlap time
        end
        else if (cnt >= duty) begin
            pwm1_reg <= 1'b0; // reset PWM1 when counter is >= duty cycle value
        end
        else if (cnt >= NONOVERLAP) begin
            pwm1_reg <= 1'b1; // set PWM1 high if counter is >= non-overlap time
        end 
    end

    // PWM2 generation logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm2_reg <= 1'b0; // reset PWM2 to 0
        end 
        else if (duty <= NONOVERLAP) begin
            pwm2_reg <= 1'b0; // duty cycle is 0 bc duty <= non-overlap time
        end
        else if (duty > (12'hFFF - NONOVERLAP)) begin // overflow case
            pwm2_reg <= 1'b0; // duty too high, PWM2 stays off
        end
        else if (&cnt) begin
            pwm2_reg <= 1'b0; // reset PWM2 when counter is at max value
        end
        else if (cnt >= (duty + NONOVERLAP)) begin
            pwm2_reg <= 1'b1; // set PWM2 high if counter is >= duty cycle value + non-overlap time
        end
    end

    assign PWM1 = pwm1_reg; // assign PWM1 output to register value
    assign PWM2 = pwm2_reg; // assign PWM2 output to register value


endmodule
