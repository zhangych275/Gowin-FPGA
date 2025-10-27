// Simple, synthesizable PWM module.
// - period: 16-bit period value (counts). PWM repeats every 'period' clock cycles.
//           If period == 0, output is held low.
// - duty : 16-bit duty numerator (0..65535). Duty fraction = duty / 65536.
//           Effective high time (in counts) = floor(period * duty / 65536).
// - clk, rst_n: synchronous clock and active-low reset.
// - pwm_out: PWM output (1 = high, 0 = low).
//
// Notes:
// - Duty is a 16-bit fractional value; to get percentage, use duty/65536.
// - If you prefer duty expressed as absolute counts (0..period), convert externally
//   before writing into duty. This implementation keeps duty as a fractional 16-bit
//   to maximize resolution regardless of chosen period.
//
// Example usage (matches serial_protocol_converter instantiation):
//   .period     (pwm_config_reg[i][31:16]),
//   .duty       (pwm_config_reg[i][15:0]),
//   .pwm_out    (pwm_out[i])
module pwm_module (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] period,   // period in clock cycles (0 => disabled / output low)
    input  wire [15:0] duty,     // duty fraction numerator (0..65535)
    output reg         pwm_out
);

    // Counter width: enough to hold period (16 bits)
    reg [15:0] cnt;

    // Computed compare threshold: number of clock cycles pwm_out should be high.
    // thresh = floor(period * duty / 65536)
    // Use 32-bit intermediate product.
    wire [31:0] prod = period * {16'd0, duty}; // period(16) * duty(16) -> 32-bit product placed in prod[31:0]
    // But above multiplies 16 by 32 due to concatenation; instead compute directly:
    wire [31:0] mul = ( {16'd0, period} * {16'd0, duty} );
    wire [15:0] thresh = mul[31:16]; // equivalent to floor(period * duty / 2^16)

    // On each clock, update counter and output
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 16'd0;
            pwm_out <= 1'b0;
        end else begin
            if (period == 16'd0) begin
                // Disabled: hold low
                cnt <= 16'd0;
                pwm_out <= 1'b0;
            end else begin
                // increment counter, wrap at period-1
                if (cnt >= period - 1) begin
                    cnt <= 16'd0;
                end else begin
                    cnt <= cnt + 1'b1;
                end

                // Determine output: high when cnt < thresh
                // thresh can be zero (duty==0) -> always low
                // if duty ~ full (65535) thresh approx = period - period/65536 -> nearly period
                if (thresh == 16'd0) begin
                    pwm_out <= 1'b0;
                end else begin
                    pwm_out <= (cnt < thresh) ? 1'b1 : 1'b0;
                end
            end
        end
    end

endmodule