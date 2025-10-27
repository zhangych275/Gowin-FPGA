// Simple stub I2C controller for simulation purposes.
// Ports are compatible with serial_protocol_converter.v instantiation:
// - start: pulse to start a transaction
// - done: asserted when transaction completes
// - tx_data: data to be sent
// - rx_data: received data (here we just echo tx_data after delay)
// SDA/SCL are modeled as inouts but kept tri-stated (no actual timing)
module i2c_controller (
    input               clk,
    input               rst_n,
    input  [31:0]       config_reg,
    input  [31:0]       tx_data,
    output reg [31:0]   rx_data,
    input               start,
    output reg          done,
    inout               sda,
    inout               scl
);

// Simple internal state machine that asserts done N cycles after start
localparam IDLE = 2'd0;
localparam BUSY = 2'd1;
localparam DONE = 2'd2;

reg [1:0] state;
reg [15:0] counter;

// Keep SDA/SCL tri-stated (no actual I2C signals)
wire sda_i = 1'bz;
wire scl_i = 1'bz;
assign sda = 1'bz;
assign scl = 1'bz;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        counter <= 16'd0;
        done <= 1'b0;
        rx_data <= 32'd0;
    end else begin
        case (state)
            IDLE: begin
                done <= 1'b0;
                if (start) begin
                    state <= BUSY;
                    counter <= 16'd0;
                end
            end
            BUSY: begin
                counter <= counter + 1'b1;
                if (counter == 16'd200) begin
                    // simulate that transaction finished
                    rx_data <= tx_data; // echo back transmitted data (for test)
                    done <= 1'b1;
                    state <= DONE;
                end
            end
            DONE: begin
                // keep done asserted one cycle, then return to IDLE
                done <= 1'b0;
                state <= IDLE;
            end
            default: state <= IDLE;
        endcase
    end
end

endmodule