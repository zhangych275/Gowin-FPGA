// Simple UART controller stub for simulation.
// - Supports basic byte TX and RX using clk-driven baud generator.
// - tx_start: pulse to start transmitting tx_data[7:0] (LSB first).
// - tx_done: pulsed when transmission finishes (one clk).
// - rx_data: holds received byte in lower 8 bits (zero-extended to 32).
// - rx_ready: pulsed when a byte has been received.
// - config_reg[15:0] can be used to override baud divisor (if non-zero).
module uart_controller (
    input               clk,
    input               rst_n,
    input  [31:0]       config_reg,
    input  [31:0]       tx_data,
    output reg [31:0]   rx_data,
    input               tx_start,
    output reg          tx_done,
    output reg          rx_ready,
    output reg          tx,       // UART TX output (driven by this module)
    input               rx        // UART RX input (from external)
);

parameter DEFAULT_BAUD_DIV = 434; // approximate for 50MHz / 115200 ~ 434

// Baud divider selection (if config_reg[15:0] == 0 use default)
wire [15:0] baud_div_w = (config_reg[15:0] == 16'd0) ? DEFAULT_BAUD_DIV[15:0] : config_reg[15:0];

// TX state
reg [15:0] baud_cnt_tx;
reg [3:0]  tx_bit_idx;
reg [9:0]  tx_shift; // start(0), 8 data bits LSB first, stop(1) => 10 bits
reg        tx_active;

// RX state
reg [15:0] baud_cnt_rx;
reg [3:0]  rx_bit_idx;
reg [7:0]  rx_shift;
reg        rx_active;
reg        rx_sync0, rx_sync1; // simple sync for rx input
reg        rx_sample_en;
reg [15:0] rx_sample_cnt;

// Synchronize RX input to clk domain
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_sync0 <= 1'b1;
        rx_sync1 <= 1'b1;
    end else begin
        rx_sync0 <= rx;
        rx_sync1 <= rx_sync0;
    end
end

// TX logic: sample tx_start edge, load shift register and send bits at baud
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        baud_cnt_tx <= 16'd0;
        tx_bit_idx <= 4'd0;
        tx_shift <= 10'b1111111111;
        tx_active <= 1'b0;
        tx <= 1'b1; // idle high
        tx_done <= 1'b0;
    end else begin
        tx_done <= 1'b0;
        if (!tx_active) begin
            // waiting for start
            if (tx_start) begin
                // prepare frame: {stop(1), data[7:0], start(0)}
                tx_shift <= {1'b1, tx_data[7:0], 1'b0};
                tx_bit_idx <= 4'd0;
                baud_cnt_tx <= 16'd0;
                tx_active <= 1'b1;
                tx <= 1'b0; // start bit immediately represented when baud counter reaches 0
            end else begin
                tx <= 1'b1;
            end
        end else begin
            // active sending: increment baud counter, shift when divider reached
            if (baud_cnt_tx >= baud_div_w - 1) begin
                baud_cnt_tx <= 16'd0;
                // output current bit (LSB of tx_shift)
                tx <= tx_shift[0];
                tx_shift <= {1'b1, tx_shift[9:1]}; // shift in stop(1) at MSB side
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 4'd9) begin
                    // finished sending 10 bits
                    tx_active <= 1'b0;
                    tx_done <= 1'b1; // pulse
                    tx <= 1'b1; // idle
                end
            end else begin
                baud_cnt_tx <= baud_cnt_tx + 1'b1;
            end
        end
    end
end

// RX logic: detect start bit (falling edge), then sample in middle of bit windows
localparam RX_STATE_IDLE = 2'd0;
localparam RX_STATE_START = 2'd1;
localparam RX_STATE_DATA = 2'd2;
localparam RX_STATE_STOP = 2'd3;

reg [1:0] rx_state;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state <= RX_STATE_IDLE;
        baud_cnt_rx <= 16'd0;
        rx_bit_idx <= 4'd0;
        rx_shift <= 8'd0;
        rx_active <= 1'b0;
        rx_ready <= 1'b0;
    end else begin
        rx_ready <= 1'b0;
        case (rx_state)
            RX_STATE_IDLE: begin
                // wait for start bit (detect rx_sync1 falling)
                if (rx_sync1 == 1'b0) begin
                    // start detected, wait half bit to sample center
                    baud_cnt_rx <= (baud_div_w >> 1);
                    rx_state <= RX_STATE_START;
                end
            end
            RX_STATE_START: begin
                if (baud_cnt_rx == 16'd0) begin
                    // sample start bit center
                    if (rx_sync1 == 1'b0) begin
                        // valid start, proceed to data bits
                        rx_bit_idx <= 4'd0;
                        baud_cnt_rx <= baud_div_w - 1;
                        rx_state <= RX_STATE_DATA;
                    end else begin
                        // false start, return to idle
                        rx_state <= RX_STATE_IDLE;
                    end
                end else begin
                    baud_cnt_rx <= baud_cnt_rx - 1'b1;
                end
            end
            RX_STATE_DATA: begin
                if (baud_cnt_rx == 16'd0) begin
                    // sample data bit
                    rx_shift <= {rx_sync1, rx_shift[7:1]}; // LSB first: shift right-in at MSB, will reverse later
                    rx_bit_idx <= rx_bit_idx + 1'b1;
                    baud_cnt_rx <= baud_div_w - 1;
                    if (rx_bit_idx == 4'd7) begin
                        rx_state <= RX_STATE_STOP;
                    end
                end else begin
                    baud_cnt_rx <= baud_cnt_rx - 1'b1;
                end
            end
            RX_STATE_STOP: begin
                if (baud_cnt_rx == 16'd0) begin
                    // sample stop bit center
                    // For simplicity accept any stop bit
                    rx_data <= {24'd0, rx_shift}; // present received byte in lower 8 bits
                    rx_ready <= 1'b1; // pulse rx_ready one clk
                    rx_state <= RX_STATE_IDLE;
                end else begin
                    baud_cnt_rx <= baud_cnt_rx - 1'b1;
                end
            end
            default: rx_state <= RX_STATE_IDLE;
        endcase
    end
end

endmodule