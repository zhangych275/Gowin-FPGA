module serial_protocol_converter (
    input               clk,
    input               rst_n,
    
    // 命令接口
    input [7:0]         cmd_opcode,
    input [15:0]        cmd_addr,
    input [31:0]        cmd_data,
    input               cmd_valid,
    
    // SPI接口
    output reg          spi_sclk,
    output reg          spi_mosi,
    input               spi_miso,
    output reg [3:0]    spi_cs,
    
    // I2C接口
    inout               i2c_sda,
    inout               i2c_scl,
    
    // UART接口
    input               uart_rx,
    output              uart_tx,
    
    // PWM输出
    output reg [7:0]    pwm_out,
    
    // 发送数据接口
    output reg [31:0]   tx_data,
    output reg          tx_en,
    input               tx_done
);

// 寄存器定义
reg [31:0]   spi_config_reg;  // SPI配置寄存器
reg [31:0]   spi_tx_reg;      // SPI发送寄存器
reg [31:0]   spi_rx_reg;      // SPI接收寄存器
reg [31:0]   i2c_config_reg;  // I2C配置寄存器
reg [31:0]   i2c_tx_reg;      // I2C发送寄存器
reg [31:0]   i2c_rx_reg;      // I2C接收寄存器
reg [31:0]   uart_config_reg; // UART配置寄存器
reg [31:0]   uart_tx_reg;     // UART发送寄存器
reg [31:0]   uart_rx_reg;     // UART接收寄存器
reg [31:0]   pwm_config_reg[7:0]; // PWM配置寄存器(8路)

// 状态机定义
localparam IDLE = 3'd0;
localparam SPI_TX = 3'd1;
localparam SPI_RX = 3'd2;
localparam I2C_TX = 3'd3;
localparam I2C_RX = 3'd4;
localparam UART_TX = 3'd5;
localparam UART_RX = 3'd6;

reg [2:0] current_state, next_state;

// 命令解析与寄存器读写
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 初始化所有寄存器
        spi_config_reg <= 32'd0;
        spi_tx_reg <= 32'd0;
        i2c_config_reg <= 32'd0;
        i2c_tx_reg <= 32'd0;
        uart_config_reg <= 32'd0;
        uart_tx_reg <= 32'd0;

        for (integer i = 0; i < 8; i = i + 1) begin
            pwm_config_reg[i] <= 32'd0;
        end
    end else if (cmd_valid) begin
        case (cmd_opcode)
            // SPI配置
            8'h01: spi_config_reg <= cmd_data;
            // SPI发送数据
            8'h02: spi_tx_reg <= cmd_data;
            // I2C配置
            8'h03: i2c_config_reg <= cmd_data;
            // I2C发送数据
            8'h04: i2c_tx_reg <= cmd_data;
            // UART配置
            8'h05: uart_config_reg <= cmd_data;
            // UART发送数据
            8'h06: uart_tx_reg <= cmd_data;
            // PWM配置
            8'h07: begin
                if (cmd_addr < 8) begin
                    pwm_config_reg[cmd_addr] <= cmd_data;
                end
            end
            // 读取接收数据
            8'h08: begin
                case (cmd_addr)
                    16'h0001: tx_data <= spi_rx_reg;
                    16'h0002: tx_data <= i2c_rx_reg;
                    16'h0003: tx_data <= uart_rx_reg;
                endcase
                tx_en <= 1'b1;
            end
        endcase
    end else if (tx_done) begin
        tx_en <= 1'b0;
    end
end

// SPI控制器实现
reg [5:0] spi_bit_cnt;
reg [31:0] spi_shift_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        spi_sclk <= 1'b0;
        spi_mosi <= 1'b0;
        spi_cs <= 4'hF;
        spi_bit_cnt <= 6'd0;
        spi_shift_reg <= 32'd0;
        spi_rx_reg <= 32'd0;
    end else begin
        case (current_state)
            IDLE: begin
                spi_sclk <= 1'b0;
                if (cmd_valid && cmd_opcode == 8'h02) begin
                    // 启动SPI发送
                    spi_cs <= ~(1 << (cmd_addr[1:0])); // 选择对应的SPI设备
                    spi_shift_reg <= spi_tx_reg;
                    spi_bit_cnt <= 6'd31;
                    current_state <= SPI_TX;
                end
            end
            
            SPI_TX: begin
                // 实现SPI发送逻辑，包括时钟生成和数据移位
                spi_sclk <= ~spi_sclk;
                if (spi_sclk) begin
                    // 时钟下降沿，更新数据
                    spi_mosi <= spi_shift_reg[31];
                    spi_shift_reg <= {spi_shift_reg[30:0], spi_miso};
                    if (spi_bit_cnt == 0) begin
                        current_state <= IDLE;
                        spi_cs <= 4'hF;
                        spi_rx_reg <= {spi_shift_reg[30:0], spi_miso};
                    end else begin
                        spi_bit_cnt <= spi_bit_cnt - 1'b1;
                    end
                end
            end
            
            // 其他状态实现...
            default: current_state <= IDLE;
        endcase
    end
end

// I2C控制器实例化
i2c_controller i2c_controller_inst (
    .clk        (clk),
    .rst_n      (rst_n),
    .config_reg (i2c_config_reg),
    .tx_data    (i2c_tx_reg),
    .rx_data    (i2c_rx_reg),
    .start      (cmd_valid && cmd_opcode == 8'h04),
    .done       (i2c_done),
    .sda        (i2c_sda),
    .scl        (i2c_scl)
);

// UART控制器实例化
uart_controller uart_controller_inst (
    .clk        (clk),
    .rst_n      (rst_n),
    .config_reg (uart_config_reg),
    .tx_data    (uart_tx_reg),
    .rx_data    (uart_rx_reg),
    .tx_start   (cmd_valid && cmd_opcode == 8'h06),
    .tx_done    (uart_tx_done),
    .rx_ready   (uart_rx_ready),
    .tx         (uart_tx),
    .rx         (uart_rx)
);

// PWM发生器
generate
    for (genvar i = 0; i < 8; i = i + 1) begin : pwm_generator
        pwm_module pwm_module_inst (
            .clk        (clk),
            .rst_n      (rst_n),
            .period     (pwm_config_reg[i][31:16]),
            .duty       (pwm_config_reg[i][15:0]),
            .pwm_out    (pwm_out[i])
        );
    end
endgenerate

// 状态机转换逻辑
always @(*) begin
    case (current_state)
        IDLE: begin
            if (cmd_valid && cmd_opcode == 8'h02)
                next_state = SPI_TX;
            else if (cmd_valid && cmd_opcode == 8'h04)
                next_state = I2C_TX;
            else if (cmd_valid && cmd_opcode == 8'h06)
                next_state = UART_TX;
            else
                next_state = IDLE;
        end
        
        SPI_TX: begin
            if (spi_bit_cnt == 0 && spi_sclk)
                next_state = IDLE;
            else
                next_state = SPI_TX;
        end
        
        // 其他状态转换逻辑...
        default: next_state = IDLE;
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        current_state <= IDLE;
    else
        current_state <= next_state;
end

endmodule
