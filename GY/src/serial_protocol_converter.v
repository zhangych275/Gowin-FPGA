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
    
    // PWM输出 (wire，由子模块驱动)
    output [7:0]        pwm_out,
    
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
wire [31:0]  i2c_rx_reg;      // 从 i2c_controller 驱动 -> 使用 wire
reg [31:0]   uart_config_reg; // UART配置寄存器
reg [31:0]   uart_tx_reg;     // UART发送寄存器
wire [31:0]  uart_rx_reg;     // 从 uart_controller 驱动 -> 使用 wire
reg [31:0]   pwm_config_reg[7:0]; // PWM配置寄存器(8路)

// 从子模块输出的 handshake 信号 (wire)
wire         i2c_done;
wire         uart_tx_done;
wire         uart_rx_ready;

// 状态机定义
localparam IDLE   = 3'd0;
localparam SPI_TX = 3'd1;
localparam SPI_RX = 3'd2;
localparam I2C_TX = 3'd3;
localparam I2C_RX = 3'd4;
localparam UART_TX= 3'd5;
localparam UART_RX= 3'd6;

reg [2:0] current_state, next_state;

// -------------------------------
// 命令解析与寄存器读写
// -------------------------------
integer idx;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 初始化所有寄存器
        spi_config_reg <= 32'd0;
        spi_tx_reg <= 32'd0;
        i2c_config_reg <= 32'd0;
        i2c_tx_reg <= 32'd0;
        uart_config_reg <= 32'd0;
        uart_tx_reg <= 32'd0;
        
        for (idx = 0; idx < 8; idx = idx + 1) begin
            pwm_config_reg[idx] <= 32'd0;
        end

        tx_data <= 32'd0;
        tx_en <= 1'b0;
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
                    default:  tx_data <= 32'd0;
                endcase
                tx_en <= 1'b1;
            end
            default: ;
        endcase
    end else if (tx_done) begin
        tx_en <= 1'b0;
    end
end

// -------------------------------
// SPI 控制逻辑（只操作 spi_* 寄存器，不修改 current_state）
// -------------------------------
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
        // NOTE: 不在这里写 current_state
    end else begin
        case (current_state)
            IDLE: begin
                spi_sclk <= 1'b0;
                // 当状态机处于 IDLE，若接收到 SPI 发送命令，由状态转移逻辑改变 next_state 到 SPI_TX
                // 这里仅在启动发送的周期加载寄存器，避免多驱动 current_state
                if (cmd_valid && cmd_opcode == 8'h02) begin
                    spi_cs <= ~(1 << (cmd_addr[1:0])); // 选择对应的SPI设备 (低有效)
                    spi_shift_reg <= spi_tx_reg;
                    spi_bit_cnt <= 6'd31;
                end
            end
            
            SPI_TX: begin
                // SPI 发送：产生时钟并移位
                spi_sclk <= ~spi_sclk;
                if (spi_sclk) begin
                    spi_mosi <= spi_shift_reg[31];
                    spi_shift_reg <= {spi_shift_reg[30:0], spi_miso};
                    if (spi_bit_cnt == 0) begin
                        // 发送完成，释放片选并保存接收数据
                        spi_cs <= 4'hF;
                        spi_rx_reg <= {spi_shift_reg[30:0], spi_miso};
                        spi_bit_cnt <= 6'd0;
                        // current_state 的改变由 state-register 块和 next_state 的组合逻辑完成
                    end else begin
                        spi_bit_cnt <= spi_bit_cnt - 1'b1;
                    end
                end
            end
            
            default: begin
                // 其余状态下保持 SPI 信号为默认值
                spi_sclk <= 1'b0;
                spi_mosi <= 1'b0;
                spi_cs <= 4'hF;
            end
        endcase
    end
end

// -------------------------------
// I2C / UART 子模块实例化（保持原样，输出为 wire）
// -------------------------------
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

// -------------------------------
// PWM 发生器：每路 pwm_module 驱动 pwm_out[i]（pwm_out 是 wire）
// -------------------------------
generate
    genvar j;
    for (j = 0; j < 8; j = j + 1) begin : pwm_generator
        pwm_module pwm_module_inst (
            .clk        (clk),
            .rst_n      (rst_n),
            .period     (pwm_config_reg[j][31:16]),
            .duty       (pwm_config_reg[j][15:0]),
            .pwm_out    (pwm_out[j])
        );
    end
endgenerate

// -------------------------------
// 状态机的组合 next_state 逻辑（保持原样）
// -------------------------------
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
            // 当 SPI 发送完成：spi_bit_cnt == 0 且 spi_sclk (上升沿后的评估) 时返回 IDLE
            if (spi_bit_cnt == 0 && spi_sclk)
                next_state = IDLE;
            else
                next_state = SPI_TX;
        end
        
        // 其余状态可按需扩展
        default: next_state = IDLE;
    endcase
end

// -------------------------------
// 状态寄存器（唯一驱动 current_state 的时序块）
// -------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        current_state <= IDLE;
    else
        current_state <= next_state;
end

endmodule

/*module serial_protocol_converter (
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
wire [31:0]  i2c_rx_reg;      // 从 i2c_controller 驱动 -> 使用 wire
reg [31:0]   uart_config_reg; // UART配置寄存器
reg [31:0]   uart_tx_reg;     // UART发送寄存器
wire [31:0]  uart_rx_reg;     // 从 uart_controller 驱动 -> 使用 wire
reg [31:0]   pwm_config_reg[7:0]; // PWM配置寄存器(8路)

// 从子模块输出的 handshake 信号 (wire)
wire         i2c_done;
wire         uart_tx_done;
wire         uart_rx_ready;

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
integer i;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 初始化所有寄存器
        spi_config_reg <= 32'd0;
        spi_tx_reg <= 32'd0;
        i2c_config_reg <= 32'd0;
        i2c_tx_reg <= 32'd0;
        uart_config_reg <= 32'd0;
        uart_tx_reg <= 32'd0;
        
        for (i = 0; i < 8; i = i + 1) begin
            pwm_config_reg[i] <= 32'd0;
        end

        tx_data <= 32'd0;
        tx_en <= 1'b0;
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
        current_state <= IDLE;
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
                // 简化的 SPI 发送逻辑（仿真/示例用）
                spi_sclk <= ~spi_sclk;
                if (spi_sclk) begin
                    // 时钟上升/下降边沿切换数据（此处为示例）
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
            
            default: current_state <= IDLE;
        endcase
    end
end

// I2C控制器实例化（i2c_rx_reg 是 wire，由子模块驱动）
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

// UART控制器实例化（uart_rx_reg 是 wire，由子模块驱动）
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
    for (genvar j = 0; j < 8; j = j + 1) begin : pwm_generator
        // pwm_module must exist in project; if not, provide a stub separately.
        pwm_module pwm_module_inst (
            .clk        (clk),
            .rst_n      (rst_n),
            .period     (pwm_config_reg[j][31:16]),
            .duty       (pwm_config_reg[j][15:0]),
            .pwm_out    (pwm_out[j])
        );
    end
endgenerate

// 状态机转换逻辑（简化）
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



/*module serial_protocol_converter (
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
wire [31:0]  i2c_rx_reg;      // 从 i2c_controller 驱动 -> 使用 wire
reg [31:0]   uart_config_reg; // UART配置寄存器
reg [31:0]   uart_tx_reg;     // UART发送寄存器
wire [31:0]  uart_rx_reg;     // 从 uart_controller 驱动 -> 使用 wire
reg [31:0]   pwm_config_reg[7:0]; // PWM配置寄存器(8路)

// 从子模块输出的 handshake 信号 (wire)
wire         i2c_done;
wire         uart_tx_done;
wire         uart_rx_ready;

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
integer i;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 初始化所有寄存器
        spi_config_reg <= 32'd0;
        spi_tx_reg <= 32'd0;
        i2c_config_reg <= 32'd0;
        i2c_tx_reg <= 32'd0;
        uart_config_reg <= 32'd0;
        uart_tx_reg <= 32'd0;
        
        for (i = 0; i < 8; i = i + 1) begin
            pwm_config_reg[i] <= 32'd0;
        end

        tx_data <= 32'd0;
        tx_en <= 1'b0;
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
        current_state <= IDLE;
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
                // 简化的 SPI 发送逻辑（仿真/示例用）
                spi_sclk <= ~spi_sclk;
                if (spi_sclk) begin
                    // 时钟上升/下降边沿切换数据（此处为示例）
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
            
            default: current_state <= IDLE;
        endcase
    end
end

// I2C控制器实例化（i2c_rx_reg 是 wire，由子模块驱动）
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

// UART控制器实例化（uart_rx_reg 是 wire，由子模块驱动）
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
    for (genvar j = 0; j < 8; j = j + 1) begin : pwm_generator
        pwm_module pwm_module_inst (
            .clk        (clk),
            .rst_n      (rst_n),
            .period     (pwm_config_reg[j][31:16]),
            .duty       (pwm_config_reg[j][15:0]),
            .pwm_out    (pwm_out[j])
        );
    end
endgenerate

// 状态机转换逻辑（简化）
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


/*module serial_protocol_converter (
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
integer i;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 初始化所有寄存器
        spi_config_reg <= 32'd0;
        spi_tx_reg <= 32'd0;
        i2c_config_reg <= 32'd0;
        i2c_tx_reg <= 32'd0;
        uart_config_reg <= 32'd0;
        uart_tx_reg <= 32'd0;
        
        for (i = 0; i < 8; i = i + 1) begin
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

endmodule*/


