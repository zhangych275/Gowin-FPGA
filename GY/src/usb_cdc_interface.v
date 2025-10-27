module usb_cdc_interface #(
    parameter PWM_CHANNELS = 4,       // PWM 通道数量
    parameter SPI_MAX_CLK_DIV = 16,   // SPI 最大分频系数
    parameter I2C_MAX_CLK_DIV = 64    // I2C 最大分频系数
)(
    // 全局时钟与复位
    input  wire        clk,           // 系统时钟
    input  wire        rst_n,         // 异步复位（低有效）

    // USB CDC 接口（与 USB 控制器对接）
    input  wire [7:0]  cdc_rx_data,   // CDC 接收数据
    input  wire        cdc_rx_valid,  // CDC 接收数据有效
    output reg  [7:0]  cdc_tx_data,   // CDC 发送数据
    output reg         cdc_tx_valid,  // CDC 发送数据有效
    input  wire        cdc_tx_ready,  // CDC 发送就绪

    // SPI 接口
    output reg         spi_sclk,      // SPI 时钟
    output reg         spi_mosi,      // SPI 主机输出
    input  wire        spi_miso,      // SPI 主机输入 (may be unused in this simplified driver)
    output reg  [3:0]  spi_cs_n,      // SPI 片选（低有效，4路）

    // I2C 接口
    inout  wire        i2c_sda,       // I2C 数据
    inout  wire        i2c_scl,       // I2C 时钟

    // UART 接口
    output reg         uart_tx,       // UART 发送
    input  wire        uart_rx,       // UART 接收 (may be unused in this simplified driver)
    output reg         uart_rts,      // UART 发送请求
    input  wire        uart_cts,      // UART 清除发送 (may be unused)

    // PWM 输出
    output reg [PWM_CHANNELS-1:0] pwm_out  // PWM 输出
);

// Workarounds for "unused input" warnings: refer to inputs so tools see them used.
wire _unused_spi_miso = spi_miso;
wire _unused_uart_rx  = uart_rx;
wire _unused_uart_cts = uart_cts;

// --------------------------
// 1. 命令帧格式定义
// --------------------------
localparam FRAME_HEADER = 8'hAA;
localparam CMD_SPI_CFG  = 8'h01;
localparam CMD_SPI_TX   = 8'h02;
localparam CMD_I2C_CFG  = 8'h03;
localparam CMD_I2C_TX   = 8'h04;
localparam CMD_UART_CFG = 8'h05;
localparam CMD_UART_TX  = 8'h06;
localparam CMD_PWM_CFG  = 8'h07;
localparam CMD_STATUS   = 8'h08;

// --------------------------
// 2. 状态机定义
// --------------------------
localparam S_IDLE      = 4'd0;  // 空闲态，等待帧头
localparam S_CMD       = 4'd1;  // 接收命令码
localparam S_PARA_LEN  = 4'd2;  // 接收参数长度
localparam S_PARA      = 4'd3;  // 接收参数
localparam S_CHECKSUM  = 4'd4;  // 接收校验和
localparam S_EXECUTE   = 4'd5;  // 执行命令
localparam S_RESPONSE  = 4'd6;  // 发送响应

// --------------------------
// 3. 寄存器定义
// --------------------------
reg [3:0] current_state, next_state;
reg [7:0] cmd_reg;              // 命令寄存器
reg [7:0] para_len_reg;         // 参数长度寄存器
reg [7:0] para_buf [0:63];      // 参数缓冲区（最大64字节）
reg [5:0] para_cnt;             // 参数计数
reg [7:0] checksum_reg;         // 校验和寄存器
reg [7:0] response_buf [0:31];  // 响应缓冲区
reg [5:0] response_len;         // 响应长度
reg [5:0] response_cnt;         // 响应计数

// 外设配置寄存器
reg [3:0] spi_clk_div;          // SPI 分频（2^(div+1)）
reg       spi_cpol;             // SPI 时钟极性
reg       spi_cpha;             // SPI 时钟相位
reg [15:0] i2c_clk_div;         // I2C 分频
reg [15:0] i2c_timeout;         // I2C 超时计数
reg [15:0] uart_br_div;         // UART 波特率分频
reg [1:0] uart_data_bits;       // UART 数据位（00=8, 01=7, 10=6, 11=5）
reg       uart_stop_bits;       // UART 停止位（0=1位, 1=2位）
reg [1:0] uart_parity;          // UART 校验（00=无, 01=奇, 10=偶）
reg [15:0] pwm_freq [PWM_CHANNELS-1:0]; // PWM 频率
reg [7:0]  pwm_duty [PWM_CHANNELS-1:0]; // PWM 占空比（0-100）

// I2C tri-state drivers (regs must be driven)
reg i2c_sda_out, i2c_scl_out;
reg i2c_sda_en, i2c_scl_en;
assign i2c_sda = i2c_sda_en ? i2c_sda_out : 1'bz;
assign i2c_scl = i2c_scl_en ? i2c_scl_out : 1'bz;

// loop variables declared at module scope (do not declare inside procedural blocks)
integer i;
integer p;

// --------------------------
// 4. 状态机跳转逻辑
// --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) current_state <= S_IDLE;
    else current_state <= next_state;
end

always @(*) begin
    case (current_state)
        S_IDLE:      next_state = (cdc_rx_valid && cdc_rx_data == FRAME_HEADER) ? S_CMD : S_IDLE;
        S_CMD:       next_state = cdc_rx_valid ? S_PARA_LEN : S_CMD;
        S_PARA_LEN:  next_state = cdc_rx_valid ? S_PARA : S_PARA_LEN;
        S_PARA:      next_state = (cdc_rx_valid && para_cnt == para_len_reg - 1) ? S_CHECKSUM : S_PARA;
        S_CHECKSUM:  next_state = cdc_rx_valid ? S_EXECUTE : S_CHECKSUM;
        S_EXECUTE:   next_state = S_RESPONSE;
        S_RESPONSE:  next_state = (response_cnt == response_len) ? S_IDLE : S_RESPONSE;
        default:     next_state = S_IDLE;
    endcase
end

// --------------------------
// 5. 命令接收与解析
// --------------------------
// 接收命令码
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cmd_reg <= 8'h00;
    else if (current_state == S_CMD && cdc_rx_valid) cmd_reg <= cdc_rx_data;
end

// 接收参数长度
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) para_len_reg <= 8'h00;
    else if (current_state == S_PARA_LEN && cdc_rx_valid) para_len_reg <= cdc_rx_data;
end

// 接收参数与校验和计算
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        para_cnt <= 6'd0;
        checksum_reg <= 8'h00;
        // 初始化 para_buf
        for (p = 0; p < 64; p = p + 1) para_buf[p] <= 8'h00;
    end else if (current_state == S_CMD && cdc_rx_valid) begin
        checksum_reg <= FRAME_HEADER ^ cdc_rx_data;
        para_cnt <= 6'd0;
    end else if (current_state == S_PARA_LEN && cdc_rx_valid) begin
        checksum_reg <= checksum_reg ^ cdc_rx_data;
        para_cnt <= 6'd0;
    end else if (current_state == S_PARA && cdc_rx_valid) begin
        para_buf[para_cnt] <= cdc_rx_data;
        checksum_reg <= checksum_reg ^ cdc_rx_data;
        para_cnt <= para_cnt + 1'b1;
    end else if (current_state == S_EXECUTE) begin
        para_cnt <= 6'd0;
    end
end

// --------------------------
// 6. 命令执行逻辑
// --------------------------
// NOTE: keep peripheral register initializations here, but DO NOT assign
// i2c_sda_en/i2c_scl_en/uart_tx/uart_rts here to avoid multiple drivers.
// Those signals are initialized and driven in their dedicated always blocks.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        spi_clk_div <= 4'd0;
        spi_cpol <= 1'b0;
        spi_cpha <= 1'b0;
        i2c_clk_div <= 16'd0;
        i2c_timeout <= 16'd1000;
        uart_br_div <= 16'd100;
        uart_data_bits <= 2'd0;
        uart_stop_bits <= 1'b0;
        uart_parity <= 2'd0;
        for (i = 0; i < PWM_CHANNELS; i = i + 1) begin
            pwm_freq[i] <= 16'd1000;
            pwm_duty[i] <= 8'd50;
        end
        response_len <= 6'd0;
        // 初始化 response_buf 数组，避免未驱动位警告
        for (i = 0; i < 32; i = i + 1) response_buf[i] <= 8'h00;
        // (Removed duplicated initializations for i2c and uart to avoid multi-drive)
    end else if (current_state == S_EXECUTE) begin
        // 校验和正确则执行命令
        if (checksum_reg == para_buf[para_len_reg]) begin
            case (cmd_reg)
                CMD_SPI_CFG: begin
                    // SPI配置：[分频(4b)][CPOL(1b)][CPHA(1b)][保留(2b)]
                    spi_clk_div <= para_buf[0][7:4];
                    spi_cpol <= para_buf[0][3];
                    spi_cpha <= para_buf[0][2];
                    response_buf[0] <= 8'h00;
                    response_len <= 6'd1;
                end
                CMD_PWM_CFG: begin
                    // PWM配置：[通道(4b)][频率(16b)][占空比(8b)]
                    if (para_buf[0] < PWM_CHANNELS) begin
                        pwm_freq[para_buf[0]] <= {para_buf[1], para_buf[2]};
                        pwm_duty[para_buf[0]] <= para_buf[3];
                        response_buf[0] <= 8'h00;
                    end else begin
                        response_buf[0] <= 8'hFF;
                    end
                    response_len <= 6'd1;
                end
                // I2C/TX/UART/STATUS 等命令可以在此处扩展
                default: begin
                    response_buf[0] <= 8'hFE;
                    response_len <= 6'd1;
                end
            endcase
        end else begin
            response_buf[0] <= 8'hFD;
            response_len <= 6'd1;
        end
    end
end

// --------------------------
// 7. 响应发送逻辑
// --------------------------
// response_cnt is the single driver for that signal
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cdc_tx_data <= 8'h00;
        cdc_tx_valid <= 1'b0;
        response_cnt <= 6'd0;
    end else if (current_state == S_RESPONSE) begin
        if (cdc_tx_ready) begin
            if (response_cnt < response_len) begin
                cdc_tx_data <= response_buf[response_cnt];
                cdc_tx_valid <= 1'b1;
                response_cnt <= response_cnt + 1'b1;
            end else begin
                cdc_tx_valid <= 1'b0;
                response_cnt <= 6'd0;
            end
        end else begin
            cdc_tx_valid <= 1'b0;
        end
    end else begin
        cdc_tx_valid <= 1'b0;
        response_cnt <= 6'd0;
    end
end

// --------------------------
// 8. 外设驱动实例化（简化）
// --------------------------
// SPI 驱动 - 保持简化实现，同时驱动 UART 默认线（避免无驱动警告）
// This always block is the single driver for uart_tx and uart_rts.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        spi_sclk <= 1'b0;
        spi_mosi <= 1'b0;
        spi_cs_n <= 4'hF;
        uart_tx <= 1'b1;
        uart_rts <= 1'b0;
    end else begin
        spi_sclk <= spi_cpol;
        // uart_tx / uart_rts 保持默认空闲值（可在 UART 逻辑扩展时驱动）
        uart_tx <= uart_tx;
        uart_rts <= uart_rts;
    end
end

// I2C tri-state signals: keep default idle values if no I2C implementation present
// This always block is the single driver for i2c_sda_en/i2c_scl_en and their outputs.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        i2c_sda_out <= 1'b1;
        i2c_scl_out <= 1'b1;
        i2c_sda_en  <= 1'b0;
        i2c_scl_en  <= 1'b0;
    end else begin
        // no active I2C master implementation in this simplified module;
        // keep outputs tri-stated / idle. If adding I2C logic later, drive these regs.
        i2c_sda_out <= i2c_sda_out;
        i2c_scl_out <= i2c_scl_out;
        i2c_sda_en  <= 1'b0;
        i2c_scl_en  <= 1'b0;
    end
end

// --------------------------
// PWM 生成（使用标准 Verilog 索引写法）
// --------------------------
genvar gi;
generate
    for (gi = 0; gi < PWM_CHANNELS; gi = gi + 1) begin : pwm_gen
        reg [15:0] pwm_cnt;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                pwm_cnt <= 16'd0;
                pwm_out[gi] <= 1'b0;
            end else begin
                pwm_cnt <= (pwm_cnt >= pwm_freq[gi]) ? 16'd0 : pwm_cnt + 1'b1;
                pwm_out[gi] <= (pwm_cnt < (pwm_freq[gi] * pwm_duty[gi]) / 100) ? 1'b1 : 1'b0;
            end
        end
    end
endgenerate

endmodule

/*module usb_cdc_interface #(
    parameter PWM_CHANNELS = 4,       // PWM 通道数量
    parameter SPI_MAX_CLK_DIV = 16,   // SPI 最大分频系数
    parameter I2C_MAX_CLK_DIV = 64    // I2C 最大分频系数
)(
    // 全局时钟与复位
    input  wire        clk,           // 系统时钟
    input  wire        rst_n,         // 异步复位（低有效）

    // USB CDC 接口（与 USB 控制器对接）
    input  wire [7:0]  cdc_rx_data,   // CDC 接收数据
    input  wire        cdc_rx_valid,  // CDC 接收数据有效
    output reg  [7:0]  cdc_tx_data,   // CDC 发送数据
    output reg         cdc_tx_valid,  // CDC 发送数据有效
    input  wire        cdc_tx_ready,  // CDC 发送就绪

    // SPI 接口
    output reg         spi_sclk,      // SPI 时钟
    output reg         spi_mosi,      // SPI 主机输出
    input  wire        spi_miso,      // SPI 主机输入 (may be unused in this simplified driver)
    output reg  [3:0]  spi_cs_n,      // SPI 片选（低有效，4路）

    // I2C 接口
    inout  wire        i2c_sda,       // I2C 数据
    inout  wire        i2c_scl,       // I2C 时钟

    // UART 接口
    output reg         uart_tx,       // UART 发送
    input  wire        uart_rx,       // UART 接收 (may be unused in this simplified driver)
    output reg         uart_rts,      // UART 发送请求
    input  wire        uart_cts,      // UART 清除发送 (may be unused)

    // PWM 输出
    output reg [PWM_CHANNELS-1:0] pwm_out  // PWM 输出
);

// Workarounds for "unused input" warnings: refer to inputs so tools see them used.
wire _unused_spi_miso = spi_miso;
wire _unused_uart_rx  = uart_rx;
wire _unused_uart_cts = uart_cts;

// --------------------------
// 1. 命令帧格式定义
// --------------------------
localparam FRAME_HEADER = 8'hAA;
localparam CMD_SPI_CFG  = 8'h01;
localparam CMD_SPI_TX   = 8'h02;
localparam CMD_I2C_CFG  = 8'h03;
localparam CMD_I2C_TX   = 8'h04;
localparam CMD_UART_CFG = 8'h05;
localparam CMD_UART_TX  = 8'h06;
localparam CMD_PWM_CFG  = 8'h07;
localparam CMD_STATUS   = 8'h08;

// --------------------------
// 2. 状态机定义
// --------------------------
localparam S_IDLE      = 4'd0;  // 空闲态，等待帧头
localparam S_CMD       = 4'd1;  // 接收命令码
localparam S_PARA_LEN  = 4'd2;  // 接收参数长度
localparam S_PARA      = 4'd3;  // 接收参数
localparam S_CHECKSUM  = 4'd4;  // 接收校验和
localparam S_EXECUTE   = 4'd5;  // 执行命令
localparam S_RESPONSE  = 4'd6;  // 发送响应

// --------------------------
// 3. 寄存器定义
// --------------------------
reg [3:0] current_state, next_state;
reg [7:0] cmd_reg;              // 命令寄存器
reg [7:0] para_len_reg;         // 参数长度寄存器
reg [7:0] para_buf [0:63];      // 参数缓冲区（最大64字节）
reg [5:0] para_cnt;             // 参数计数
reg [7:0] checksum_reg;         // 校验和寄存器
reg [7:0] response_buf [0:31];  // 响应缓冲区
reg [5:0] response_len;         // 响应长度
reg [5:0] response_cnt;         // 响应计数

// 外设配置寄存器
reg [3:0] spi_clk_div;          // SPI 分频（2^(div+1)）
reg       spi_cpol;             // SPI 时钟极性
reg       spi_cpha;             // SPI 时钟相位
reg [15:0] i2c_clk_div;         // I2C 分频
reg [15:0] i2c_timeout;         // I2C 超时计数
reg [15:0] uart_br_div;         // UART 波特率分频
reg [1:0] uart_data_bits;       // UART 数据位（00=8, 01=7, 10=6, 11=5）
reg       uart_stop_bits;       // UART 停止位（0=1位, 1=2位）
reg [1:0] uart_parity;          // UART 校验（00=无, 01=奇, 10=偶）
reg [15:0] pwm_freq [PWM_CHANNELS-1:0]; // PWM 频率
reg [7:0]  pwm_duty [PWM_CHANNELS-1:0]; // PWM 占空比（0-100）

// I2C tri-state drivers (regs must be driven)
reg i2c_sda_out, i2c_scl_out;
reg i2c_sda_en, i2c_scl_en;
assign i2c_sda = i2c_sda_en ? i2c_sda_out : 1'bz;
assign i2c_scl = i2c_scl_en ? i2c_scl_out : 1'bz;

// loop variables declared at module scope (fix: do not declare inside procedural blocks)
integer i;
integer p;

// --------------------------
// 4. 状态机跳转逻辑
// --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) current_state <= S_IDLE;
    else current_state <= next_state;
end

always @(*) begin
    case (current_state)
        S_IDLE:      next_state = (cdc_rx_valid && cdc_rx_data == FRAME_HEADER) ? S_CMD : S_IDLE;
        S_CMD:       next_state = cdc_rx_valid ? S_PARA_LEN : S_CMD;
        S_PARA_LEN:  next_state = cdc_rx_valid ? S_PARA : S_PARA_LEN;
        S_PARA:      next_state = (cdc_rx_valid && para_cnt == para_len_reg - 1) ? S_CHECKSUM : S_PARA;
        S_CHECKSUM:  next_state = cdc_rx_valid ? S_EXECUTE : S_CHECKSUM;
        S_EXECUTE:   next_state = S_RESPONSE;
        S_RESPONSE:  next_state = (response_cnt == response_len) ? S_IDLE : S_RESPONSE;
        default:     next_state = S_IDLE;
    endcase
end

// --------------------------
// 5. 命令接收与解析
// --------------------------
// 接收命令码
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cmd_reg <= 8'h00;
    else if (current_state == S_CMD && cdc_rx_valid) cmd_reg <= cdc_rx_data;
end

// 接收参数长度
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) para_len_reg <= 8'h00;
    else if (current_state == S_PARA_LEN && cdc_rx_valid) para_len_reg <= cdc_rx_data;
end

// 接收参数与校验和计算
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        para_cnt <= 6'd0;
        checksum_reg <= 8'h00;
        // 初始化 para_buf
        for (p = 0; p < 64; p = p + 1) para_buf[p] <= 8'h00;
    end else if (current_state == S_CMD && cdc_rx_valid) begin
        checksum_reg <= FRAME_HEADER ^ cdc_rx_data;
        para_cnt <= 6'd0;
    end else if (current_state == S_PARA_LEN && cdc_rx_valid) begin
        checksum_reg <= checksum_reg ^ cdc_rx_data;
        para_cnt <= 6'd0;
    end else if (current_state == S_PARA && cdc_rx_valid) begin
        para_buf[para_cnt] <= cdc_rx_data;
        checksum_reg <= checksum_reg ^ cdc_rx_data;
        para_cnt <= para_cnt + 1'b1;
    end else if (current_state == S_EXECUTE) begin
        para_cnt <= 6'd0;
    end
end

// --------------------------
// 6. 命令执行逻辑
// --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        spi_clk_div <= 4'd0;
        spi_cpol <= 1'b0;
        spi_cpha <= 1'b0;
        i2c_clk_div <= 16'd0;
        i2c_timeout <= 16'd1000;
        uart_br_div <= 16'd100;
        uart_data_bits <= 2'd0;
        uart_stop_bits <= 1'b0;
        uart_parity <= 2'd0;
        for (i = 0; i < PWM_CHANNELS; i = i + 1) begin
            pwm_freq[i] <= 16'd1000;
            pwm_duty[i] <= 8'd50;
        end
        response_len <= 6'd0;
        // 初始化 response_buf 数组，避免未驱动位警告
        for (i = 0; i < 32; i = i + 1) response_buf[i] <= 8'h00;
        response_cnt <= 6'd0;

        // 初始化 I2C tri-state drivers 的默认值（无完整 I2C 实现，保持空闲）
        i2c_sda_out <= 1'b1;
        i2c_scl_out <= 1'b1;
        i2c_sda_en  <= 1'b0;
        i2c_scl_en  <= 1'b0;

        // UART 默认空闲/RTS
        uart_tx <= 1'b1;
        uart_rts <= 1'b0;

    end else if (current_state == S_EXECUTE) begin
        // 校验和正确则执行命令
        if (checksum_reg == para_buf[para_len_reg]) begin
            case (cmd_reg)
                CMD_SPI_CFG: begin
                    // SPI配置：[分频(4b)][CPOL(1b)][CPHA(1b)][保留(2b)]
                    spi_clk_div <= para_buf[0][7:4];
                    spi_cpol <= para_buf[0][3];
                    spi_cpha <= para_buf[0][2];
                    response_buf[0] <= 8'h00;
                    response_len <= 6'd1;
                end
                CMD_PWM_CFG: begin
                    // PWM配置：[通道(4b)][频率(16b)][占空比(8b)]
                    if (para_buf[0] < PWM_CHANNELS) begin
                        pwm_freq[para_buf[0]] <= {para_buf[1], para_buf[2]};
                        pwm_duty[para_buf[0]] <= para_buf[3];
                        response_buf[0] <= 8'h00;
                    end else begin
                        response_buf[0] <= 8'hFF;
                    end
                    response_len <= 6'd1;
                end
                // I2C/TX/UART/STATUS 等命令可以在此处扩展
                default: begin
                    response_buf[0] <= 8'hFE;
                    response_len <= 6'd1;
                end
            endcase
        end else begin
            response_buf[0] <= 8'hFD;
            response_len <= 6'd1;
        end
    end
end

// --------------------------
// 7. 响应发送逻辑
// --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cdc_tx_data <= 8'h00;
        cdc_tx_valid <= 1'b0;
        response_cnt <= 6'd0;
    end else if (current_state == S_RESPONSE) begin
        if (cdc_tx_ready) begin
            if (response_cnt < response_len) begin
                cdc_tx_data <= response_buf[response_cnt];
                cdc_tx_valid <= 1'b1;
                response_cnt <= response_cnt + 1'b1;
            end else begin
                cdc_tx_valid <= 1'b0;
                response_cnt <= 6'd0;
            end
        end else begin
            cdc_tx_valid <= 1'b0;
        end
    end else begin
        cdc_tx_valid <= 1'b0;
        response_cnt <= 6'd0;
    end
end

// --------------------------
// 8. 外设驱动实例化（简化）
// --------------------------
// SPI 驱动 - 保持简化实现，同时驱动 UART 默认线（避免无驱动警告）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        spi_sclk <= 1'b0;
        spi_mosi <= 1'b0;
        spi_cs_n <= 4'hF;
        uart_tx <= 1'b1;
        uart_rts <= 1'b0;
    end else begin
        spi_sclk <= spi_cpol;
        // uart_tx / uart_rts 保持默认空闲值（可在 UART 逻辑扩展时驱动）
        uart_tx <= uart_tx;
        uart_rts <= uart_rts;
    end
end

// I2C tri-state signals: keep default idle values if no I2C implementation present
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        i2c_sda_out <= 1'b1;
        i2c_scl_out <= 1'b1;
        i2c_sda_en  <= 1'b0;
        i2c_scl_en  <= 1'b0;
    end else begin
        // no active I2C master implementation in this simplified module;
        // keep outputs tri-stated / idle. If adding I2C logic later, drive these regs.
        i2c_sda_out <= i2c_sda_out;
        i2c_scl_out <= i2c_scl_out;
        i2c_sda_en  <= 1'b0;
        i2c_scl_en  <= 1'b0;
    end
end

// --------------------------
// PWM 生成（使用标准 Verilog 索引写法）
// --------------------------
genvar gi;
generate
    for (gi = 0; gi < PWM_CHANNELS; gi = gi + 1) begin : pwm_gen
        reg [15:0] pwm_cnt;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                pwm_cnt <= 16'd0;
                pwm_out[gi] <= 1'b0;
            end else begin
                pwm_cnt <= (pwm_cnt >= pwm_freq[gi]) ? 16'd0 : pwm_cnt + 1'b1;
                pwm_out[gi] <= (pwm_cnt < (pwm_freq[gi] * pwm_duty[gi]) / 100) ? 1'b1 : 1'b0;
            end
        end
    end
endgenerate

endmodule
/*module usb_cdc_interface #(
    parameter PWM_CHANNELS = 4,       // PWM 通道数量
    parameter SPI_MAX_CLK_DIV = 16,   // SPI 最大分频系数
    parameter I2C_MAX_CLK_DIV = 64    // I2C 最大分频系数
)(
    // 全局时钟与复位
    input  wire        clk,           // 系统时钟
    input  wire        rst_n,         // 异步复位（低有效）

    // USB CDC 接口（与 USB 控制器对接）
    input  wire [7:0]  cdc_rx_data,   // CDC 接收数据
    input  wire        cdc_rx_valid,  // CDC 接收数据有效
    output reg  [7:0]  cdc_tx_data,   // CDC 发送数据
    output reg         cdc_tx_valid,  // CDC 发送数据有效
    input  wire        cdc_tx_ready,  // CDC 发送就绪

    // SPI 接口
    output reg         spi_sclk,      // SPI 时钟
    output reg         spi_mosi,      // SPI 主机输出
    input  wire        spi_miso,      // SPI 主机输入
    output reg  [3:0]  spi_cs_n,      // SPI 片选（低有效，4路）

    // I2C 接口
    inout  wire        i2c_sda,       // I2C 数据
    inout  wire        i2c_scl,       // I2C 时钟

    // UART 接口
    output reg         uart_tx,       // UART 发送
    input  wire        uart_rx,       // UART 接收
    output reg         uart_rts,      // UART 发送请求
    input  wire        uart_cts,      // UART 清除发送

    // PWM 输出
    output reg [PWM_CHANNELS-1:0] pwm_out  // PWM 输出
);

// --------------------------
// 1. 命令帧格式定义
// --------------------------
localparam FRAME_HEADER = 8'hAA;
localparam CMD_SPI_CFG  = 8'h01;
localparam CMD_SPI_TX   = 8'h02;
localparam CMD_I2C_CFG  = 8'h03;
localparam CMD_I2C_TX   = 8'h04;
localparam CMD_UART_CFG = 8'h05;
localparam CMD_UART_TX  = 8'h06;
localparam CMD_PWM_CFG  = 8'h07;
localparam CMD_STATUS   = 8'h08;

// --------------------------
// 2. 状态机定义
// --------------------------
localparam S_IDLE      = 4'd0;  // 空闲态，等待帧头
localparam S_CMD       = 4'd1;  // 接收命令码
localparam S_PARA_LEN  = 4'd2;  // 接收参数长度
localparam S_PARA      = 4'd3;  // 接收参数
localparam S_CHECKSUM  = 4'd4;  // 接收校验和
localparam S_EXECUTE   = 4'd5;  // 执行命令
localparam S_RESPONSE  = 4'd6;  // 发送响应

// --------------------------
// 3. 寄存器定义
// --------------------------
reg [3:0] current_state, next_state;
reg [7:0] cmd_reg;              // 命令寄存器
reg [7:0] para_len_reg;         // 参数长度寄存器
reg [7:0] para_buf [0:63];      // 参数缓冲区（最大64字节）
reg [5:0] para_cnt;             // 参数计数
reg [7:0] checksum_reg;         // 校验和寄存器
reg [7:0] response_buf [0:31];  // 响应缓冲区
reg [5:0] response_len;         // 响应长度
reg [5:0] response_cnt;         // 响应计数

// 外设配置寄存器
reg [3:0] spi_clk_div;          // SPI 分频（2^(div+1)）
reg       spi_cpol;             // SPI 时钟极性
reg       spi_cpha;             // SPI 时钟相位
reg [15:0] i2c_clk_div;         // I2C 分频
reg [15:0] i2c_timeout;         // I2C 超时计数
reg [15:0] uart_br_div;         // UART 波特率分频
reg [1:0] uart_data_bits;       // UART 数据位（00=8, 01=7, 10=6, 11=5）
reg       uart_stop_bits;       // UART 停止位（0=1位, 1=2位）
reg [1:0] uart_parity;          // UART 校验（00=无, 01=奇, 10=偶）
reg [15:0] pwm_freq [PWM_CHANNELS-1:0]; // PWM 频率
reg [7:0]  pwm_duty [PWM_CHANNELS-1:0]; // PWM 占空比（0-100）

// --------------------------
// 4. 状态机跳转逻辑
// --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) current_state <= S_IDLE;
    else current_state <= next_state;
end

always @(*) begin
    case (current_state)
        S_IDLE:      next_state = (cdc_rx_valid && cdc_rx_data == FRAME_HEADER) ? S_CMD : S_IDLE;
        S_CMD:       next_state = cdc_rx_valid ? S_PARA_LEN : S_CMD;
        S_PARA_LEN:  next_state = cdc_rx_valid ? S_PARA : S_PARA_LEN;
        S_PARA:      next_state = (cdc_rx_valid && para_cnt == para_len_reg - 1) ? S_CHECKSUM : S_PARA;
        S_CHECKSUM:  next_state = cdc_rx_valid ? S_EXECUTE : S_CHECKSUM;
        S_EXECUTE:   next_state = S_RESPONSE;
        S_RESPONSE:  next_state = (response_cnt == response_len) ? S_IDLE : S_RESPONSE;
        default:     next_state = S_IDLE;
    endcase
end

// --------------------------
// 5. 命令接收与解析
// --------------------------
// 接收命令码
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cmd_reg <= 8'h00;
    else if (current_state == S_CMD && cdc_rx_valid) cmd_reg <= cdc_rx_data;
end

// 接收参数长度
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) para_len_reg <= 8'h00;
    else if (current_state == S_PARA_LEN && cdc_rx_valid) para_len_reg <= cdc_rx_data;
end

// 接收参数与校验和计算
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        para_cnt <= 6'd0;
        checksum_reg <= 8'h00;
    end else if (current_state == S_CMD && cdc_rx_valid) begin
        checksum_reg <= FRAME_HEADER ^ cdc_rx_data;
        para_cnt <= 6'd0;
    end else if (current_state == S_PARA_LEN && cdc_rx_valid) begin
        checksum_reg <= checksum_reg ^ cdc_rx_data;
        para_cnt <= 6'd0;
    end else if (current_state == S_PARA && cdc_rx_valid) begin
        para_buf[para_cnt] <= cdc_rx_data;
        checksum_reg <= checksum_reg ^ cdc_rx_data;
        para_cnt <= para_cnt + 1'b1;
    end else if (current_state == S_EXECUTE) begin
        para_cnt <= 6'd0;
    end
end

// --------------------------
// 6. 命令执行逻辑
// --------------------------
integer i;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        spi_clk_div <= 4'd0;
        spi_cpol <= 1'b0;
        spi_cpha <= 1'b0;
        i2c_clk_div <= 16'd0;
        i2c_timeout <= 16'd1000;
        uart_br_div <= 16'd100;
        uart_data_bits <= 2'd0;
        uart_stop_bits <= 1'b0;
        uart_parity <= 2'd0;
        for (i = 0; i < PWM_CHANNELS; i = i + 1) begin
            pwm_freq[i] <= 16'd1000;
            pwm_duty[i] <= 8'd50;
        end
        response_len <= 6'd0;
    end else if (current_state == S_EXECUTE) begin
        // 校验和正确则执行命令
        if (checksum_reg == para_buf[para_len_reg]) begin
            case (cmd_reg)
                CMD_SPI_CFG: begin
                    // SPI配置：[分频(4b)][CPOL(1b)][CPHA(1b)][保留(2b)]
                    spi_clk_div <= para_buf[0][7:4];
                    spi_cpol <= para_buf[0][3];
                    spi_cpha <= para_buf[0][2];
                    response_buf[0] <= 8'h00;
                    response_len <= 6'd1;
                end
                CMD_PWM_CFG: begin
                    // PWM配置：[通道(4b)][频率(16b)][占空比(8b)]
                    if (para_buf[0] < PWM_CHANNELS) begin
                        pwm_freq[para_buf[0]] <= {para_buf[1], para_buf[2]};
                        pwm_duty[para_buf[0]] <= para_buf[3];
                        response_buf[0] <= 8'h00;
                    end else begin
                        response_buf[0] <= 8'hFF;
                    end
                    response_len <= 6'd1;
                end
                // 其他命令略
                default: begin
                    response_buf[0] <= 8'hFE;
                    response_len <= 6'd1;
                end
            endcase
        end else begin
            response_buf[0] <= 8'hFD;
            response_len <= 6'd1;
        end
    end
end

// --------------------------
// 7. 响应发送逻辑
// --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cdc_tx_data <= 8'h00;
        cdc_tx_valid <= 1'b0;
        response_cnt <= 6'd0;
    end else if (current_state == S_RESPONSE) begin
        if (cdc_tx_ready) begin
            if (response_cnt < response_len) begin
                cdc_tx_data <= response_buf[response_cnt];
                cdc_tx_valid <= 1'b1;
                response_cnt <= response_cnt + 1'b1;
            end else begin
                cdc_tx_valid <= 1'b0;
                response_cnt <= 6'd0;
            end
        end else begin
            cdc_tx_valid <= 1'b0;
        end
    end else begin
        cdc_tx_valid <= 1'b0;
        response_cnt <= 6'd0;
    end
end

// --------------------------
// 8. 外设驱动实例化（简化）
// --------------------------
// SPI 驱动
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        spi_sclk <= 1'b0;
        spi_mosi <= 1'b0;
        spi_cs_n <= 4'hF;
    end else begin
        spi_sclk <= spi_cpol;
    end
end

// I2C 驱动
reg i2c_sda_out, i2c_scl_out;
reg i2c_sda_en, i2c_scl_en;
assign i2c_sda = i2c_sda_en ? i2c_sda_out : 1'bz;
assign i2c_scl = i2c_scl_en ? i2c_scl_out : 1'bz;

// PWM 生成（使用标准 Verilog 索引写法）
genvar gi;
generate
    for (gi = 0; gi < PWM_CHANNELS; gi = gi + 1) begin : pwm_gen
        reg [15:0] pwm_cnt;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                pwm_cnt <= 16'd0;
                pwm_out[gi] <= 1'b0;
            end else begin
                pwm_cnt <= (pwm_cnt >= pwm_freq[gi]) ? 16'd0 : pwm_cnt + 1'b1;
                // 替换SystemVerilog切片为标准Verilog写法
                // pwm_out[gi] <= (pwm_cnt < pwm_freq[gi] * pwm_duty[gi] / 100) ? 1'b1 : 1'b0;
                // ↑此处不涉及[a +: b]等写法，已兼容标准Verilog
                pwm_out[gi] <= (pwm_cnt < (pwm_freq[gi] * pwm_duty[gi]) / 100) ? 1'b1 : 1'b0;
            end
        end
    end
endgenerate

// UART 驱动（需补充发送/接收时序逻辑）

endmodule*/