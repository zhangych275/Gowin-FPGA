module command_parser #(
    parameter MAX_PARA_LEN = 64,      // 最大参数长度（字节）
    parameter CHECKSUM_EN  = 1'b1     // 使能校验和（1=启用，0=禁用）
)(
    // 全局时钟与复位
    input  wire                  clk,
    input  wire                  rst_n,

    // 输入：待解析的 CDC 数据
    input  wire [7:0]            cdc_rx_data,    // 接收数据
    input  wire                  cdc_rx_valid,   // 数据有效信号

    // 输出：解析结果
    output reg                   cmd_valid,      // 命令有效（解析成功）
    output reg [7:0]             cmd_opcode,     // 命令 opcode（操作类型）
    output reg [2:0]             cmd_periph,     // 目标外设（0=SPI,1=I2C,2=UART,3=PWM）
    output reg [7:0]             para_len,       // 参数长度
    output reg [7:0]             para_buf,       // 参数缓冲区（单字节输出端口）
    output reg                   frame_error,    // 帧错误（格式/校验错）
    output reg [1:0]             error_type      // 错误类型（0=帧头错,1=校验错,2=长度错）
);

// --------------------------
// 1. 命令帧格式定义
// --------------------------
localparam FRAME_HEADER  = 8'h5A;
localparam PERIPH_SPI    = 3'd0;
localparam PERIPH_I2C    = 3'd1;
localparam PERIPH_UART   = 3'd2;
localparam PERIPH_PWM    = 3'd3;

localparam OP_CONFIG     = 5'd1;  // 配置外设参数
localparam OP_TRANSFER   = 5'd2;  // 数据传输
localparam OP_QUERY      = 5'd3;  // 查询状态

// 错误类型定义
localparam ERR_HEADER    = 2'd0;  // 帧头错误
localparam ERR_CHECKSUM  = 2'd1;  // 校验和错误
localparam ERR_PARA_LEN  = 2'd2;  // 参数长度超限

// --------------------------
// 2. 解析状态机定义
// --------------------------
localparam S_IDLE        = 4'd0;  // 等待帧头
localparam S_CMD         = 4'd1;  // 接收命令字节（外设ID+操作码）
localparam S_PARA_LEN    = 4'd2;  // 接收参数长度
localparam S_PARA        = 4'd3;  // 接收参数
localparam S_CHECKSUM    = 4'd4;  // 接收校验和
localparam S_VERIFY      = 4'd5;  // 校验帧合法性
localparam S_DONE        = 4'd6;  // 解析完成

// --------------------------
// 3. 寄存器定义
// --------------------------
reg [3:0] current_state, next_state;
reg [7:0] checksum_calc;          // 计算的校验和
reg [7:0] checksum_rx;            // 接收的校验和
reg [7:0] para_cnt;               // 参数接收计数
reg [7:0] cmd_reg;                // 命令寄存器（外设ID+操作码）

reg [7:0] internal_para_buf [0:MAX_PARA_LEN-1]; // 内部参数缓冲区（仅内部使用）

// --------------------------
// 4. 状态机跳转逻辑
// --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) current_state <= S_IDLE;
    else current_state <= next_state;
end

always @(*) begin
    case (current_state)
        S_IDLE:      // 等待帧头
            next_state = (cdc_rx_valid && cdc_rx_data == FRAME_HEADER) ? S_CMD : S_IDLE;
        
        S_CMD:       // 接收命令字节
            next_state = cdc_rx_valid ? S_PARA_LEN : S_CMD;
        
        S_PARA_LEN:  // 接收参数长度
            next_state = cdc_rx_valid ? 
                        (cdc_rx_data > MAX_PARA_LEN ? S_VERIFY : S_PARA) :  // 长度超限直接进入校验
                        S_PARA_LEN;
        
        S_PARA:      // 接收参数
            next_state = (cdc_rx_valid && para_cnt == para_len - 1) ? S_CHECKSUM : S_PARA;
        
        S_CHECKSUM:  // 接收校验和
            next_state = cdc_rx_valid ? S_VERIFY : S_CHECKSUM;
        
        S_VERIFY:    // 校验帧合法性
            next_state = S_DONE;
        
        S_DONE:      // 解析完成，回到空闲
            next_state = S_IDLE;
        
        default: 
            next_state = S_IDLE;
    endcase
end

// --------------------------
// 5. 数据接收与校验和计算
// --------------------------
// 命令寄存器（外设ID+操作码）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cmd_reg <= 8'h00;
    else if (current_state == S_CMD && cdc_rx_valid) cmd_reg <= cdc_rx_data;
end

// 参数长度寄存器
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) para_len <= 8'h00;
    else if (current_state == S_PARA_LEN && cdc_rx_valid) begin
        para_len <= (cdc_rx_data > MAX_PARA_LEN) ? MAX_PARA_LEN : cdc_rx_data;
    end
end

// 参数缓冲区与计数
integer i;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        para_cnt <= 8'h00;
        for (i = 0; i < MAX_PARA_LEN; i = i + 1) internal_para_buf[i] <= 8'h00;
    end else if (current_state == S_PARA && cdc_rx_valid) begin
        internal_para_buf[para_cnt] <= cdc_rx_data;
        para_cnt <= para_cnt + 1'b1;
    end else if (current_state == S_IDLE) begin
        para_cnt <= 8'h00;
    end
end

// 校验和计算（累加和）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) checksum_calc <= 8'h00;
    else begin
        case (current_state)
            S_IDLE:
                checksum_calc <= 8'h00;
            S_CMD:
                if (cdc_rx_valid) checksum_calc <= FRAME_HEADER + cdc_rx_data;
            S_PARA_LEN:
                if (cdc_rx_valid) checksum_calc <= checksum_calc + cdc_rx_data;
            S_PARA:
                if (cdc_rx_valid) checksum_calc <= checksum_calc + cdc_rx_data;
            default:
                checksum_calc <= checksum_calc;
        endcase
    end
end

// 接收校验和
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) checksum_rx <= 8'h00;
    else if (current_state == S_CHECKSUM && cdc_rx_valid) checksum_rx <= cdc_rx_data;
end

// --------------------------
// 6. 帧校验与结果输出
// --------------------------
// 解析结果与错误标识
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cmd_valid  <= 1'b0;
        cmd_opcode <= 8'h00;
        cmd_periph <= 3'd0;
        para_buf   <= 8'h00;
        frame_error <= 1'b0;
        error_type <= 2'd0;
    end else if (current_state == S_VERIFY) begin
        // 默认解析失败
        cmd_valid  <= 1'b0;
        frame_error <= 1'b1;
        error_type <= 2'd0;

        if (cdc_rx_data != FRAME_HEADER && current_state == S_IDLE) begin
            error_type <= ERR_HEADER;
        end
        else if (para_len > MAX_PARA_LEN) begin
            error_type <= ERR_PARA_LEN;
        end
        else if (CHECKSUM_EN && (checksum_calc != checksum_rx)) begin
            error_type <= ERR_CHECKSUM;
        end
        else begin
            frame_error <= 1'b0;
            cmd_valid   <= 1'b1;
            cmd_periph  <= cmd_reg[7:5];
            cmd_opcode  <= {3'h0, cmd_reg[4:0]};
            // 输出第一个参数字节，实际应用可根据需要做多字节参数输出
            para_buf    <= internal_para_buf[0];
        end
    end else begin
        cmd_valid <= 1'b0;
    end
end

endmodule