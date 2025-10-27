module analog_signal_acquisition (
    input               clk,            // 100MHz主时钟
    input               clk_high,       // 200MHz高速时钟
    input               rst_n,
    
    // ADC接口
    input [11:0]        adc_data,       // ADC数据输入
    output reg          adc_clk,        // ADC时钟输出
    output reg          adc_rst,        // ADC复位
    
    // 命令接口
    input [7:0]         cmd_opcode,
    input [15:0]        cmd_addr,
    input [31:0]        cmd_data,
    input               cmd_valid,
    
    // 数据发送接口
    output reg [31:0]   tx_data,
    output reg          tx_en,
    input               tx_done
);

// 寄存器定义
reg [31:0]   adc_config_reg;     // ADC配置寄存器
reg [31:0]   sample_rate_reg;    // 采样率寄存器
reg [31:0]   sample_count_reg;   // 采样点数寄存器
reg [31:0]   trigger_level_reg;  // 触发电平寄存器
reg [31:0]   status_reg;         // 状态寄存器  <-- 现在由单一 always 驱动

// 内部信号
reg [11:0]   adc_data_sync;      // 同步后的ADC数据
reg          adc_sample_en;      // ADC采样使能
reg [31:0]   sample_cnt;         // 采样计数器
reg [31:0]   sample_total;       // 总采样点数
reg          trigger_detected;   // 触发检测标志
reg          fifo_wr_en;         // FIFO写使能
reg [11:0]   fifo_wr_data;       // FIFO写入数据
wire         fifo_full;          // FIFO满标志
wire         fifo_empty;         // FIFO空标志
wire [11:0]  fifo_rd_data;       // FIFO读出数据
reg          fifo_rd_en;         // FIFO读使能
reg [1:0]    adc_state;          // ADC状态机

// 新增：独立状态标志，避免在多个 always 中驱动 status_reg
reg status_sampling_f;
reg status_trigger_f;
reg status_data_ready_f;

// 状态定义
localparam ADC_IDLE = 2'd0;
localparam ADC_SAMPLING = 2'd1;
localparam ADC_TRIGGERED = 2'd2;
localparam ADC_DONE = 2'd3;

// ADC时钟生成 (100MHz)
always @(posedge clk_high or negedge rst_n) begin
    if (!rst_n) begin
        adc_clk <= 1'b0;
    end else begin
        adc_clk <= ~adc_clk;  // 由200MHz分频得到100MHz ADC时钟
    end
end

// ADC复位控制
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_rst <= 1'b1;  // 复位有效
    end else if (cmd_valid && cmd_opcode == 8'h20 && cmd_data[0] == 1'b1) begin
        adc_rst <= 1'b1;  // 启动复位
    end else begin
        adc_rst <= 1'b0;  // 复位释放
    end
end

// ADC数据同步
always @(posedge adc_clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_data_sync <= 12'd0;
    end else begin
        adc_data_sync <= adc_data;
    end
end

// 触发检测 (adc_clk 域)
always @(posedge adc_clk or negedge rst_n) begin
    if (!rst_n) begin
        trigger_detected <= 1'b0;
    end else if (adc_sample_en && adc_state == ADC_SAMPLING) begin
        // 简单电平触发检测
        if (adc_data_sync >= trigger_level_reg[11:0]) begin
            trigger_detected <= 1'b1;
        end else begin
            trigger_detected <= 1'b0;
        end
    end else begin
        trigger_detected <= 1'b0;
    end
end

// 采样FIFO实例化 (深度4096)
// 注意：确保工程中存在 fifo_12x4096 的实现文件或 IP
fifo_12x4096 adc_fifo (
    .wr_clk     (adc_clk),
    .rd_clk     (clk),
    .rst        (!rst_n),
    .din        (fifo_wr_data),
    .wr_en      (fifo_wr_en),
    .rd_en      (fifo_rd_en),
    .dout       (fifo_rd_data),
    .full       (fifo_full),
    .empty      (fifo_empty)
);

// FIFO写入控制 (adc_clk 域)
always @(posedge adc_clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_wr_en <= 1'b0;
        fifo_wr_data <= 12'd0;
        sample_cnt <= 32'd0;
    end else begin
        fifo_wr_en <= 1'b0;
        
        if (adc_sample_en) begin
            // 当采样使能时，写入FIFO
            if (!fifo_full) begin
                fifo_wr_en <= 1'b1;
                fifo_wr_data <= adc_data_sync;
                
                // 计数已采样点数（只有触发后才计数）
                if (adc_state == ADC_TRIGGERED) begin
                    sample_cnt <= sample_cnt + 1'b1;
                end
            end
        end else begin
            sample_cnt <= 32'd0;
        end
    end
end

// ADC状态机 (clk 域)
// 状态机只修改内部标志 status_*_f，不直接写 status_reg，避免多驱动
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_state <= ADC_IDLE;
        adc_sample_en <= 1'b0;
        sample_total <= 32'd0;
        status_sampling_f <= 1'b0;
        status_trigger_f <= 1'b0;
        status_data_ready_f <= 1'b0;
    end else begin
        case (adc_state)
            ADC_IDLE: begin
                // 等待启动命令
                if (cmd_valid && cmd_opcode == 8'h21) begin
                    adc_sample_en <= 1'b1;
                    sample_total <= cmd_data;  // 设置采样点数
                    adc_state <= ADC_SAMPLING;
                    status_sampling_f <= 1'b1;  // 采样中标志
                    status_trigger_f <= 1'b0;
                    status_data_ready_f <= 1'b0;
                end
            end
            
            ADC_SAMPLING: begin
                // 等待触发
                if (trigger_detected || (adc_config_reg[0] == 1'b0)) begin
                    // 触发条件满足或处于自由运行模式
                    adc_state <= ADC_TRIGGERED;
                    status_trigger_f <= 1'b1;  // 触发标志
                end
            end
            
            ADC_TRIGGERED: begin
                // 采样指定数量的数据
                if (sample_cnt >= sample_total) begin
                    adc_sample_en <= 1'b0;
                    adc_state <= ADC_DONE;
                    status_sampling_f <= 1'b0;  // 采样完成
                    status_data_ready_f <= 1'b1;  // 数据就绪标志
                end
            end
            
            ADC_DONE: begin
                // 等待数据读取完成
                if (cmd_valid && cmd_opcode == 8'h22) begin
                    adc_state <= ADC_IDLE;
                    status_trigger_f <= 1'b0;
                    status_data_ready_f <= 1'b0;
                end
            end
        endcase
    end
end

// 寄存器配置与 status_reg 统一驱动 (clk 域)
// status_reg 仅在这里被赋值，避免多驱动
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_config_reg <= 32'd0;      // 默认: 触发模式
        sample_rate_reg <= 32'd10_000_000;  // 默认10Msps
        trigger_level_reg <= 32'd2048;  // 中点触发
        status_reg <= 32'd0;
    end else begin
        // 更新配置寄存器（收到命令时）
        if (cmd_valid) begin
            case (cmd_opcode)
                8'h23: adc_config_reg <= cmd_data;      // 配置寄存器
                8'h24: sample_rate_reg <= cmd_data;     // 采样率寄存器
                8'h25: trigger_level_reg <= cmd_data;   // 触发电平
            endcase
        end
        // 根据内部标志组合 status_reg 的低位（[2]=data_ready, [1]=trigger, [0]=sampling）
        status_reg <= {29'd0, status_data_ready_f, status_trigger_f, status_sampling_f};
    end
end

// FIFO数据读出与发送 (clk 域)
reg [1:0] tx_state;
localparam TX_IDLE = 2'd0;
localparam TX_READ = 2'd1;
localparam TX_SEND = 2'd2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_state <= TX_IDLE;
        tx_data <= 32'd0;
        tx_en <= 1'b0;
        fifo_rd_en <= 1'b0;
    end else begin
        case (tx_state)
            TX_IDLE: begin
                // 等待读取命令
                if (cmd_valid && cmd_opcode == 8'h26) begin
                    fifo_rd_en <= 1'b1;  // 开始读取FIFO
                    tx_state <= TX_READ;
                end
            end
            
            TX_READ: begin
                fifo_rd_en <= 1'b0;
                // 等待FIFO数据输出
                tx_data <= {20'd0, fifo_rd_data};  // 扩展为32位
                tx_en <= 1'b1;
                tx_state <= TX_SEND;
            end
            
            TX_SEND: begin
                if (tx_done) begin
                    tx_en <= 1'b0;
                    // 检查是否还有数据或是否完成读取
                    if (fifo_empty || (cmd_valid && cmd_opcode == 8'h27)) begin
                        tx_state <= TX_IDLE;
                    end else begin
                        fifo_rd_en <= 1'b1;
                        tx_state <= TX_READ;
                    end
                end
            end
        endcase
    end
end

endmodule



/*module analog_signal_acquisition (
    input               clk,            // 100MHz主时钟
    input               clk_high,       // 200MHz高速时钟
    input               rst_n,
    
    // ADC接口
    input [11:0]        adc_data,       // ADC数据输入
    output reg          adc_clk,        // ADC时钟输出
    output reg          adc_rst,        // ADC复位
    
    // 命令接口
    input [7:0]         cmd_opcode,
    input [15:0]        cmd_addr,
    input [31:0]        cmd_data,
    input               cmd_valid,
    
    // 数据发送接口
    output reg [31:0]   tx_data,
    output reg          tx_en,
    input               tx_done
);

// 寄存器定义
reg [31:0]   adc_config_reg;     // ADC配置寄存器
reg [31:0]   sample_rate_reg;    // 采样率寄存器
reg [31:0]   sample_count_reg;   // 采样点数寄存器
reg [31:0]   trigger_level_reg;  // 触发电平寄存器
reg [31:0]   status_reg;         // 状态寄存器

// 内部信号
reg [11:0]   adc_data_sync;      // 同步后的ADC数据
reg          adc_sample_en;      // ADC采样使能
reg [31:0]   sample_cnt;         // 采样计数器
reg [31:0]   sample_total;       // 总采样点数
reg          trigger_detected;   // 触发检测标志
reg          fifo_wr_en;         // FIFO写使能
reg [11:0]   fifo_wr_data;       // FIFO写入数据
wire         fifo_full;          // FIFO满标志
wire         fifo_empty;         // FIFO空标志
wire [11:0]  fifo_rd_data;       // FIFO读出数据
reg          fifo_rd_en;         // FIFO读使能
reg [1:0]    adc_state;          // ADC状态机

// 状态定义
localparam ADC_IDLE = 2'd0;
localparam ADC_SAMPLING = 2'd1;
localparam ADC_TRIGGERED = 2'd2;
localparam ADC_DONE = 2'd3;

// ADC时钟生成 (100MHz)
always @(posedge clk_high or negedge rst_n) begin
    if (!rst_n) begin
        adc_clk <= 1'b0;
    end else begin
        adc_clk <= ~adc_clk;  // 由200MHz分频得到100MHz ADC时钟
    end
end

// ADC复位控制
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_rst <= 1'b1;  // 复位有效
    end else if (cmd_valid && cmd_opcode == 8'h20 && cmd_data[0] == 1'b1) begin
        adc_rst <= 1'b1;  // 启动复位
    end else begin
        adc_rst <= 1'b0;  // 复位释放
    end
end

// ADC数据同步
always @(posedge adc_clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_data_sync <= 12'd0;
    end else begin
        adc_data_sync <= adc_data;
    end
end

// 触发检测
always @(posedge adc_clk or negedge rst_n) begin
    if (!rst_n) begin
        trigger_detected <= 1'b0;
    end else if (adc_sample_en && adc_state == ADC_SAMPLING) begin
        // 简单电平触发检测
        if (adc_data_sync >= trigger_level_reg[11:0]) begin
            trigger_detected <= 1'b1;
        end else begin
            trigger_detected <= 1'b0;
        end
    end else begin
        trigger_detected <= 1'b0;
    end
end

// 采样FIFO实例化 (深度4096)
fifo_12x4096 adc_fifo (
    .wr_clk     (adc_clk),
    .rd_clk     (clk),
    .rst        (!rst_n),
    .din        (fifo_wr_data),
    .wr_en      (fifo_wr_en),
    .rd_en      (fifo_rd_en),
    .dout       (fifo_rd_data),
    .full       (fifo_full),
    .empty      (fifo_empty)
);

// FIFO写入控制
always @(posedge adc_clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_wr_en <= 1'b0;
        fifo_wr_data <= 12'd0;
        sample_cnt <= 32'd0;
    end else begin
        fifo_wr_en <= 1'b0;
        
        if (adc_sample_en) begin
            // 当采样使能时，写入FIFO
            if (!fifo_full) begin
                fifo_wr_en <= 1'b1;
                fifo_wr_data <= adc_data_sync;
                
                // 计数已采样点数
                if (adc_state == ADC_TRIGGERED) begin
                    sample_cnt <= sample_cnt + 1'b1;
                end
            end
        end else begin
            sample_cnt <= 32'd0;
        end
    end
end

// ADC状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_state <= ADC_IDLE;
        adc_sample_en <= 1'b0;
        sample_total <= 32'd0;
    end else begin
        case (adc_state)
            ADC_IDLE: begin
                // 等待启动命令
                if (cmd_valid && cmd_opcode == 8'h21) begin
                    adc_sample_en <= 1'b1;
                    sample_total <= cmd_data;  // 设置采样点数
                    adc_state <= ADC_SAMPLING;
                    status_reg[0] <= 1'b1;  // 采样中标志
                end
            end
            
            ADC_SAMPLING: begin
                // 等待触发
                if (trigger_detected || (adc_config_reg[0] == 1'b0)) begin
                    // 触发条件满足或处于自由运行模式
                    adc_state <= ADC_TRIGGERED;
                    status_reg[1] <= 1'b1;  // 触发标志
                end
            end
            
            ADC_TRIGGERED: begin
                // 采样指定数量的数据
                if (sample_cnt >= sample_total) begin
                    adc_sample_en <= 1'b0;
                    adc_state <= ADC_DONE;
                    status_reg[0] <= 1'b0;  // 采样完成
                    status_reg[2] <= 1'b1;  // 数据就绪标志
                end
            end
            
            ADC_DONE: begin
                // 等待数据读取完成
                if (cmd_valid && cmd_opcode == 8'h22) begin
                    adc_state <= ADC_IDLE;
                    status_reg[1] <= 1'b0;
                    status_reg[2] <= 1'b0;
                end
            end
        endcase
    end
end

// 寄存器配置
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_config_reg <= 32'd0;      // 默认: 触发模式
        sample_rate_reg <= 32'd10_000_000;  // 默认10Msps
        trigger_level_reg <= 32'd2048;  // 中点触发
        status_reg <= 32'd0;
    end else if (cmd_valid) begin
        case (cmd_opcode)
            8'h23: adc_config_reg <= cmd_data;      // 配置寄存器
            8'h24: sample_rate_reg <= cmd_data;     // 采样率寄存器
            8'h25: trigger_level_reg <= cmd_data;   // 触发电平
        endcase
    end
end

// FIFO数据读出与发送
reg [1:0] tx_state;
localparam TX_IDLE = 2'd0;
localparam TX_READ = 2'd1;
localparam TX_SEND = 2'd2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_state <= TX_IDLE;
        tx_data <= 32'd0;
        tx_en <= 1'b0;
        fifo_rd_en <= 1'b0;
    end else begin
        case (tx_state)
            TX_IDLE: begin
                // 等待读取命令
                if (cmd_valid && cmd_opcode == 8'h26) begin
                    fifo_rd_en <= 1'b1;  // 开始读取FIFO
                    tx_state <= TX_READ;
                end
            end
            
            TX_READ: begin
                fifo_rd_en <= 1'b0;
                // 等待FIFO数据输出
                tx_data <= {20'd0, fifo_rd_data};  // 扩展为32位
                tx_en <= 1'b1;
                tx_state <= TX_SEND;
            end
            
            TX_SEND: begin
                if (tx_done) begin
                    tx_en <= 1'b0;
                    // 检查是否还有数据或是否完成读取
                    if (fifo_empty || (cmd_valid && cmd_opcode == 8'h27)) begin
                        tx_state <= TX_IDLE;
                    end else begin
                        fifo_rd_en <= 1'b1;
                        tx_state <= TX_READ;
                    end
                end
            end
        endcase
    end
end

endmodule*/
