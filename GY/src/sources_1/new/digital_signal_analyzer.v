module digital_signal_analyzer (
    input               clk,            // 100MHz主时钟
    input               clk_high,       // 200MHz高速时钟，用于精确测量
    input               rst_n,
    
    // 输入信号
    input [3:0]         sig_in,         // 4路输入信号
    
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
reg [31:0]   freq_reg[3:0];      // 频率测量结果寄存器
reg [31:0]   high_time_reg[3:0]; // 高电平时间寄存器
reg [31:0]   low_time_reg[3:0];  // 低电平时间寄存器
reg [31:0]   duty_reg[3:0];      // 占空比寄存器(以万分之一为单位)
reg [3:0]    measure_en;         // 测量使能寄存器

// 内部信号
reg [3:0]    sig_sync;           // 同步后的输入信号
reg [3:0]    sig_prev;           // 上一周期的信号值
wire [3:0]   sig_rise;           // 上升沿检测
wire [3:0]   sig_fall;           // 下降沿检测

// 高速计数器
reg [31:0]   high_cnt[3:0];      // 高速计数器
reg [31:0]   period_cnt[3:0];    // 周期计数器
reg [31:0]   high_time[3:0];     // 高电平时间
reg [31:0]   low_time[3:0];      // 低电平时间

// 输入信号同步，防止 metastability
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sig_sync <= 4'd0;
        sig_prev <= 4'd0;
    end else begin
        sig_sync <= sig_in;
        sig_prev <= sig_sync;
    end
end

// 边沿检测
assign sig_rise = sig_sync & ~sig_prev;
assign sig_fall = ~sig_sync & sig_prev;

// 高速计数器逻辑 (在200MHz时钟下运行)
generate
    for (genvar i = 0; i < 4; i = i + 1) begin : signal_measure
        always @(posedge clk_high or negedge rst_n) begin
            if (!rst_n) begin
                high_cnt[i] <= 32'd0;
                period_cnt[i] <= 32'd0;
                high_time[i] <= 32'd0;
                low_time[i] <= 32'd0;
            end else if (measure_en[i]) begin
                // 周期计数器一直运行
                period_cnt[i] <= period_cnt[i] + 1'b1;
                
                if (sig_sync[i]) begin
                    // 信号为高电平，计数高电平时间
                    high_cnt[i] <= high_cnt[i] + 1'b1;
                end
                
                // 上升沿检测到，完成一个周期
                if (sig_rise[i]) begin
                    // 保存高电平时间和低电平时间
                    high_time[i] <= high_cnt[i];
                    low_time[i] <= period_cnt[i] - high_cnt[i];
                    
                    // 重置计数器
                    high_cnt[i] <= 32'd0;
                    period_cnt[i] <= 32'd0;
                end
            end else begin
                high_cnt[i] <= 32'd0;
                period_cnt[i] <= 32'd0;
            end
        end
    end
endgenerate

// 计算频率和占空比 (在100MHz时钟下运行)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (integer i = 0; i < 4; i = i + 1) begin
            freq_reg[i] <= 32'd0;
            high_time_reg[i] <= 32'd0;
            low_time_reg[i] <= 32'd0;
            duty_reg[i] <= 32'd0;
        end
    end else begin
        for (integer i = 0; i < 4; i = i + 1) begin
            // 当检测到上升沿时更新测量结果
            if (sig_rise[i] && measure_en[i]) begin
                // 计算频率: 频率 = 200MHz / (周期计数)
                if (period_cnt[i] != 0) begin
                    freq_reg[i] <= 32'd200_000_000 / period_cnt[i];
                end else begin
                    freq_reg[i] <= 32'd0;
                end
                
                // 保存高低电平时间(单位: ns)
                high_time_reg[i] <= high_time[i] * 5;  // 200MHz时钟周期为5ns
                low_time_reg[i] <= low_time[i] * 5;
                
                // 计算占空比: (高电平时间 / 周期时间) * 10000
                if (period_cnt[i] != 0) begin
                    duty_reg[i] <= (high_time[i] * 32'd10000) / period_cnt[i];
                end else begin
                    duty_reg[i] <= 32'd0;
                end
            end
        end
    end
end

// 命令处理
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        measure_en <= 4'd0;
        tx_data <= 32'd0;
        tx_en <= 1'b0;
    end else begin
        if (cmd_valid) begin
            case (cmd_opcode)
                // 启动/停止测量
                8'h10: begin
                    measure_en <= cmd_data[3:0];
                end
                
                // 读取测量结果
                8'h11: begin
                    if (cmd_addr < 4) begin
                        case (cmd_data[1:0])
                            2'd0: tx_data <= freq_reg[cmd_addr];         // 频率(Hz)
                            2'd1: tx_data <= high_time_reg[cmd_addr];    // 高电平时间(ns)
                            2'd2: tx_data <= low_time_reg[cmd_addr];     // 低电平时间(ns)
                            2'd3: tx_data <= duty_reg[cmd_addr];         // 占空比(0.01%)
                        endcase
                        tx_en <= 1'b1;
                    end
                end
            endcase
        end else if (tx_done) begin
            tx_en <= 1'b0;
        end
    end
end

endmodule
