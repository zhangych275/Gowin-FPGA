module analog_signal_generator (
    input               clk,            // 100MHz主时钟
    input               clk_high,       // 200MHz高速时钟
    input               rst_n,
    
    // DAC接口
    output reg [11:0]   dac_data,       // DAC数据输出
    output reg          dac_clk,        // DAC时钟
    output reg          dac_rst,        // DAC复位
    
    // 命令接口
    input [7:0]         cmd_opcode,
    input [15:0]        cmd_addr,
    input [31:0]        cmd_data,
    input               cmd_valid
);

// 寄存器定义
reg [31:0]   dac_config_reg;     // DAC配置寄存器
reg [31:0]   freq_reg;           // 频率寄存器
reg [31:0]   amplitude_reg;      // 幅度寄存器
reg [31:0]   phase_reg;          // 相位寄存器
reg [31:0]   offset_reg;         // 偏移寄存器

// 内部信号
reg [11:0]   wave_data;          // 波形数据
reg [31:0]   phase_accum;        // 相位累加器
reg [31:0]   phase_inc;          // 相位增量
reg [9:0]    wave_index;         // 波形表索引
reg [11:0]   wave_rom [1023:0];  // 波形ROM (1024点)
reg          dac_en;             // DAC使能

// 波形类型定义
localparam SINE_WAVE = 2'd0;
localparam SQUARE_WAVE = 2'd1;
localparam TRIANGLE_WAVE = 2'd2;
localparam SAWTOOTH_WAVE = 2'd3;

// DAC时钟生成 (100MHz)
always @(posedge clk_high or negedge rst_n) begin
    if (!rst_n) begin
        dac_clk <= 1'b0;
    end else begin
        dac_clk <= ~dac_clk;  // 由200MHz分频得到100MHz DAC时钟
    end
end

// DAC复位控制
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dac_rst <= 1'b1;  // 复位有效
    end else if (cmd_valid && cmd_opcode == 8'h30 && cmd_data[0] == 1'b1) begin
        dac_rst <= 1'b1;  // 启动复位
    end else begin
        dac_rst <= 1'b0;  // 复位释放
    end
end

// 相位增量计算 (决定输出频率)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        phase_inc <= 32'd0;
    end else begin
        // 相位增量 = (2^32 * 输出频率) / 采样频率(100MHz)
        phase_inc <= (freq_reg * 32'd42949) >> 16;  // 近似计算
    end
end

// 相位累加器
always @(posedge dac_clk or negedge rst_n) begin
    if (!rst_n || !dac_en) begin
        phase_accum <= phase_reg;  // 初始相位
    end else begin
        phase_accum <= phase_accum + phase_inc;
    end
end

// 波形表索引
always @(posedge dac_clk or negedge rst_n) begin
    if (!rst_n) begin
        wave_index <= 10'd0;
    end else begin
        // 取相位累加器的高10位作为索引
        wave_index <= phase_accum[31:22];
    end
end

// 波形生成
always @(posedge dac_clk or negedge rst_n) begin
    if (!rst_n) begin
        wave_data <= 12'd0;
    end else if (dac_en) begin
        case (dac_config_reg[1:0])
            SINE_WAVE: begin
                // 从ROM中读取正弦波数据
                wave_data <= wave_rom[wave_index];
            end
            
            SQUARE_WAVE: begin
                // 方波: 前半周期高电平，后半周期低电平
                if (wave_index < 512) begin
                    wave_data <= 12'd4095;
                end else begin
                    wave_data <= 12'd0;
                end
            end
            
            TRIANGLE_WAVE: begin
                // 三角波
                if (wave_index < 512) begin
                    wave_data <= {1'b0, wave_index, 1'b0};
                end else begin
                    wave_data <= 12'd4095 - {1'b0, wave_index - 512, 1'b0};
                end
            end
            
            SAWTOOTH_WAVE: begin
                // 锯齿波
                wave_data <= {1'b0, wave_index, 1'b0};
            end
        endcase
        
        // 应用幅度和偏移
        wave_data <= wave_data * amplitude_reg[11:0] / 4095 + offset_reg[11:0];
    end else begin
        wave_data <= 12'd0;
    end
end

// DAC数据输出
always @(posedge dac_clk or negedge rst_n) begin
    if (!rst_n) begin
        dac_data <= 12'd0;
    end else begin
        dac_data <= wave_data;
    end
end

// 初始化正弦波ROM
  integer i;
initial begin
    
    for (i = 0; i < 1024; i = i + 1) begin
        // 生成12位正弦波数据 (0-4095)
        wave_rom[i] = 12'd2048 + $rtoi($sin(2 * 3.1415926 * i / 1024) * 2047);
    end
end

// 命令处理与寄存器配置
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dac_config_reg <= 32'd0;  // 默认正弦波
        freq_reg <= 32'd1000;     // 默认1kHz
        amplitude_reg <= 32'd4095; // 最大幅度
        phase_reg <= 32'd0;       // 初始相位0
        offset_reg <= 32'd0;      // 无偏移
        dac_en <= 1'b0;           // 禁用DAC
    end else if (cmd_valid) begin
        case (cmd_opcode)
            8'h31: dac_config_reg <= cmd_data;  // 配置寄存器(波形选择等)
            8'h32: freq_reg <= cmd_data;        // 频率设置(Hz)
            8'h33: amplitude_reg <= cmd_data;   // 幅度设置(0-4095)
            8'h34: phase_reg <= cmd_data;       // 相位设置
            8'h35: offset_reg <= cmd_data;      // 偏移设置
            8'h36: dac_en <= cmd_data[0];       // 启动/停止输出
            8'h37: begin                        // 写入自定义波形数据
                if (cmd_addr < 1024) begin
                    wave_rom[cmd_addr] <= cmd_data[11:0];
                end
            end
        endcase
    end
end

endmodule
