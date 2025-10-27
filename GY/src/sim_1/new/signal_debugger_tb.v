`timescale 1ns / 1ps

module fixed_enhanced_signal_debugger_tb();

    // 系统信号
    reg sys_clk;
    reg sys_rst_n;
    
    // USB接口信号
    reg usb_rx;
    wire usb_tx;
    
    // 串行协议接口
    wire spi_sclk;
    wire spi_mosi;
    reg spi_miso;
    wire [3:0] spi_cs;
    wire i2c_sda;
    wire i2c_scl;
    reg uart_rx;
    wire uart_tx;
    wire [7:0] pwm_out;
    
    // 数字信号测量输入
    reg [3:0] sig_in;
    
    // ADC接口
    reg [11:0] adc_data;
    wire adc_clk;
    wire adc_rst;
    
    // DAC接口
    wire [11:0] dac_data;
    wire dac_clk;
    wire dac_rst;
    
    // 以太网接口
    wire eth_tx_clk;
    wire [3:0] eth_txd;
    wire eth_tx_en;
    reg eth_rx_clk;
    reg [3:0] eth_rxd;
    reg eth_rx_dv;
    
    // 数字信号分析专用测试信号
    reg [7:0] parallel_bus;  // 并行总线测试信号
    reg frame_sync;          // 帧同步信号
    reg data_valid;          // 数据有效信号
    
    // 实例化被测试模块
    top_module uut (
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n),
        .usb_rx(usb_rx),
        .usb_tx(usb_tx),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs(spi_cs),
        .i2c_sda(i2c_sda),
        .i2c_scl(i2c_scl),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .pwm_out(pwm_out),
        .sig_in({frame_sync, data_valid, sig_in[1:0]}),  // 复用2路测量通道
        .adc_data(adc_data),
        .adc_clk(adc_clk),
        .adc_rst(adc_rst),
        .dac_data(dac_data),
        .dac_clk(dac_clk),
        .dac_rst(dac_rst),
        .eth_tx_clk(eth_tx_clk),
        .eth_txd(eth_txd),
        .eth_tx_en(eth_tx_en),
        .eth_rx_clk(eth_rx_clk),
        .eth_rxd(eth_rxd),
        .eth_rx_dv(eth_rx_dv)
    );
    
    // 生成系统时钟
    initial begin
        sys_clk = 0;
        forever #10 sys_clk = ~sys_clk; // 50MHz时钟
    end
    
    // 以太网接收时钟
    initial begin
        eth_rx_clk = 0;
        forever #8 eth_rx_clk = ~eth_rx_clk; // 约62.5MHz
    end
    
    // 测试主过程
    initial begin
        // 初始化
        sys_rst_n = 0;
        usb_rx = 1;
        spi_miso = 0;
        uart_rx = 1;
        sig_in = 4'b0000;
        adc_data = 12'h000;
        eth_rxd = 4'h0;
        eth_rx_dv = 0;
        parallel_bus = 8'h00;
        frame_sync = 0;
        data_valid = 0;
        
        // 释放复位
        #100;
        sys_rst_n = 1;
        
        // 等待PLL锁定
        #200;
        
        // 原有功能测试
        $display("开始基础功能测试...");
        test_spi_protocol();
        test_uart_protocol();
        test_pwm_generator();
        #1000;
        
        // 数字信号分析和调试功能测试
        $display("开始数字信号分析和调试功能测试...");
        test_signal_timing_analysis();      // 时序分析测试
        test_protocol_decoding();           // 协议解码测试
        test_triggered_sampling();          // 触发采样测试
        test_parallel_bus_analysis();       // 并行总线分析测试
        #1000;
        
        // 其他原有测试
        $display("开始模拟信号相关测试...");
        test_analog_acquisition();
        test_analog_generator();
        #1000;
        
        $display("所有测试完成!");
        $finish;
    end
    
    // 发送USB命令函数
    task send_usb_command;
        input [7:0] opcode;
        input [15:0] addr;
        input [31:0] data;
        begin
            // 命令帧格式:  opcode(8) + addr(16) + data(32)
            #10;
            $display("[%0t] 发送命令: opcode=0x%02X, addr=0x%04X, data=0x%08X", 
                     $time, opcode, addr, data);
            #100;
        end
    endtask
    
    // 时序分析测试
    task test_signal_timing_analysis;
        begin
            $display("\n=== 开始时序分析测试 ===");
            
            // 配置测量模块为高精度时序分析模式
            send_usb_command(8'h12, 16'h0000, 32'h00000001); // 使能高精度模式
            
            // 启动通道0和1的测量
            send_usb_command(8'h10, 16'h0000, 32'h00000003); // 使能通道0和1
            
            // 生成两个相关信号用于建立时间和保持时间测试
            fork
                // 生成参考时钟 (10MHz)
                begin : ref_clk_gen
                    forever begin
                        sig_in[0] = 1;
                        #50;
                        sig_in[0] = 0;
                        #50;
                    end
                end
                
                // 生成数据信号 (相对于时钟有延迟)
                begin : data_sig_gen
                    #20; // 20ns延迟
                    forever begin
                        sig_in[1] = 1;
                        #50;
                        sig_in[1] = 0;
                        #50;
                    end
                end
            join_none
            
            // 等待足够的测量时间
            #100000;
            
            // 读取时序参数
            send_usb_command(8'h13, 16'h0000, 32'h00000000); // 读取通道0-1的建立时间
            send_usb_command(8'h14, 16'h0000, 32'h00000000); // 读取通道0-1的保持时间
            send_usb_command(8'h15, 16'h0000, 32'h00000000); // 读取最大频率偏差
            
            // 停止生成测试信号
            disable ref_clk_gen;
            disable data_sig_gen;
            
            $display("=== 时序分析测试完成 ===");
        end
    endtask
    
    // 协议解码测试
    task test_protocol_decoding;
        begin
            $display("\n=== 开始协议解码测试 ===");
            
            // 配置协议分析器
            send_usb_command(8'h40, 16'h0000, 32'h00000001); // 使能SPI协议解码
            send_usb_command(8'h41, 16'h0000, 32'h00000008); // 设置SPI速率1MHz
            send_usb_command(8'h42, 16'h0000, 32'h00000000); // 设置为模式0 (CPOL=0, CPHA=0)
            
            // 生成SPI协议数据
            fork
                begin : spi_host_sim
                    // 模拟SPI主机发送数据
                    #1000;
                    // 注意：spi_cs在原始设计中是输出，不能在这里赋值
                    // 改为通过命令控制片选
                    send_usb_command(8'h01, 16'h0001, 32'h00000000); // 选中设备0
                    
                    // 发送数据帧: 0xAA -> 0x55 -> 0x33 -> 0xCC
                    send_spi_byte(8'hAA);
                    #10;
                    send_spi_byte(8'h55);
                    #10;
                    send_spi_byte(8'h33);
                    #10;
                    send_spi_byte(8'hCC);
                    
                    send_usb_command(8'h01, 16'h0001, 32'h0000000F); // 取消选中
                end
                
                // 模拟SPI从机响应
                begin : spi_slave_sim
                    #1020;
                    spi_miso = 1;
                    #20;
                    spi_miso = 0;
                    #20;
                    spi_miso = 1;
                    #20;
                    spi_miso = 0;
                    #20;
                    spi_miso = 1;
                end
            join_none
            
            // 等待解码完成
            #5000;
            
            // 读取解码结果
            send_usb_command(8'h43, 16'h0000, 32'h00000000); // 读取解码状态
            send_usb_command(8'h44, 16'h0000, 32'h00000000); // 读取第一个数据帧
            send_usb_command(8'h44, 16'h0001, 32'h00000000); // 读取第二个数据帧
            
            // 停止SPI模拟
            disable spi_host_sim;
            disable spi_slave_sim;
            
            $display("=== 协议解码测试完成 ===");
        end
    endtask
    
    // 辅助函数：发送SPI字节
    task send_spi_byte;
        input [7:0] data;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                // 注意：spi_sclk在原始设计中是输出，不能直接赋值
                // 这里改为通过监测spi_sclk的边沿来同步数据
                @(negedge spi_sclk);  // 等待时钟下降沿
                spi_miso = data[i];   // 在下降沿设置数据
            end
        end
    endtask
    
    // 触发采样测试
    task test_triggered_sampling;
        begin
            $display("\n=== 开始触发采样测试 ===");
            
            // 配置触发条件: 通道0上升沿 + 通道1高电平
            send_usb_command(8'h16, 16'h0000, 32'h00000001); // 触发使能
            send_usb_command(8'h17, 16'h0000, 32'h00000003); // 组合触发模式
            send_usb_command(8'h18, 16'h0000, 32'h00000001); // 通道0上升沿触发
            send_usb_command(8'h18, 16'h0001, 32'h00000002); // 通道1高电平触发
            send_usb_command(8'h19, 16'h0000, 32'h00000200); // 触发前后各512点数据
            
            // 生成触发信号序列
            fork
                begin : trig_ch0_gen
                    // 通道0信号 (低频方波)
                    #1000;
                    forever begin
                        sig_in[0] = 1;
                        #1000;
                        sig_in[0] = 0;
                        #1000;
                    end
                end
                
                begin : trig_ch1_gen
                    // 通道1信号 (大部分时间低，偶尔高)
                    #2500; // 错过第一次上升沿
                    sig_in[1] = 1;
                    #1500; // 与第三次上升沿重合
                    sig_in[1] = 0;
                end
            join_none
            
            // 等待触发和数据采集
            #10000;
            
            // 检查触发状态并读取数据
            send_usb_command(8'h1A, 16'h0000, 32'h00000000); // 读取触发状态
            send_usb_command(8'h1B, 16'h0000, 32'h00000000); // 读取触发位置
            send_usb_command(8'h1C, 16'h0000, 32'h00000000); // 读取触发前后数据
            
            // 停止触发信号生成
            disable trig_ch0_gen;
            disable trig_ch1_gen;
            
            $display("=== 触发采样测试完成 ===");
        end
    endtask
    
    // 并行总线分析测试
    task test_parallel_bus_analysis;
        begin
            $display("\n=== 开始并行总线分析测试 ===");
            
            // 配置并行总线分析器
            send_usb_command(8'h50, 16'h0000, 32'h00000001); // 使能并行总线分析
            send_usb_command(8'h51, 16'h0000, 32'h00000008); // 8位总线宽度
            send_usb_command(8'h52, 16'h0000, 32'h00000001); // 上升沿采样
            
            // 生成带帧同步的并行总线数据
            fork
                begin : parallel_data_gen
                    #1000;
                    // 帧1
                    frame_sync = 1;
                    #100;
                    frame_sync = 0;
                    
                    // 发送数据序列
                    send_parallel_data(8'h11);
                    send_parallel_data(8'h22);
                    send_parallel_data(8'h33);
                    send_parallel_data(8'h44);
                    
                    #500;
                    // 帧2
                    frame_sync = 1;
                    #100;
                    frame_sync = 0;
                    
                    send_parallel_data(8'hAA);
                    send_parallel_data(8'hBB);
                    send_parallel_data(8'hCC);
                    send_parallel_data(8'hDD);
                end
            join_none
            
            // 等待分析完成
            #5000;
            
            // 读取分析结果
            send_usb_command(8'h53, 16'h0000, 32'h00000000); // 读取帧计数
            send_usb_command(8'h54, 16'h0000, 32'h00000000); // 读取帧1数据
            send_usb_command(8'h54, 16'h0001, 32'h00000000); // 读取帧2数据
            send_usb_command(8'h55, 16'h0000, 32'h00000000); // 读取总线错误计数
            
            // 停止并行数据生成
            disable parallel_data_gen;
            
            $display("=== 并行总线分析测试完成 ===");
        end
    endtask
    
    // 辅助函数：发送并行数据
    task send_parallel_data;
        input [7:0] data;
        begin
            #20;
            parallel_bus = data;
            data_valid = 1;
            #50;
            data_valid = 0;
            #30;
        end
    endtask
    
    // 原有测试任务
    task test_spi_protocol;
        begin
            $display("\n=== 开始SPI协议测试 ===");
            send_usb_command(8'h01, 16'h0000, 32'h00000001); // SPI使能，模式0
            send_usb_command(8'h02, 16'h0000, 32'hA55A1234); // 发送数据
            #1000;
            send_usb_command(8'h08, 16'h0001, 32'h00000000); // 读取接收数据
        end
    endtask
    
    task test_uart_protocol;
        begin
            $display("\n=== 开始UART协议测试 ===");
            send_usb_command(8'h05, 16'h0000, 32'h00000003); // UART使能，115200
            send_usb_command(8'h06, 16'h0000, 32'h00000041); // 发送字符'A'
            #1000;
        end
    endtask
    
    task test_pwm_generator;
        begin
            $display("\n=== 开始PWM发生器测试 ===");
            send_usb_command(8'h07, 16'h0000, 32'h00640032); // PWM0: 周期100，占空比50
            #10000;
        end
    endtask
    
    task test_analog_acquisition;
        begin
            $display("\n=== 开始模拟信号采集测试 ===");
            send_usb_command(8'h20, 16'h0000, 32'h00000001); // 复位ADC
            #100;
            send_usb_command(8'h21, 16'h0000, 32'h00000400); // 设置采样点数1024
            #10000;
        end
    endtask
    
    task test_analog_generator;
        begin
            $display("\n=== 开始模拟信号发生器测试 ===");
            send_usb_command(8'h30, 16'h0000, 32'h00000001); // 复位DAC
            #100;
            send_usb_command(8'h31, 16'h0000, 32'h00000000); // 正弦波
            send_usb_command(8'h36, 16'h0000, 32'h00000001); // 启动输出
            #100000;
            send_usb_command(8'h36, 16'h0000, 32'h00000000); // 停止输出
        end
    endtask
    
    // 增强型监控
    initial begin
        $monitor("时间: %0t, 触发状态: %b, 帧同步: %b, 数据有效: %b, 并行数据: %h",
                 $time, uut.digital_signal_analyzer_inst.status_reg[3], 
                 frame_sync, data_valid, parallel_bus);
    end
    
    // 生成增强型波形文件
    initial begin
        $dumpfile("fixed_enhanced_signal_debugger.vcd");
        $dumpvars(0, fixed_enhanced_signal_debugger_tb);
    end

endmodule
