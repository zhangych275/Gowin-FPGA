`timescale 1ns / 1ps

module signal_debugger_tb();

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

    integer i;
    integer k;
    reg enable_sig0, enable_sig1;

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
        .sig_in(sig_in),
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

    // 有限次产生信号0 (1MHz 50%)
    initial begin
        enable_sig0 = 0;
        wait(sys_rst_n == 1);
        #5000;
        enable_sig0 = 1;
        for (k = 0; k < 1000; k = k + 1) begin
            sig_in[0] = 1;
            #500;
            sig_in[0] = 0;
            #500;
        end
        sig_in[0] = 0;
        enable_sig0 = 0;
    end

    // 有限次产生信号1 (2MHz 30%)
    initial begin
        enable_sig1 = 0;
        wait(sys_rst_n == 1);
        #5000;
        enable_sig1 = 1;
        for (k = 0; k < 1000; k = k + 1) begin
            sig_in[1] = 1;
            #150;
            sig_in[1] = 0;
            #350;
        end
        sig_in[1] = 0;
        enable_sig1 = 0;
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
        enable_sig0 = 0;
        enable_sig1 = 0;

        // 释放复位
        #100;
        sys_rst_n = 1;

        // 等待PLL锁定
        #200;

        // 测试1: 串行协议转换模块测试
        $display("开始测试串行协议转换模块...");
        test_spi_protocol();
        test_i2c_protocol();
        test_uart_protocol();
        test_pwm_generator();
        #1000;

        // 测试2: 数字信号测量模块测试
        $display("开始测试数字信号测量模块...");
        enable_sig0 = 1;
        enable_sig1 = 1;
        test_digital_measurement();
        #1000;
        enable_sig0 = 0;
        enable_sig1 = 0;

        // 测试3: 模拟信号采集模块测试
        $display("开始测试模拟信号采集模块...");
        test_analog_acquisition();
        #1000;

        // 测试4: 模拟信号发生器模块测试
        $display("开始测试模拟信号发生器模块...");
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
            #10;
            $display("发送命令: opcode=0x%02X, addr=0x%04X, data=0x%08X", opcode, addr, data);
            #100;
        end
    endtask

    // 测试SPI协议
    task test_spi_protocol;
        begin
            send_usb_command(8'h01, 16'h0000, 32'h00000001); // SPI使能，模式0
            send_usb_command(8'h02, 16'h0000, 32'hA55A1234); // 向SPI设备0发送数据

            // 模拟SPI从机返回数据
            spi_miso = 1;
            #100;
            spi_miso = 0;
            #100;
            spi_miso = 1;

            send_usb_command(8'h08, 16'h0001, 32'h00000000); // 读取SPI接收数据
        end
    endtask

    // 测试I2C协议
    task test_i2c_protocol;
        begin
            send_usb_command(8'h03, 16'h0000, 32'h00000400); // I2C使能，400kHz
            send_usb_command(8'h04, 16'h0048, 32'h12345678); // 向地址0x48发送数据
            send_usb_command(8'h08, 16'h0002, 32'h00000000); // 读取I2C接收数据
        end
    endtask

    // 测试UART协议
    task test_uart_protocol;
        begin
            send_usb_command(8'h05, 16'h0000, 32'h00000003); // UART使能，115200
            send_usb_command(8'h06, 16'h0000, 32'h00000041); // 发送字符'A'

            // 模拟UART输入数据
            #500;
            uart_rx = 0; // 起始位
            #8680; // 115200波特率的位时间
            uart_rx = 1; // 数据位0
            #8680;
            uart_rx = 0; // 数据位1
            #8680;
            uart_rx = 0; // 数据位2
            #8680;
            uart_rx = 0; // 数据位3
            #8680;
            uart_rx = 0; // 数据位4
            #8680;
            uart_rx = 1; // 数据位5
            #8680;
            uart_rx = 0; // 数据位6
            #8680;
            uart_rx = 0; // 数据位7 (字符'@')
            #8680;
            uart_rx = 1; // 停止位

            send_usb_command(8'h08, 16'h0003, 32'h00000000); // 读取UART接收数据
        end
    endtask

    // 测试PWM发生器
    task test_pwm_generator;
        begin
            send_usb_command(8'h07, 16'h0000, 32'h00640032); // PWM0: 周期100，占空比50
            send_usb_command(8'h07, 16'h0001, 32'h00C80032); // PWM1: 周期200，占空比25
            #10000;
        end
    endtask

    // 测试数字信号测量
    task test_digital_measurement;
        begin
            send_usb_command(8'h10, 16'h0000, 32'h0000000F); // 使能所有4路测量

            // 等待测量完成
            #100000;

            // 读取测量结果
            send_usb_command(8'h11, 16'h0000, 32'h00000000); // 读取通道0频率
            send_usb_command(8'h11, 16'h0000, 32'h00000003); // 读取通道0占空比
            send_usb_command(8'h11, 16'h0001, 32'h00000000); // 读取通道1频率
        end
    endtask

    // 测试模拟信号采集
    task test_analog_acquisition;
        begin
            send_usb_command(8'h20, 16'h0000, 32'h00000001);
            #100;
            send_usb_command(8'h20, 16'h0000, 32'h00000000);

            send_usb_command(8'h23, 16'h0000, 32'h00000000);
            send_usb_command(8'h24, 16'h0000, 32'h00098968);
            send_usb_command(8'h21, 16'h0000, 32'h00000400);

            // 生成模拟输入信号 (正弦波)
            for (i = 0; i < 2048; i = i + 1) begin
                @(posedge adc_clk);
                adc_data = 12'h800 + $rtoi($sin(2 * 3.1415926 * i / 128) * 1023);
            end

            #100000;
            send_usb_command(8'h26, 16'h0000, 32'h00000000); // 开始读取FIFO
            #10000;
            send_usb_command(8'h27, 16'h0000, 32'h00000000); // 结束读取
        end
    endtask

    // 测试模拟信号发生器
    task test_analog_generator;
        begin
            send_usb_command(8'h30, 16'h0000, 32'h00000001);
            #100;
            send_usb_command(8'h30, 16'h0000, 32'h00000000);

            send_usb_command(8'h31, 16'h0000, 32'h00000000); // 正弦波
            send_usb_command(8'h32, 16'h0000, 32'h000003E8); // 1kHz
            send_usb_command(8'h33, 16'h0000, 32'h00000FFF); // 幅度
            send_usb_command(8'h36, 16'h0000, 32'h00000001); // 启动输出
            #1000000;
            send_usb_command(8'h31, 16'h0000, 32'h00000001); // 方波
            #1000000;
            send_usb_command(8'h31, 16'h0000, 32'h00000002); // 三角波
            #1000000;
            send_usb_command(8'h36, 16'h0000, 32'h00000000);
        end
    endtask

    // 监控关键信号
    initial begin
        $monitor("时间: %0t, SPI: sclk=%b, mosi=%b, cs=%b, PWM: %b, DAC数据: %h",
                 $time, spi_sclk, spi_mosi, spi_cs, pwm_out, dac_data);
    end

    // 生成波形文件
    initial begin
        $dumpfile("signal_debugger.vcd");
        $dumpvars(0, signal_debugger_tb);
    end

endmodule






/*`timescale 1ns / 1ps

module signal_debugger_tb();

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
        .sig_in(sig_in),
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
        
        // 释放复位
        #100;
        sys_rst_n = 1;
        
        // 等待PLL锁定
        #200;
        
        // 测试1: 串行协议转换模块测试
        $display("开始测试串行协议转换模块...");
        test_spi_protocol();
        test_i2c_protocol();
        test_uart_protocol();
        test_pwm_generator();
        #1000;
        
        // 测试2: 数字信号测量模块测试
        $display("开始测试数字信号测量模块...");
        test_digital_measurement();
        #1000;
        
        // 测试3: 模拟信号采集模块测试
        $display("开始测试模拟信号采集模块...");
        test_analog_acquisition();
        #1000;
        
        // 测试4: 模拟信号发生器模块测试
        $display("开始测试模拟信号发生器模块...");
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
            // 简单的USB命令帧格式:  opcode(8) + addr(16) + data(32)
            // 使用简单的异步发送，实际应根据USB CDC协议实现
            #10;
            // 这里简化处理，实际应模拟UART异步传输
            $display("发送命令: opcode=0x%02X, addr=0x%04X, data=0x%08X", opcode, addr, data);
            // 模拟数据发送完成
            #100;
        end
    endtask
    
    // 测试SPI协议
    task test_spi_protocol;
        begin
            // 配置SPI
            send_usb_command(8'h01, 16'h0000, 32'h00000001); // SPI使能，模式0
            // 发送SPI数据
            send_usb_command(8'h02, 16'h0000, 32'hA55A1234); // 向SPI设备0发送数据
            
            // 模拟SPI从机返回数据
            spi_miso = 1;
            #100;
            spi_miso = 0;
            #100;
            spi_miso = 1;
            
            // 读取SPI接收数据
            send_usb_command(8'h08, 16'h0001, 32'h00000000); // 读取SPI接收数据
        end
    endtask
    
    // 测试I2C协议
    task test_i2c_protocol;
        begin
            // 配置I2C
            send_usb_command(8'h03, 16'h0000, 32'h00000400); // I2C使能，400kHz
            // 发送I2C数据
            send_usb_command(8'h04, 16'h0048, 32'h12345678); // 向地址0x48发送数据
            // 读取I2C接收数据
            send_usb_command(8'h08, 16'h0002, 32'h00000000); // 读取I2C接收数据
        end
    endtask
    
    // 测试UART协议
    task test_uart_protocol;
        begin
            // 配置UART (波特率115200)
            send_usb_command(8'h05, 16'h0000, 32'h00000003); // UART使能，115200
            // 发送UART数据
            send_usb_command(8'h06, 16'h0000, 32'h00000041); // 发送字符'A'
            
            // 模拟UART输入数据
            #500;
            uart_rx = 0; // 起始位
            #8680; // 115200波特率的位时间
            uart_rx = 1; // 数据位0
            #8680;
            uart_rx = 0; // 数据位1
            #8680;
            uart_rx = 0; // 数据位2
            #8680;
            uart_rx = 0; // 数据位3
            #8680;
            uart_rx = 0; // 数据位4
            #8680;
            uart_rx = 1; // 数据位5
            #8680;
            uart_rx = 0; // 数据位6
            #8680;
            uart_rx = 0; // 数据位7 (字符'@')
            #8680;
            uart_rx = 1; // 停止位
            
            // 读取UART接收数据
            send_usb_command(8'h08, 16'h0003, 32'h00000000); // 读取UART接收数据
        end
    endtask
    
    // 测试PWM发生器
    task test_pwm_generator;
        begin
            // 配置PWM0: 周期100，占空比50
            send_usb_command(8'h07, 16'h0000, 32'h00640032);
            // 配置PWM1: 周期200，占空比25
            send_usb_command(8'h07, 16'h0001, 32'h00C80032);
            // 等待PWM稳定
            #10000;
        end
    endtask
    
    // 测试数字信号测量
    task test_digital_measurement;
        begin
            // 启动信号测量
            send_usb_command(8'h10, 16'h0000, 32'h0000000F); // 使能所有4路测量
            
            // 生成测试信号1: 1MHz方波，50%占空比
            fork
                forever begin
                    sig_in[0] = 1;
                    #500;
                    sig_in[0] = 0;
                    #500;
                end
            join_none
            
            // 生成测试信号2: 2MHz方波，30%占空比
            fork
                forever begin
                    sig_in[1] = 1;
                    #150;
                    sig_in[1] = 0;
                    #350;
                end
            join_none
            
            // 等待测量完成
            #100000;
            
            // 读取测量结果
            send_usb_command(8'h11, 16'h0000, 32'h00000000); // 读取通道0频率
            send_usb_command(8'h11, 16'h0000, 32'h00000003); // 读取通道0占空比
            send_usb_command(8'h11, 16'h0001, 32'h00000000); // 读取通道1频率
        end
    endtask
    
    // 测试模拟信号采集
    task test_analog_acquisition;
        begin
            // 复位ADC
            send_usb_command(8'h20, 16'h0000, 32'h00000001);
            #100;
            send_usb_command(8'h20, 16'h0000, 32'h00000000);
            
            // 配置ADC为自由运行模式
            send_usb_command(8'h23, 16'h0000, 32'h00000000);
            // 设置采样率10MHz
            send_usb_command(8'h24, 16'h0000, 32'h00098968);
            // 设置采样点数1024
            send_usb_command(8'h21, 16'h0000, 32'h00000400);
            
            // 生成模拟输入信号 (正弦波)
            fork
                begin
                    integer i;
                    for (i = 0; i < 2048; i = i + 1) begin
                        @(posedge adc_clk);
                        adc_data = 12'h800 + $rtoi($sin(2 * 3.1415926 * i / 128) * 1023);
                    end
                end
            join_none
            
            // 等待采样完成
            #100000;
            
            // 读取采样数据
            send_usb_command(8'h26, 16'h0000, 32'h00000000); // 开始读取FIFO
            #10000;
            send_usb_command(8'h27, 16'h0000, 32'h00000000); // 结束读取
        end
    endtask
    
    // 测试模拟信号发生器
    task test_analog_generator;
        begin
            // 复位DAC
            send_usb_command(8'h30, 16'h0000, 32'h00000001);
            #100;
            send_usb_command(8'h30, 16'h0000, 32'h00000000);
            
            // 测试正弦波
            send_usb_command(8'h31, 16'h0000, 32'h00000000); // 正弦波
            send_usb_command(8'h32, 16'h0000, 32'h000003E8); // 1kHz
            send_usb_command(8'h33, 16'h0000, 32'h00000FFF); // 幅度
            send_usb_command(8'h36, 16'h0000, 32'h00000001); // 启动输出
            #1000000;
            
            // 测试方波
            send_usb_command(8'h31, 16'h0000, 32'h00000001); // 方波
            #1000000;
            
            // 测试三角波
            send_usb_command(8'h31, 16'h0000, 32'h00000002); // 三角波
            #1000000;
            
            // 停止输出
            send_usb_command(8'h36, 16'h0000, 32'h00000000);
        end
    endtask
    
    // 监控关键信号
    initial begin
        $monitor("时间: %0t, SPI: sclk=%b, mosi=%b, cs=%b, PWM: %b, DAC数据: %h",
                 $time, spi_sclk, spi_mosi, spi_cs, pwm_out, dac_data);
    end
    
    // 生成波形文件
    initial begin
        $dumpfile("signal_debugger.vcd");
        $dumpvars(0, signal_debugger_tb);
    end

endmodule
*/
