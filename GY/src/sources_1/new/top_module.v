module top_module (
    // 系统时钟与复位
    input           sys_clk,        // 50MHz系统时钟
    input           sys_rst_n,      // 系统复位，低有效
    
    // USB接口
    input           usb_rx,         // USB接收数据
    output          usb_tx,         // USB发送数据
    
    // 串行协议接口
    // SPI
    output          spi_sclk,       // SPI时钟
    output          spi_mosi,       // SPI主机输出
    input           spi_miso,       // SPI主机输入
    output [3:0]    spi_cs,         // SPI片选（4路）
    
    // I2C
    inout           i2c_sda,        // I2C数据
    inout           i2c_scl,        // I2C时钟
    
    // UART
    input           uart_rx,        // UART接收
    output          uart_tx,        // UART发送
    
    // PWM输出
    output [7:0]    pwm_out,        // 8路PWM输出
    
    // 数字信号测量输入
    input [3:0]     sig_in,         // 4路测量信号输入
    
    // ADC接口 (LVDS)
    input [11:0]    adc_data,       // ADC数据
    output          adc_clk,        // ADC时钟
    output          adc_rst,        // ADC复位
    
    // DAC接口 (LVDS)
    output [11:0]   dac_data,       // DAC数据
    output          dac_clk,        // DAC时钟
    output          dac_rst,        // DAC复位
    
    // 以太网接口 (可选)
    output          eth_tx_clk,
    output [3:0]    eth_txd,
    output          eth_tx_en,
    input           eth_rx_clk,
    input [3:0]     eth_rxd,
    input           eth_rx_dv
);

// 内部信号连线
wire [1:0]       pll_locked;
wire             clk_100m;        // 100MHz系统时钟
wire             clk_200m;        // 200MHz高速时钟
wire             clk_125m;        // 125MHz以太网时钟

// 数据总线
wire [31:0]      data_rx;         // 接收数据总线
wire             data_rx_valid;   // 接收数据有效
wire [31:0]      data_tx;         // 发送数据总线
wire             data_tx_en;      // 发送数据使能
wire             data_tx_done;    // 发送数据完成

// 控制信号
wire [7:0]       cmd_opcode;      // 命令操作码
wire [15:0]      cmd_addr;        // 命令地址
wire [31:0]      cmd_data;        // 命令数据
wire             cmd_valid;       // 命令有效

// PLL模块，生成各模块所需时钟
pll_inst pll_inst (
    .clk_in1    (sys_clk),
    .rst        (!sys_rst_n),
    .clk_out1   (clk_100m),    // 100MHz
    .clk_out2   (clk_200m),    // 200MHz
    .clk_out3   (clk_125m),    // 125MHz
    .locked     (pll_locked)
);

// USB CDC接口模块
usb_cdc_interface usb_cdc_interface_inst (
    .clk        (clk_100m),
    .rst_n      (sys_rst_n & pll_locked[0]),
    .usb_rx     (usb_rx),
    .usb_tx     (usb_tx),
    .data_out   (data_rx),
    .data_valid (data_rx_valid),
    .data_in    (data_tx),
    .data_en    (data_tx_en),
    .data_done  (data_tx_done)
);

// 命令解析模块
command_parser command_parser_inst (
    .clk        (clk_100m),
    .rst_n      (sys_rst_n & pll_locked[0]),
    .data_in    (data_rx),
    .data_valid (data_rx_valid),
    .opcode     (cmd_opcode),
    .addr       (cmd_addr),
    .data       (cmd_data),
    .valid      (cmd_valid)
);

// 串行协议转换模块
serial_protocol_converter serial_protocol_converter_inst (
    .clk        (clk_100m),
    .rst_n      (sys_rst_n & pll_locked[0]),
    .cmd_opcode (cmd_opcode),
    .cmd_addr   (cmd_addr),
    .cmd_data   (cmd_data),
    .cmd_valid  (cmd_valid),
    .spi_sclk   (spi_sclk),
    .spi_mosi   (spi_mosi),
    .spi_miso   (spi_miso),
    .spi_cs     (spi_cs),
    .i2c_sda    (i2c_sda),
    .i2c_scl    (i2c_scl),
    .uart_rx    (uart_rx),
    .uart_tx    (uart_tx),
    .pwm_out    (pwm_out),
    .tx_data    (data_tx),
    .tx_en      (data_tx_en),
    .tx_done    (data_tx_done)
);

// 数字信号测量模块
digital_signal_analyzer digital_signal_analyzer_inst (
    .clk        (clk_100m),
    .clk_high   (clk_200m),  // 高速时钟用于精确测量
    .rst_n      (sys_rst_n & pll_locked[0]),
    .sig_in     (sig_in),
    .cmd_opcode (cmd_opcode),
    .cmd_addr   (cmd_addr),
    .cmd_data   (cmd_data),
    .cmd_valid  (cmd_valid),
    .tx_data    (data_tx),
    .tx_en      (data_tx_en),
    .tx_done    (data_tx_done)
);

// 模拟信号采集模块
analog_signal_acquisition analog_signal_acquisition_inst (
    .clk        (clk_100m),
    .clk_high   (clk_200m),
    .rst_n      (sys_rst_n & pll_locked[0]),
    .adc_data   (adc_data),
    .adc_clk    (adc_clk),
    .adc_rst    (adc_rst),
    .cmd_opcode (cmd_opcode),
    .cmd_addr   (cmd_addr),
    .cmd_data   (cmd_data),
    .cmd_valid  (cmd_valid),
    .tx_data    (data_tx),
    .tx_en      (data_tx_en),
    .tx_done    (data_tx_done)
);

// 模拟信号发生器模块
analog_signal_generator analog_signal_generator_inst (
    .clk        (clk_100m),
    .clk_high   (clk_200m),
    .rst_n      (sys_rst_n & pll_locked[0]),
    .dac_data   (dac_data),
    .dac_clk    (dac_clk),
    .dac_rst    (dac_rst),
    .cmd_opcode (cmd_opcode),
    .cmd_addr   (cmd_addr),
    .cmd_data   (cmd_data),
    .cmd_valid  (cmd_valid)
);

// 以太网接口模块 (可选)
eth_interface eth_interface_inst (
    .clk        (clk_125m),
    .rst_n      (sys_rst_n & pll_locked[1]),
    .tx_clk     (eth_tx_clk),
    .txd        (eth_txd),
    .tx_en      (eth_tx_en),
    .rx_clk     (eth_rx_clk),
    .rxd        (eth_rxd),
    .rx_dv      (eth_rx_dv),
    .data_in    (data_tx),
    .data_en    (data_tx_en),
    .data_out   (data_rx),
    .data_valid (data_rx_valid)
);

endmodule
