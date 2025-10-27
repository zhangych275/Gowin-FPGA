module top_module (
    // ϵͳʱ���븴λ
    input           sys_clk,        // 50MHzϵͳʱ��
    input           sys_rst_n,      // ϵͳ��λ������Ч
    
    // USB�ӿ�
    input           usb_rx,         // USB��������
    output          usb_tx,         // USB��������
    
    // ����Э��ӿ�
    // SPI
    output          spi_sclk,       // SPIʱ��
    output          spi_mosi,       // SPI�������
    input           spi_miso,       // SPI��������
    output [3:0]    spi_cs,         // SPIƬѡ��4·��
    
    // I2C
    inout           i2c_sda,        // I2C����
    inout           i2c_scl,        // I2Cʱ��
    
    // UART
    input           uart_rx,        // UART����
    output          uart_tx,        // UART����
    
    // PWM���
    output [7:0]    pwm_out,        // 8·PWM���
    
    // �����źŲ�������
    input [3:0]     sig_in,         // 4·�����ź�����
    
    // ADC�ӿ� (LVDS)
    input [11:0]    adc_data,       // ADC����
    output          adc_clk,        // ADCʱ��
    output          adc_rst,        // ADC��λ
    
    // DAC�ӿ� (LVDS)
    output [11:0]   dac_data,       // DAC����
    output          dac_clk,        // DACʱ��
    output          dac_rst,        // DAC��λ
    
    // ��̫���ӿ� (��ѡ)
    output          eth_tx_clk,
    output [3:0]    eth_txd,
    output          eth_tx_en,
    input           eth_rx_clk,
    input [3:0]     eth_rxd,
    input           eth_rx_dv
);

// �ڲ��ź�����
wire [1:0]       pll_locked;
wire             clk_100m;        // 100MHzϵͳʱ��
wire             clk_200m;        // 200MHz����ʱ��
wire             clk_125m;        // 125MHz��̫��ʱ��

// ��������
wire [31:0]      data_rx;         // ������������
wire             data_rx_valid;   // ����������Ч
wire [31:0]      data_tx;         // ������������
wire             data_tx_en;      // ��������ʹ��
wire             data_tx_done;    // �����������

// �����ź�
wire [7:0]       cmd_opcode;      // ���������
wire [15:0]      cmd_addr;        // �����ַ
wire [31:0]      cmd_data;        // ��������
wire             cmd_valid;       // ������Ч

// PLLģ�飬���ɸ�ģ������ʱ��
pll_inst pll_inst (
    .clk_in1    (sys_clk),
    .rst        (!sys_rst_n),
    .clk_out1   (clk_100m),    // 100MHz
    .clk_out2   (clk_200m),    // 200MHz
    .clk_out3   (clk_125m),    // 125MHz
    .locked     (pll_locked)
);

// USB CDC�ӿ�ģ��
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

// �������ģ��
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

// ����Э��ת��ģ��
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

// �����źŲ���ģ��
digital_signal_analyzer digital_signal_analyzer_inst (
    .clk        (clk_100m),
    .clk_high   (clk_200m),  // ����ʱ�����ھ�ȷ����
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

// ģ���źŲɼ�ģ��
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

// ģ���źŷ�����ģ��
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

// ��̫���ӿ�ģ�� (��ѡ)
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
