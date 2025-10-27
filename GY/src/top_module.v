// Top level to integrate modules provided in the project.
// This top module wires the key submodules together and exposes the ports
// used by the provided testbench (signal_debugger_tb.v).
//
// Notes:
// - The project contains multiple command/CDC parsing modules (command_parser,
//   usb_cdc_interface, serial_protocol_converter). For simplicity this top
//   instantiates command_parser and uses its parsed single-byte parameter
//   as a simple (minimal) command source to the submodules. The command
//   infrastructure in the repository is richer than this small glue and
//   can be replaced/extended as needed.
// - cdc_rx_data/cdc_rx_valid are tied to 0 here (no real USB CDC bytes driven).
//   To make the system respond in simulation, drive the command_parser inputs
//   from a proper CDC byte source or instantiate/use the usb_cdc_interface
//   parsing logic and route its outputs to a unified command bus.
// - pll_init is instantiated with PLL_INIT_BYPASS=1 to bypass MD interface
//   (typical in a simulation environment).
//
module top_module (
    input           sys_clk,        // system clock (from testbench)
    input           sys_rst_n,      // active low reset

    // USB UART / CDC-side physical serial line (simple)
    input           usb_rx,
    output          usb_tx,

    // SPI
    output          spi_sclk,
    output          spi_mosi,
    input           spi_miso,
    output [3:0]    spi_cs,

    // I2C (open-drain)
    inout           i2c_sda,
    inout           i2c_scl,

    // UART
    input           uart_rx,
    output          uart_tx,

    // PWM outputs
    output [7:0]    pwm_out,

    // Digital signal analyzer inputs
    input [3:0]     sig_in,

    // ADC interface (from external ADC)
    input  [11:0]   adc_data,
    output          adc_clk,
    output          adc_rst,

    // DAC interface (to external DAC)
    output [11:0]   dac_data,
    output          dac_clk,
    output          dac_rst,

    // RMII / Ethernet
    output          eth_tx_clk,
    output [3:0]    eth_txd,
    output          eth_tx_en,
    input           eth_rx_clk,
    input  [3:0]    eth_rxd,
    input           eth_rx_dv
);

// -----------------------------------------------------------------------------
// Clocking
// - Many modules in the repo expect two clocks: "clk" (100MHz in comments) and
//   "clk_high" (200MHz in comments). In this top we map sys_clk to both clk
//   and clk_high. For real hardware you should provide clocks at the required
//   frequencies. For simulation this is acceptable if relative timing isn't
//   strict.
//
wire clk       = sys_clk;
wire clk_high  = sys_clk;

// -----------------------------------------------------------------------------
// Minimal CDC / command source
// - The repository contains multiple command parsing modules. The provided
//   command_parser expects cdc_rx_data and cdc_rx_valid. Here we tie the
//   inputs to zero (no external CDC bytes). Replace these connections with a
//   real CDC byte source to make the whole control path functional.
//
wire [7:0] cdc_rx_data  = 8'h00;
wire       cdc_rx_valid = 1'b0;

// Instantiate command_parser to produce a simple command stream (single byte
// parameter output). This is a small bridge; other more feature-complete
// parsers (usb_cdc_interface) can be used in the future.
wire        cp_cmd_valid;
wire [7:0]  cp_cmd_opcode;
wire [2:0]  cp_cmd_periph;
wire [7:0]  cp_para_len;
wire [7:0]  cp_para_buf;
wire        cp_frame_error;
wire [1:0]  cp_error_type;

command_parser command_parser_inst (
    .clk            (clk),
    .rst_n          (sys_rst_n),
    .cdc_rx_data    (cdc_rx_data),
    .cdc_rx_valid   (cdc_rx_valid),
    .cmd_valid      (cp_cmd_valid),
    .cmd_opcode     (cp_cmd_opcode),
    .cmd_periph     (cp_cmd_periph),
    .para_len       (cp_para_len),
    .para_buf       (cp_para_buf),
    .frame_error    (cp_frame_error),
    .error_type     (cp_error_type)
);

// -----------------------------------------------------------------------------
// Command bus (simple translation)
// - Most functional modules accept a 32-bit cmd_data and 16-bit cmd_addr.
//   command_parser provides only opcode + first parameter byte in this
//   integration. We construct a minimal cmd bus where cmd_data carries the
//   parameter in its LSBs and cmd_addr = 0. For a complete system the
//   higher-level USB/CDC parser must deliver full opcode/addr/data fields.
//
reg  [7:0]  cmd_opcode;
reg [15:0]  cmd_addr;
reg [31:0]  cmd_data;
reg         cmd_valid;

// simple latch of command_parser outputs
always @(posedge clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cmd_opcode <= 8'd0;
        cmd_addr   <= 16'd0;
        cmd_data   <= 32'd0;
        cmd_valid  <= 1'b0;
    end else begin
        if (cp_cmd_valid) begin
            cmd_opcode <= cp_cmd_opcode;
            cmd_addr   <= 16'd0;
            // put the single-byte parameter into LSBs of cmd_data
            cmd_data   <= {24'd0, cp_para_buf};
            cmd_valid  <= 1'b1;
        end else begin
            // clear valid next cycle
            cmd_valid <= 1'b0;
        end
    end
end

// -----------------------------------------------------------------------------
// Instantiate modules and wire the simple command bus to them.
// -----------------------------------------------------------------------------
// Analog signal acquisition
wire [31:0] asa_tx_data;
wire        asa_tx_en;
wire        asa_tx_done; // not connected externally in this top

analog_signal_acquisition analog_signal_acquisition_inst (
    .clk            (clk),
    .clk_high       (clk_high),
    .rst_n          (sys_rst_n),

    .adc_data       (adc_data),
    .adc_clk        (adc_clk),
    .adc_rst        (adc_rst),

    .cmd_opcode     (cmd_opcode),
    .cmd_addr       (cmd_addr),
    .cmd_data       (cmd_data),
    .cmd_valid      (cmd_valid),

    .tx_data        (asa_tx_data),
    .tx_en          (asa_tx_en),
    .tx_done        (1'b0)
);

// Analog signal generator (DAC)
analog_signal_generator analog_signal_generator_inst (
    .clk            (clk),
    .clk_high       (clk_high),
    .rst_n          (sys_rst_n),

    .dac_data       (dac_data),
    .dac_clk        (dac_clk),
    .dac_rst        (dac_rst),

    .cmd_opcode     (cmd_opcode),
    .cmd_addr       (cmd_addr),
    .cmd_data       (cmd_data),
    .cmd_valid      (cmd_valid)
);

// Digital signal analyzer
wire [31:0] dsa_tx_data;
wire        dsa_tx_en;

digital_signal_analyzer digital_signal_analyzer_inst (
    .clk            (clk),
    .clk_high       (clk_high),
    .rst_n          (sys_rst_n),

    .sig_in         (sig_in),

    .cmd_opcode     (cmd_opcode),
    .cmd_addr       (cmd_addr),
    .cmd_data       (cmd_data),
    .cmd_valid      (cmd_valid),

    .tx_data        (dsa_tx_data),
    .tx_en          (dsa_tx_en),
    .tx_done        (1'b0)
);

// Serial protocol converter (SPI/I2C/UART/PWM)
wire [31:0] spc_tx_data;
wire        spc_tx_en;

serial_protocol_converter serial_protocol_converter_inst (
    .clk            (clk),
    .rst_n          (sys_rst_n),

    .cmd_opcode     (cmd_opcode),
    .cmd_addr       (cmd_addr),
    .cmd_data       (cmd_data),
    .cmd_valid      (cmd_valid),

    // SPI
    .spi_sclk       (spi_sclk),
    .spi_mosi       (spi_mosi),
    .spi_miso       (spi_miso),
    .spi_cs         (spi_cs),

    // I2C
    .i2c_sda        (i2c_sda),
    .i2c_scl        (i2c_scl),

    // UART
    .uart_rx        (uart_rx),
    .uart_tx        (uart_tx),

    // PWM
    .pwm_out        (pwm_out),

    // TX path
    .tx_data        (spc_tx_data),
    .tx_en          (spc_tx_en),
    .tx_done        (1'b0)
);

// Ethernet interface
eth_interface #(
    .LOCAL_MAC(48'h00_1A_2B_3C_4D_5E)
) eth_interface_inst (
    .clk            (clk),            // system clock used as rmii_tx_clk
    .rst_n          (sys_rst_n),

    // RMII input domain
    .rmii_rx_clk    (eth_rx_clk),
    .rmii_rxd       (eth_rxd[1:0]),    // NOTE: eth_interface expects 2-bit rmii_rxd;
                                              // testbench provides 4-bit bus. Use lower 2 bits.
    .rmii_crs_dv    (eth_rx_dv),

    // RMII output
    .rmii_tx_clk    (eth_tx_clk),
    .rmii_txd       (eth_txd[1:0]),    // note: mapping 2-bit output; higher bits tied low
    .rmii_tx_en     (eth_tx_en),

    // upper layer interface
    .tx_data        (8'h00),
    .tx_valid       (1'b0),
    .tx_last        (1'b0),
    .tx_ready       (/* open */),

    .rx_data        (/* open */),
    .rx_valid       (/* open */),
    .rx_last        (/* open */),
    .rx_error       (/* open */),

    .src_mac        (/* open */),
    .dst_mac        (/* open */),
    .eth_type       (/* open */),
    .frame_valid    (/* open */)
);

// The eth_interface in this repository expects 2-bit RMII data signals (RMII uses 2 bits).
// To keep port matching with the testbench which uses 4-bit eth_rxd/eth_txd,
// we map the lower two bits. If you need full 4-bit GMII/MII behavior,
// replace or adapt the eth module accordingly.

// -----------------------------------------------------------------------------
// PLL init (bypass in simulation)
// -----------------------------------------------------------------------------
wire pll_lock_in = 1'b1;
wire pll_lock_out;
wire md_rddata_unused;
wire [7:0] mdrdo_unused;

PLL_INIT #(
    .CLK_PERIOD (20),
    .MULTI_FAC  (24)
) pll_init_inst (
    .I_RST          (1'b0),
    .I_MD_CLK       (clk),
    .O_RST          (/* open */),
    .O_MD_INC       (/* open */),
    .O_MD_OPC       (/* open */),
    .O_MD_WR_DATA   (/* open */),
    .I_MD_RD_DATA   (8'h00),
    .I_LOCK         (pll_lock_in),
    .O_LOCK         (pll_lock_out),
    .PLL_INIT_BYPASS(1'b1),
    .MDRDO          (mdrdo_unused),
    .MDOPC          (2'b00),
    .MDAINC         (1'b0),
    .MDWDI          (8'h00)
);

// -----------------------------------------------------------------------------
// Simple USB transmit mapping
// - The repository includes richer CDC/USB building blocks. For this top we
//   provide a very small mapping: reflect a status bit onto usb_tx so the TB
//   can observe activity (if any). This is a placeholder.
// -----------------------------------------------------------------------------
assign usb_tx = 1'b1; // idle high for UART/CDC

// -----------------------------------------------------------------------------
// Final notes:
// - This top module focuses on structural integration. For a fully functional
//   system in simulation you should drive command_parser.cdc_rx_data/cdc_rx_valid
//   from a USB/CDC byte source, or use usb_cdc_interface (which itself requires
//   CDC byte-level inputs) and route parsed commands to the modules here.
// - Some ports in instantiated modules were left unconnected or partially
//   connected because the repo contains multiple overlapping control paths.
// -----------------------------------------------------------------------------

endmodule





/*module top_module (
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
gowin_pll u_gowin_pll (
    .clk_in     (sys_clk),      // 端口名以生成的v文件为准
    .reset      (!sys_rst_n),   // 端口名以生成的v文件为准
    .clk_out1   (clk_100m),
    .clk_out2   (clk_200m),
    .clk_out3   (clk_125m),
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

// 以太网接口模块 
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

endmodule*/
