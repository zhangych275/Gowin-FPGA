module serial_protocol_converter (
    input               clk,
    input               rst_n,
    
    // ����ӿ�
    input [7:0]         cmd_opcode,
    input [15:0]        cmd_addr,
    input [31:0]        cmd_data,
    input               cmd_valid,
    
    // SPI�ӿ�
    output reg          spi_sclk,
    output reg          spi_mosi,
    input               spi_miso,
    output reg [3:0]    spi_cs,
    
    // I2C�ӿ�
    inout               i2c_sda,
    inout               i2c_scl,
    
    // UART�ӿ�
    input               uart_rx,
    output              uart_tx,
    
    // PWM���
    output reg [7:0]    pwm_out,
    
    // �������ݽӿ�
    output reg [31:0]   tx_data,
    output reg          tx_en,
    input               tx_done
);

// �Ĵ�������
reg [31:0]   spi_config_reg;  // SPI���üĴ���
reg [31:0]   spi_tx_reg;      // SPI���ͼĴ���
reg [31:0]   spi_rx_reg;      // SPI���ռĴ���
reg [31:0]   i2c_config_reg;  // I2C���üĴ���
reg [31:0]   i2c_tx_reg;      // I2C���ͼĴ���
reg [31:0]   i2c_rx_reg;      // I2C���ռĴ���
reg [31:0]   uart_config_reg; // UART���üĴ���
reg [31:0]   uart_tx_reg;     // UART���ͼĴ���
reg [31:0]   uart_rx_reg;     // UART���ռĴ���
reg [31:0]   pwm_config_reg[7:0]; // PWM���üĴ���(8·)

// ״̬������
localparam IDLE = 3'd0;
localparam SPI_TX = 3'd1;
localparam SPI_RX = 3'd2;
localparam I2C_TX = 3'd3;
localparam I2C_RX = 3'd4;
localparam UART_TX = 3'd5;
localparam UART_RX = 3'd6;

reg [2:0] current_state, next_state;

// ���������Ĵ�����д
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // ��ʼ�����мĴ���
        spi_config_reg <= 32'd0;
        spi_tx_reg <= 32'd0;
        i2c_config_reg <= 32'd0;
        i2c_tx_reg <= 32'd0;
        uart_config_reg <= 32'd0;
        uart_tx_reg <= 32'd0;

        for (integer i = 0; i < 8; i = i + 1) begin
            pwm_config_reg[i] <= 32'd0;
        end
    end else if (cmd_valid) begin
        case (cmd_opcode)
            // SPI����
            8'h01: spi_config_reg <= cmd_data;
            // SPI��������
            8'h02: spi_tx_reg <= cmd_data;
            // I2C����
            8'h03: i2c_config_reg <= cmd_data;
            // I2C��������
            8'h04: i2c_tx_reg <= cmd_data;
            // UART����
            8'h05: uart_config_reg <= cmd_data;
            // UART��������
            8'h06: uart_tx_reg <= cmd_data;
            // PWM����
            8'h07: begin
                if (cmd_addr < 8) begin
                    pwm_config_reg[cmd_addr] <= cmd_data;
                end
            end
            // ��ȡ��������
            8'h08: begin
                case (cmd_addr)
                    16'h0001: tx_data <= spi_rx_reg;
                    16'h0002: tx_data <= i2c_rx_reg;
                    16'h0003: tx_data <= uart_rx_reg;
                endcase
                tx_en <= 1'b1;
            end
        endcase
    end else if (tx_done) begin
        tx_en <= 1'b0;
    end
end

// SPI������ʵ��
reg [5:0] spi_bit_cnt;
reg [31:0] spi_shift_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        spi_sclk <= 1'b0;
        spi_mosi <= 1'b0;
        spi_cs <= 4'hF;
        spi_bit_cnt <= 6'd0;
        spi_shift_reg <= 32'd0;
        spi_rx_reg <= 32'd0;
    end else begin
        case (current_state)
            IDLE: begin
                spi_sclk <= 1'b0;
                if (cmd_valid && cmd_opcode == 8'h02) begin
                    // ����SPI����
                    spi_cs <= ~(1 << (cmd_addr[1:0])); // ѡ���Ӧ��SPI�豸
                    spi_shift_reg <= spi_tx_reg;
                    spi_bit_cnt <= 6'd31;
                    current_state <= SPI_TX;
                end
            end
            
            SPI_TX: begin
                // ʵ��SPI�����߼�������ʱ�����ɺ�������λ
                spi_sclk <= ~spi_sclk;
                if (spi_sclk) begin
                    // ʱ���½��أ���������
                    spi_mosi <= spi_shift_reg[31];
                    spi_shift_reg <= {spi_shift_reg[30:0], spi_miso};
                    if (spi_bit_cnt == 0) begin
                        current_state <= IDLE;
                        spi_cs <= 4'hF;
                        spi_rx_reg <= {spi_shift_reg[30:0], spi_miso};
                    end else begin
                        spi_bit_cnt <= spi_bit_cnt - 1'b1;
                    end
                end
            end
            
            // ����״̬ʵ��...
            default: current_state <= IDLE;
        endcase
    end
end

// I2C������ʵ����
i2c_controller i2c_controller_inst (
    .clk        (clk),
    .rst_n      (rst_n),
    .config_reg (i2c_config_reg),
    .tx_data    (i2c_tx_reg),
    .rx_data    (i2c_rx_reg),
    .start      (cmd_valid && cmd_opcode == 8'h04),
    .done       (i2c_done),
    .sda        (i2c_sda),
    .scl        (i2c_scl)
);

// UART������ʵ����
uart_controller uart_controller_inst (
    .clk        (clk),
    .rst_n      (rst_n),
    .config_reg (uart_config_reg),
    .tx_data    (uart_tx_reg),
    .rx_data    (uart_rx_reg),
    .tx_start   (cmd_valid && cmd_opcode == 8'h06),
    .tx_done    (uart_tx_done),
    .rx_ready   (uart_rx_ready),
    .tx         (uart_tx),
    .rx         (uart_rx)
);

// PWM������
generate
    for (genvar i = 0; i < 8; i = i + 1) begin : pwm_generator
        pwm_module pwm_module_inst (
            .clk        (clk),
            .rst_n      (rst_n),
            .period     (pwm_config_reg[i][31:16]),
            .duty       (pwm_config_reg[i][15:0]),
            .pwm_out    (pwm_out[i])
        );
    end
endgenerate

// ״̬��ת���߼�
always @(*) begin
    case (current_state)
        IDLE: begin
            if (cmd_valid && cmd_opcode == 8'h02)
                next_state = SPI_TX;
            else if (cmd_valid && cmd_opcode == 8'h04)
                next_state = I2C_TX;
            else if (cmd_valid && cmd_opcode == 8'h06)
                next_state = UART_TX;
            else
                next_state = IDLE;
        end
        
        SPI_TX: begin
            if (spi_bit_cnt == 0 && spi_sclk)
                next_state = IDLE;
            else
                next_state = SPI_TX;
        end
        
        // ����״̬ת���߼�...
        default: next_state = IDLE;
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        current_state <= IDLE;
    else
        current_state <= next_state;
end

endmodule
