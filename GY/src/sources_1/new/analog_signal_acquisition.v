module analog_signal_acquisition (
    input               clk,            // 100MHz��ʱ��
    input               clk_high,       // 200MHz����ʱ��
    input               rst_n,
    
    // ADC�ӿ�
    input [11:0]        adc_data,       // ADC��������
    output reg          adc_clk,        // ADCʱ�����
    output reg          adc_rst,        // ADC��λ
    
    // ����ӿ�
    input [7:0]         cmd_opcode,
    input [15:0]        cmd_addr,
    input [31:0]        cmd_data,
    input               cmd_valid,
    
    // ���ݷ��ͽӿ�
    output reg [31:0]   tx_data,
    output reg          tx_en,
    input               tx_done
);

// �Ĵ�������
reg [31:0]   adc_config_reg;     // ADC���üĴ���
reg [31:0]   sample_rate_reg;    // �����ʼĴ���
reg [31:0]   sample_count_reg;   // ���������Ĵ���
reg [31:0]   trigger_level_reg;  // ������ƽ�Ĵ���
reg [31:0]   status_reg;         // ״̬�Ĵ���

// �ڲ��ź�
reg [11:0]   adc_data_sync;      // ͬ�����ADC����
reg          adc_sample_en;      // ADC����ʹ��
reg [31:0]   sample_cnt;         // ����������
reg [31:0]   sample_total;       // �ܲ�������
reg          trigger_detected;   // ��������־
reg          fifo_wr_en;         // FIFOдʹ��
reg [11:0]   fifo_wr_data;       // FIFOд������
wire         fifo_full;          // FIFO����־
wire         fifo_empty;         // FIFO�ձ�־
wire [11:0]  fifo_rd_data;       // FIFO��������
reg          fifo_rd_en;         // FIFO��ʹ��
reg [1:0]    adc_state;          // ADC״̬��

// ״̬����
localparam ADC_IDLE = 2'd0;
localparam ADC_SAMPLING = 2'd1;
localparam ADC_TRIGGERED = 2'd2;
localparam ADC_DONE = 2'd3;

// ADCʱ������ (100MHz)
always @(posedge clk_high or negedge rst_n) begin
    if (!rst_n) begin
        adc_clk <= 1'b0;
    end else begin
        adc_clk <= ~adc_clk;  // ��200MHz��Ƶ�õ�100MHz ADCʱ��
    end
end

// ADC��λ����
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_rst <= 1'b1;  // ��λ��Ч
    end else if (cmd_valid && cmd_opcode == 8'h20 && cmd_data[0] == 1'b1) begin
        adc_rst <= 1'b1;  // ������λ
    end else begin
        adc_rst <= 1'b0;  // ��λ�ͷ�
    end
end

// ADC����ͬ��
always @(posedge adc_clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_data_sync <= 12'd0;
    end else begin
        adc_data_sync <= adc_data;
    end
end

// �������
always @(posedge adc_clk or negedge rst_n) begin
    if (!rst_n) begin
        trigger_detected <= 1'b0;
    end else if (adc_sample_en && adc_state == ADC_SAMPLING) begin
        // �򵥵�ƽ�������
        if (adc_data_sync >= trigger_level_reg[11:0]) begin
            trigger_detected <= 1'b1;
        end else begin
            trigger_detected <= 1'b0;
        end
    end else begin
        trigger_detected <= 1'b0;
    end
end

// ����FIFOʵ���� (���4096)
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

// FIFOд�����
always @(posedge adc_clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_wr_en <= 1'b0;
        fifo_wr_data <= 12'd0;
        sample_cnt <= 32'd0;
    end else begin
        fifo_wr_en <= 1'b0;
        
        if (adc_sample_en) begin
            // ������ʹ��ʱ��д��FIFO
            if (!fifo_full) begin
                fifo_wr_en <= 1'b1;
                fifo_wr_data <= adc_data_sync;
                
                // �����Ѳ�������
                if (adc_state == ADC_TRIGGERED) begin
                    sample_cnt <= sample_cnt + 1'b1;
                end
            end
        end else begin
            sample_cnt <= 32'd0;
        end
    end
end

// ADC״̬��
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_state <= ADC_IDLE;
        adc_sample_en <= 1'b0;
        sample_total <= 32'd0;
    end else begin
        case (adc_state)
            ADC_IDLE: begin
                // �ȴ���������
                if (cmd_valid && cmd_opcode == 8'h21) begin
                    adc_sample_en <= 1'b1;
                    sample_total <= cmd_data;  // ���ò�������
                    adc_state <= ADC_SAMPLING;
                    status_reg[0] <= 1'b1;  // �����б�־
                end
            end
            
            ADC_SAMPLING: begin
                // �ȴ�����
                if (trigger_detected || (adc_config_reg[0] == 1'b0)) begin
                    // �����������������������ģʽ
                    adc_state <= ADC_TRIGGERED;
                    status_reg[1] <= 1'b1;  // ������־
                end
            end
            
            ADC_TRIGGERED: begin
                // ����ָ������������
                if (sample_cnt >= sample_total) begin
                    adc_sample_en <= 1'b0;
                    adc_state <= ADC_DONE;
                    status_reg[0] <= 1'b0;  // �������
                    status_reg[2] <= 1'b1;  // ���ݾ�����־
                end
            end
            
            ADC_DONE: begin
                // �ȴ����ݶ�ȡ���
                if (cmd_valid && cmd_opcode == 8'h22) begin
                    adc_state <= ADC_IDLE;
                    status_reg[1] <= 1'b0;
                    status_reg[2] <= 1'b0;
                end
            end
        endcase
    end
end

// �Ĵ�������
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_config_reg <= 32'd0;      // Ĭ��: ����ģʽ
        sample_rate_reg <= 32'd10_000_000;  // Ĭ��10Msps
        trigger_level_reg <= 32'd2048;  // �е㴥��
        status_reg <= 32'd0;
    end else if (cmd_valid) begin
        case (cmd_opcode)
            8'h23: adc_config_reg <= cmd_data;      // ���üĴ���
            8'h24: sample_rate_reg <= cmd_data;     // �����ʼĴ���
            8'h25: trigger_level_reg <= cmd_data;   // ������ƽ
        endcase
    end
end

// FIFO���ݶ����뷢��
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
                // �ȴ���ȡ����
                if (cmd_valid && cmd_opcode == 8'h26) begin
                    fifo_rd_en <= 1'b1;  // ��ʼ��ȡFIFO
                    tx_state <= TX_READ;
                end
            end
            
            TX_READ: begin
                fifo_rd_en <= 1'b0;
                // �ȴ�FIFO�������
                tx_data <= {20'd0, fifo_rd_data};  // ��չΪ32λ
                tx_en <= 1'b1;
                tx_state <= TX_SEND;
            end
            
            TX_SEND: begin
                if (tx_done) begin
                    tx_en <= 1'b0;
                    // ����Ƿ������ݻ��Ƿ���ɶ�ȡ
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
