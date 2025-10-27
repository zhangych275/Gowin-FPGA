module digital_signal_analyzer (
    input               clk,            // 100MHz��ʱ��
    input               clk_high,       // 200MHz����ʱ�ӣ����ھ�ȷ����
    input               rst_n,
    
    // �����ź�
    input [3:0]         sig_in,         // 4·�����ź�
    
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
reg [31:0]   freq_reg[3:0];      // Ƶ�ʲ�������Ĵ���
reg [31:0]   high_time_reg[3:0]; // �ߵ�ƽʱ��Ĵ���
reg [31:0]   low_time_reg[3:0];  // �͵�ƽʱ��Ĵ���
reg [31:0]   duty_reg[3:0];      // ռ�ձȼĴ���(�����֮һΪ��λ)
reg [3:0]    measure_en;         // ����ʹ�ܼĴ���

// �ڲ��ź�
reg [3:0]    sig_sync;           // ͬ����������ź�
reg [3:0]    sig_prev;           // ��һ���ڵ��ź�ֵ
wire [3:0]   sig_rise;           // �����ؼ��
wire [3:0]   sig_fall;           // �½��ؼ��

// ���ټ�����
reg [31:0]   high_cnt[3:0];      // ���ټ�����
reg [31:0]   period_cnt[3:0];    // ���ڼ�����
reg [31:0]   high_time[3:0];     // �ߵ�ƽʱ��
reg [31:0]   low_time[3:0];      // �͵�ƽʱ��

// �����ź�ͬ������ֹ metastability
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sig_sync <= 4'd0;
        sig_prev <= 4'd0;
    end else begin
        sig_sync <= sig_in;
        sig_prev <= sig_sync;
    end
end

// ���ؼ��
assign sig_rise = sig_sync & ~sig_prev;
assign sig_fall = ~sig_sync & sig_prev;

// ���ټ������߼� (��200MHzʱ��������)
generate
    for (genvar i = 0; i < 4; i = i + 1) begin : signal_measure
        always @(posedge clk_high or negedge rst_n) begin
            if (!rst_n) begin
                high_cnt[i] <= 32'd0;
                period_cnt[i] <= 32'd0;
                high_time[i] <= 32'd0;
                low_time[i] <= 32'd0;
            end else if (measure_en[i]) begin
                // ���ڼ�����һֱ����
                period_cnt[i] <= period_cnt[i] + 1'b1;
                
                if (sig_sync[i]) begin
                    // �ź�Ϊ�ߵ�ƽ�������ߵ�ƽʱ��
                    high_cnt[i] <= high_cnt[i] + 1'b1;
                end
                
                // �����ؼ�⵽�����һ������
                if (sig_rise[i]) begin
                    // ����ߵ�ƽʱ��͵͵�ƽʱ��
                    high_time[i] <= high_cnt[i];
                    low_time[i] <= period_cnt[i] - high_cnt[i];
                    
                    // ���ü�����
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

// ����Ƶ�ʺ�ռ�ձ� (��100MHzʱ��������)
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
            // ����⵽������ʱ���²������
            if (sig_rise[i] && measure_en[i]) begin
                // ����Ƶ��: Ƶ�� = 200MHz / (���ڼ���)
                if (period_cnt[i] != 0) begin
                    freq_reg[i] <= 32'd200_000_000 / period_cnt[i];
                end else begin
                    freq_reg[i] <= 32'd0;
                end
                
                // ����ߵ͵�ƽʱ��(��λ: ns)
                high_time_reg[i] <= high_time[i] * 5;  // 200MHzʱ������Ϊ5ns
                low_time_reg[i] <= low_time[i] * 5;
                
                // ����ռ�ձ�: (�ߵ�ƽʱ�� / ����ʱ��) * 10000
                if (period_cnt[i] != 0) begin
                    duty_reg[i] <= (high_time[i] * 32'd10000) / period_cnt[i];
                end else begin
                    duty_reg[i] <= 32'd0;
                end
            end
        end
    end
end

// �����
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        measure_en <= 4'd0;
        tx_data <= 32'd0;
        tx_en <= 1'b0;
    end else begin
        if (cmd_valid) begin
            case (cmd_opcode)
                // ����/ֹͣ����
                8'h10: begin
                    measure_en <= cmd_data[3:0];
                end
                
                // ��ȡ�������
                8'h11: begin
                    if (cmd_addr < 4) begin
                        case (cmd_data[1:0])
                            2'd0: tx_data <= freq_reg[cmd_addr];         // Ƶ��(Hz)
                            2'd1: tx_data <= high_time_reg[cmd_addr];    // �ߵ�ƽʱ��(ns)
                            2'd2: tx_data <= low_time_reg[cmd_addr];     // �͵�ƽʱ��(ns)
                            2'd3: tx_data <= duty_reg[cmd_addr];         // ռ�ձ�(0.01%)
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
