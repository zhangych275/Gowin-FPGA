`timescale 1ns / 1ps

module fixed_enhanced_signal_debugger_tb();

    // ϵͳ�ź�
    reg sys_clk;
    reg sys_rst_n;
    
    // USB�ӿ��ź�
    reg usb_rx;
    wire usb_tx;
    
    // ����Э��ӿ�
    wire spi_sclk;
    wire spi_mosi;
    reg spi_miso;
    wire [3:0] spi_cs;
    wire i2c_sda;
    wire i2c_scl;
    reg uart_rx;
    wire uart_tx;
    wire [7:0] pwm_out;
    
    // �����źŲ�������
    reg [3:0] sig_in;
    
    // ADC�ӿ�
    reg [11:0] adc_data;
    wire adc_clk;
    wire adc_rst;
    
    // DAC�ӿ�
    wire [11:0] dac_data;
    wire dac_clk;
    wire dac_rst;
    
    // ��̫���ӿ�
    wire eth_tx_clk;
    wire [3:0] eth_txd;
    wire eth_tx_en;
    reg eth_rx_clk;
    reg [3:0] eth_rxd;
    reg eth_rx_dv;
    
    // �����źŷ���ר�ò����ź�
    reg [7:0] parallel_bus;  // �������߲����ź�
    reg frame_sync;          // ֡ͬ���ź�
    reg data_valid;          // ������Ч�ź�
    
    // ʵ����������ģ��
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
        .sig_in({frame_sync, data_valid, sig_in[1:0]}),  // ����2·����ͨ��
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
    
    // ����ϵͳʱ��
    initial begin
        sys_clk = 0;
        forever #10 sys_clk = ~sys_clk; // 50MHzʱ��
    end
    
    // ��̫������ʱ��
    initial begin
        eth_rx_clk = 0;
        forever #8 eth_rx_clk = ~eth_rx_clk; // Լ62.5MHz
    end
    
    // ����������
    initial begin
        // ��ʼ��
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
        
        // �ͷŸ�λ
        #100;
        sys_rst_n = 1;
        
        // �ȴ�PLL����
        #200;
        
        // ԭ�й��ܲ���
        $display("��ʼ�������ܲ���...");
        test_spi_protocol();
        test_uart_protocol();
        test_pwm_generator();
        #1000;
        
        // �����źŷ����͵��Թ��ܲ���
        $display("��ʼ�����źŷ����͵��Թ��ܲ���...");
        test_signal_timing_analysis();      // ʱ���������
        test_protocol_decoding();           // Э��������
        test_triggered_sampling();          // ������������
        test_parallel_bus_analysis();       // �������߷�������
        #1000;
        
        // ����ԭ�в���
        $display("��ʼģ���ź���ز���...");
        test_analog_acquisition();
        test_analog_generator();
        #1000;
        
        $display("���в������!");
        $finish;
    end
    
    // ����USB�����
    task send_usb_command;
        input [7:0] opcode;
        input [15:0] addr;
        input [31:0] data;
        begin
            // ����֡��ʽ:  opcode(8) + addr(16) + data(32)
            #10;
            $display("[%0t] ��������: opcode=0x%02X, addr=0x%04X, data=0x%08X", 
                     $time, opcode, addr, data);
            #100;
        end
    endtask
    
    // ʱ���������
    task test_signal_timing_analysis;
        begin
            $display("\n=== ��ʼʱ��������� ===");
            
            // ���ò���ģ��Ϊ�߾���ʱ�����ģʽ
            send_usb_command(8'h12, 16'h0000, 32'h00000001); // ʹ�ܸ߾���ģʽ
            
            // ����ͨ��0��1�Ĳ���
            send_usb_command(8'h10, 16'h0000, 32'h00000003); // ʹ��ͨ��0��1
            
            // ������������ź����ڽ���ʱ��ͱ���ʱ�����
            fork
                // ���ɲο�ʱ�� (10MHz)
                begin : ref_clk_gen
                    forever begin
                        sig_in[0] = 1;
                        #50;
                        sig_in[0] = 0;
                        #50;
                    end
                end
                
                // ���������ź� (�����ʱ�����ӳ�)
                begin : data_sig_gen
                    #20; // 20ns�ӳ�
                    forever begin
                        sig_in[1] = 1;
                        #50;
                        sig_in[1] = 0;
                        #50;
                    end
                end
            join_none
            
            // �ȴ��㹻�Ĳ���ʱ��
            #100000;
            
            // ��ȡʱ�����
            send_usb_command(8'h13, 16'h0000, 32'h00000000); // ��ȡͨ��0-1�Ľ���ʱ��
            send_usb_command(8'h14, 16'h0000, 32'h00000000); // ��ȡͨ��0-1�ı���ʱ��
            send_usb_command(8'h15, 16'h0000, 32'h00000000); // ��ȡ���Ƶ��ƫ��
            
            // ֹͣ���ɲ����ź�
            disable ref_clk_gen;
            disable data_sig_gen;
            
            $display("=== ʱ������������ ===");
        end
    endtask
    
    // Э��������
    task test_protocol_decoding;
        begin
            $display("\n=== ��ʼЭ�������� ===");
            
            // ����Э�������
            send_usb_command(8'h40, 16'h0000, 32'h00000001); // ʹ��SPIЭ�����
            send_usb_command(8'h41, 16'h0000, 32'h00000008); // ����SPI����1MHz
            send_usb_command(8'h42, 16'h0000, 32'h00000000); // ����Ϊģʽ0 (CPOL=0, CPHA=0)
            
            // ����SPIЭ������
            fork
                begin : spi_host_sim
                    // ģ��SPI������������
                    #1000;
                    // ע�⣺spi_cs��ԭʼ���������������������︳ֵ
                    // ��Ϊͨ���������Ƭѡ
                    send_usb_command(8'h01, 16'h0001, 32'h00000000); // ѡ���豸0
                    
                    // ��������֡: 0xAA -> 0x55 -> 0x33 -> 0xCC
                    send_spi_byte(8'hAA);
                    #10;
                    send_spi_byte(8'h55);
                    #10;
                    send_spi_byte(8'h33);
                    #10;
                    send_spi_byte(8'hCC);
                    
                    send_usb_command(8'h01, 16'h0001, 32'h0000000F); // ȡ��ѡ��
                end
                
                // ģ��SPI�ӻ���Ӧ
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
            
            // �ȴ��������
            #5000;
            
            // ��ȡ������
            send_usb_command(8'h43, 16'h0000, 32'h00000000); // ��ȡ����״̬
            send_usb_command(8'h44, 16'h0000, 32'h00000000); // ��ȡ��һ������֡
            send_usb_command(8'h44, 16'h0001, 32'h00000000); // ��ȡ�ڶ�������֡
            
            // ֹͣSPIģ��
            disable spi_host_sim;
            disable spi_slave_sim;
            
            $display("=== Э����������� ===");
        end
    endtask
    
    // ��������������SPI�ֽ�
    task send_spi_byte;
        input [7:0] data;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                // ע�⣺spi_sclk��ԭʼ����������������ֱ�Ӹ�ֵ
                // �����Ϊͨ�����spi_sclk�ı�����ͬ������
                @(negedge spi_sclk);  // �ȴ�ʱ���½���
                spi_miso = data[i];   // ���½�����������
            end
        end
    endtask
    
    // ������������
    task test_triggered_sampling;
        begin
            $display("\n=== ��ʼ������������ ===");
            
            // ���ô�������: ͨ��0������ + ͨ��1�ߵ�ƽ
            send_usb_command(8'h16, 16'h0000, 32'h00000001); // ����ʹ��
            send_usb_command(8'h17, 16'h0000, 32'h00000003); // ��ϴ���ģʽ
            send_usb_command(8'h18, 16'h0000, 32'h00000001); // ͨ��0�����ش���
            send_usb_command(8'h18, 16'h0001, 32'h00000002); // ͨ��1�ߵ�ƽ����
            send_usb_command(8'h19, 16'h0000, 32'h00000200); // ����ǰ���512������
            
            // ���ɴ����ź�����
            fork
                begin : trig_ch0_gen
                    // ͨ��0�ź� (��Ƶ����)
                    #1000;
                    forever begin
                        sig_in[0] = 1;
                        #1000;
                        sig_in[0] = 0;
                        #1000;
                    end
                end
                
                begin : trig_ch1_gen
                    // ͨ��1�ź� (�󲿷�ʱ��ͣ�ż����)
                    #2500; // �����һ��������
                    sig_in[1] = 1;
                    #1500; // ��������������غ�
                    sig_in[1] = 0;
                end
            join_none
            
            // �ȴ����������ݲɼ�
            #10000;
            
            // ��鴥��״̬����ȡ����
            send_usb_command(8'h1A, 16'h0000, 32'h00000000); // ��ȡ����״̬
            send_usb_command(8'h1B, 16'h0000, 32'h00000000); // ��ȡ����λ��
            send_usb_command(8'h1C, 16'h0000, 32'h00000000); // ��ȡ����ǰ������
            
            // ֹͣ�����ź�����
            disable trig_ch0_gen;
            disable trig_ch1_gen;
            
            $display("=== ��������������� ===");
        end
    endtask
    
    // �������߷�������
    task test_parallel_bus_analysis;
        begin
            $display("\n=== ��ʼ�������߷������� ===");
            
            // ���ò������߷�����
            send_usb_command(8'h50, 16'h0000, 32'h00000001); // ʹ�ܲ������߷���
            send_usb_command(8'h51, 16'h0000, 32'h00000008); // 8λ���߿��
            send_usb_command(8'h52, 16'h0000, 32'h00000001); // �����ز���
            
            // ���ɴ�֡ͬ���Ĳ�����������
            fork
                begin : parallel_data_gen
                    #1000;
                    // ֡1
                    frame_sync = 1;
                    #100;
                    frame_sync = 0;
                    
                    // ������������
                    send_parallel_data(8'h11);
                    send_parallel_data(8'h22);
                    send_parallel_data(8'h33);
                    send_parallel_data(8'h44);
                    
                    #500;
                    // ֡2
                    frame_sync = 1;
                    #100;
                    frame_sync = 0;
                    
                    send_parallel_data(8'hAA);
                    send_parallel_data(8'hBB);
                    send_parallel_data(8'hCC);
                    send_parallel_data(8'hDD);
                end
            join_none
            
            // �ȴ��������
            #5000;
            
            // ��ȡ�������
            send_usb_command(8'h53, 16'h0000, 32'h00000000); // ��ȡ֡����
            send_usb_command(8'h54, 16'h0000, 32'h00000000); // ��ȡ֡1����
            send_usb_command(8'h54, 16'h0001, 32'h00000000); // ��ȡ֡2����
            send_usb_command(8'h55, 16'h0000, 32'h00000000); // ��ȡ���ߴ������
            
            // ֹͣ������������
            disable parallel_data_gen;
            
            $display("=== �������߷���������� ===");
        end
    endtask
    
    // �������������Ͳ�������
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
    
    // ԭ�в�������
    task test_spi_protocol;
        begin
            $display("\n=== ��ʼSPIЭ����� ===");
            send_usb_command(8'h01, 16'h0000, 32'h00000001); // SPIʹ�ܣ�ģʽ0
            send_usb_command(8'h02, 16'h0000, 32'hA55A1234); // ��������
            #1000;
            send_usb_command(8'h08, 16'h0001, 32'h00000000); // ��ȡ��������
        end
    endtask
    
    task test_uart_protocol;
        begin
            $display("\n=== ��ʼUARTЭ����� ===");
            send_usb_command(8'h05, 16'h0000, 32'h00000003); // UARTʹ�ܣ�115200
            send_usb_command(8'h06, 16'h0000, 32'h00000041); // �����ַ�'A'
            #1000;
        end
    endtask
    
    task test_pwm_generator;
        begin
            $display("\n=== ��ʼPWM���������� ===");
            send_usb_command(8'h07, 16'h0000, 32'h00640032); // PWM0: ����100��ռ�ձ�50
            #10000;
        end
    endtask
    
    task test_analog_acquisition;
        begin
            $display("\n=== ��ʼģ���źŲɼ����� ===");
            send_usb_command(8'h20, 16'h0000, 32'h00000001); // ��λADC
            #100;
            send_usb_command(8'h21, 16'h0000, 32'h00000400); // ���ò�������1024
            #10000;
        end
    endtask
    
    task test_analog_generator;
        begin
            $display("\n=== ��ʼģ���źŷ��������� ===");
            send_usb_command(8'h30, 16'h0000, 32'h00000001); // ��λDAC
            #100;
            send_usb_command(8'h31, 16'h0000, 32'h00000000); // ���Ҳ�
            send_usb_command(8'h36, 16'h0000, 32'h00000001); // �������
            #100000;
            send_usb_command(8'h36, 16'h0000, 32'h00000000); // ֹͣ���
        end
    endtask
    
    // ��ǿ�ͼ��
    initial begin
        $monitor("ʱ��: %0t, ����״̬: %b, ֡ͬ��: %b, ������Ч: %b, ��������: %h",
                 $time, uut.digital_signal_analyzer_inst.status_reg[3], 
                 frame_sync, data_valid, parallel_bus);
    end
    
    // ������ǿ�Ͳ����ļ�
    initial begin
        $dumpfile("fixed_enhanced_signal_debugger.vcd");
        $dumpvars(0, fixed_enhanced_signal_debugger_tb);
    end

endmodule
