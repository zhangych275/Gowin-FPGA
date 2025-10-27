module eth_interface #(
    parameter LOCAL_MAC = 48'h00_1A_2B_3C_4D_5E,  // 本地MAC地址
    parameter MAX_FRAME_LEN = 1522,                // 最大帧长度（字节）
    parameter MIN_FRAME_LEN = 64                   // 最小帧长度（字节）
)(
    // 全局时钟与复位
    input  wire         clk,                       // 系统时钟（50MHz，RMII要求）
    input  wire         rst_n,                     // 异步复位（低有效）

    // RMII 物理层接口
    input  wire         rmii_rx_clk,               // RMII接收时钟（50MHz）
    input  wire [1:0]   rmii_rxd,                  // RMII接收数据
    input  wire         rmii_crs_dv,               // RMII载波检测/数据有效
    output wire         rmii_tx_clk,               // RMII发送时钟（50MHz，通常与clk同源）
    output reg  [1:0]   rmii_txd,                  // RMII发送数据
    output reg          rmii_tx_en,                // RMII发送使能

    // 上层数据接口（发送）
    input  wire [7:0]   tx_data,                   // 待发送数据
    input  wire         tx_valid,                  // 发送数据有效
    input  wire         tx_last,                   // 发送帧结束标记
    output reg          tx_ready,                  // 发送缓冲区就绪

    // 上层数据接口（接收）
    output reg  [7:0]   rx_data,                   // 接收数据 (clk 域输出)
    output reg          rx_valid,                  // 接收数据有效 (clk 域输出)
    output reg          rx_last,                   // 接收帧结束标记 (clk 域 output)
    output reg          rx_error,                  // 接收错误（校验错/长度错） (clk 域 output)

    // 帧信息输出
    output reg  [47:0]  src_mac,                   // 源MAC地址
    output reg  [47:0]  dst_mac,                   // 目的MAC地址
    output reg  [15:0]  eth_type,                  // 以太网类型（如0x0800=IP）
    output reg          frame_valid                // 帧解析有效（通过MAC过滤）
);

// --------------------------
// 1. 时钟与复位处理
// --------------------------
assign rmii_tx_clk = clk;  // 发送时钟与系统时钟同源（50MHz）

// 同步rmii_rx_clk域的复位信号
reg [1:0] rx_rst_sync;
always @(posedge rmii_rx_clk or negedge rst_n) begin
    if (!rst_n) rx_rst_sync <= 2'b00;
    else rx_rst_sync <= {rx_rst_sync[0], 1'b1};
end
wire rx_rst_n = rx_rst_sync[1];

// --------------------------
// 2. 接收逻辑（RMII -> 并行）
// --------------------------
// RMII接收数据为2位宽，需拼接为8位字节
reg [1:0] rx_data_reg;
reg [7:0] rx_byte;             // temporary assembling byte (rmii domain)
reg [1:0] rx_byte_cnt;         // 0-3：计数2位数据拼接为8位
reg       rx_en;               // 接收使能（同步rmii_crs_dv）

// 同步rmii_crs_dv到接收时钟域
reg [1:0] crs_dv_sync;
always @(posedge rmii_rx_clk or negedge rx_rst_n) begin
    if (!rx_rst_n) crs_dv_sync <= 2'b00;
    else crs_dv_sync <= {crs_dv_sync[0], rmii_crs_dv};
end
wire rx_active = crs_dv_sync[1];

// RMII domain intermediate outputs (do NOT drive top-level rx_data directly here)
reg [7:0] rx_byte_rmii;
reg       rx_valid_rmii;
reg       rx_last_rmii;
reg       rx_error_rmii;

// 拼接2位数据为8位字节 (RMII 时钟域)
always @(posedge rmii_rx_clk or negedge rx_rst_n) begin
    if (!rx_rst_n) begin
        rx_data_reg <= 2'b00;
        rx_byte_cnt <= 2'd0;
        rx_byte <= 8'h00;
        rx_en <= 1'b0;
        rx_byte_rmii <= 8'h00;
        rx_valid_rmii <= 1'b0;
        rx_last_rmii <= 1'b0;
        rx_error_rmii <= 1'b0;
    end else begin
        // clear the valid flag by default; will assert for one cycle when a byte is ready
        rx_valid_rmii <= 1'b0;
        rx_last_rmii <= 1'b0;
        if (rx_active) begin
            rx_data_reg <= rmii_rxd;
            // 每4个2位数据拼接为1个字节（little-endian）
            case (rx_byte_cnt)
                2'd0: rx_byte[1:0] <= rmii_rxd;
                2'd1: rx_byte[3:2] <= rmii_rxd;
                2'd2: rx_byte[5:4] <= rmii_rxd;
                2'd3: rx_byte[7:6] <= rmii_rxd;
            endcase
            rx_byte_cnt <= rx_byte_cnt + 1'b1;
            rx_en <= (rx_byte_cnt == 2'd3);  // 字节拼接下一时钟周期可认为完成
            if (rx_byte_cnt == 2'd3) begin
                // output assembled byte into rmii-domain intermediate regs (one-cycle valid)
                rx_byte_rmii <= {rmii_rxd, rx_byte[5:0]}; // reassembled current byte
                rx_valid_rmii <= 1'b1;
            end
        end else begin
            rx_byte_cnt <= 2'd0;
            rx_en <= 1'b0;
            // If the link went inactive and we had been receiving, mark last
            if (rx_byte_cnt != 2'd0) begin
                rx_last_rmii <= 1'b1;
            end
        end
    end
end

// --------------------------
// 3. 接收帧解析与过滤 (RMII -> frame fields)
//    ---- NOTE ----
// We do frame-level parsing also in rmii_rx_clk domain, but we must NOT drive
// top-level outputs (rx_data/rx_valid/...) from this domain. Instead the rmii
// domain writes intermediate signals (rx_byte_rmii / rx_valid_rmii / rx_last_rmii / rx_error_rmii)
// and the clk domain will synchronize them and drive module outputs.
// --------------------------
localparam RX_IDLE = 3'd0;
localparam RX_DST_MAC = 3'd1;  // 接收目的MAC（6字节）
localparam RX_SRC_MAC = 3'd2;  // 接收源MAC（6字节）
localparam RX_TYPE = 3'd3;     // 接收帧类型（2字节）
localparam RX_PAYLOAD = 3'd4;  // 接收 payload
localparam RX_CRC = 3'd5;      // 接收CRC（4字节，忽略）
localparam RX_CHECK = 3'd6;    // 帧校验

reg [2:0] rx_state_rmii;
reg [5:0] rx_byte_idx_rmii;         // 帧内字节计数
reg [47:0] dst_mac_reg;
reg [47:0] src_mac_reg;
reg [15:0] eth_type_reg;
reg [15:0] frame_len;          // 帧长度计数
reg [31:0] crc_reg;            // CRC接收寄存器（仅占位）
reg frame_valid_rmii;
reg rx_error_local_rmii;

always @(posedge rmii_rx_clk or negedge rx_rst_n) begin
    if (!rx_rst_n) begin
        rx_state_rmii <= RX_IDLE;
        rx_byte_idx_rmii <= 6'd0;
        dst_mac_reg <= 48'h00;
        src_mac_reg <= 48'h00;
        eth_type_reg <= 16'h00;
        frame_len <= 16'd0;
        crc_reg <= 32'h00;
        frame_valid_rmii <= 1'b0;
        rx_error_local_rmii <= 1'b0;
    end else begin
        // On each rmii clock, if a byte is ready (rx_valid_rmii asserted) process it
        if (rx_valid_rmii) begin
            case (rx_state_rmii)
                RX_IDLE: begin
                    // first byte is dest MAC low byte
                    dst_mac_reg[7:0] <= rx_byte_rmii;
                    rx_byte_idx_rmii <= 6'd1;
                    frame_len <= 16'd1;
                    rx_state_rmii <= RX_DST_MAC;
                    frame_valid_rmii <= 1'b0;
                    rx_error_local_rmii <= 1'b0;
                end

                RX_DST_MAC: begin
                    frame_len <= frame_len + 1'b1;
                    case (rx_byte_idx_rmii)
                        6'd1: dst_mac_reg[15:8]  <= rx_byte_rmii;
                        6'd2: dst_mac_reg[23:16] <= rx_byte_rmii;
                        6'd3: dst_mac_reg[31:24] <= rx_byte_rmii;
                        6'd4: dst_mac_reg[39:32] <= rx_byte_rmii;
                        6'd5: dst_mac_reg[47:40] <= rx_byte_rmii;
                    endcase
                    rx_byte_idx_rmii <= rx_byte_idx_rmii + 1'b1;
                    if (rx_byte_idx_rmii == 6'd5) begin
                        rx_state_rmii <= RX_SRC_MAC;
                        rx_byte_idx_rmii <= 6'd0;
                    end
                end

                RX_SRC_MAC: begin
                    frame_len <= frame_len + 1'b1;
                    case (rx_byte_idx_rmii)
                        6'd0: src_mac_reg[7:0]  <= rx_byte_rmii;
                        6'd1: src_mac_reg[15:8] <= rx_byte_rmii;
                        6'd2: src_mac_reg[23:16] <= rx_byte_rmii;
                        6'd3: src_mac_reg[31:24] <= rx_byte_rmii;
                        6'd4: src_mac_reg[39:32] <= rx_byte_rmii;
                        6'd5: src_mac_reg[47:40] <= rx_byte_rmii;
                    endcase
                    rx_byte_idx_rmii <= rx_byte_idx_rmii + 1'b1;
                    if (rx_byte_idx_rmii == 6'd5) begin
                        rx_state_rmii <= RX_TYPE;
                        rx_byte_idx_rmii <= 6'd0;
                    end
                end

                RX_TYPE: begin
                    frame_len <= frame_len + 1'b1;
                    if (rx_byte_idx_rmii == 6'd0) eth_type_reg[7:0] <= rx_byte_rmii;
                    else eth_type_reg[15:8] <= rx_byte_rmii;
                    rx_byte_idx_rmii <= rx_byte_idx_rmii + 1'b1;
                    if (rx_byte_idx_rmii == 6'd1) begin
                        // check destination MAC
                        if (dst_mac_reg == LOCAL_MAC || dst_mac_reg == 48'hFF_FF_FF_FF_FF_FF) begin
                            frame_valid_rmii <= 1'b1;
                            // store parsed fields so clk domain can pick them up later
                            dst_mac <= dst_mac_reg;
                            src_mac <= src_mac_reg;
                            eth_type <= eth_type_reg;
                            rx_state_rmii <= RX_PAYLOAD;
                        end else begin
                            // drop frame if not for us
                            rx_state_rmii <= RX_IDLE;
                        end
                        rx_byte_idx_rmii <= 6'd0;
                    end
                end

                RX_PAYLOAD: begin
                    frame_len <= frame_len + 1'b1;
                    // in rmii domain we do not attempt to push payload to the host here;
                    // instead we forward assembled bytes via rx_byte_rmii + rx_valid_rmii
                    // and signal last when active becomes false (handled below)
                    // if rx_active goes false, we will detect it via rx_last_rmii already set.
                    if (!rx_active) begin
                        rx_state_rmii <= RX_CRC;
                        rx_byte_idx_rmii <= 6'd0;
                    end
                end

                RX_CRC: begin
                    // accept CRC bytes (ignored)
                    // we don't know how many payload bytes were; this state receives CRC bytes
                    // via rx_valid_rmii. Accept 4 bytes to progress.
                    case (rx_byte_idx_rmii)
                        6'd0: crc_reg[7:0] <= rx_byte_rmii;
                        6'd1: crc_reg[15:8] <= rx_byte_rmii;
                        6'd2: crc_reg[23:16] <= rx_byte_rmii;
                        6'd3: crc_reg[31:24] <= rx_byte_rmii;
                    endcase
                    rx_byte_idx_rmii <= rx_byte_idx_rmii + 1'b1;
                    if (rx_byte_idx_rmii == 6'd3) begin
                        rx_state_rmii <= RX_CHECK;
                    end
                end

                RX_CHECK: begin
                    if (frame_len < MIN_FRAME_LEN || frame_len > MAX_FRAME_LEN) begin
                        rx_error_local_rmii <= 1'b1;
                    end
                    rx_state_rmii <= RX_IDLE;
                end

                default: rx_state_rmii <= RX_IDLE;
            endcase
        end
    end
end

// --------------------------
// 跨时钟域同步接收数据（rmii_rx_clk -> clk）
// We will synchronize intermediate rmii signals into clk domain.
// --------------------------
reg [7:0] rx_data_sync1, rx_data_sync2;
reg       rx_valid_sync1, rx_valid_sync2;
reg       rx_last_sync1, rx_last_sync2;
reg       rx_error_sync1, rx_error_sync2;
reg       frame_valid_sync1, frame_valid_sync2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_data_sync1 <= 8'h00;
        rx_data_sync2 <= 8'h00;
        rx_valid_sync1 <= 1'b0;
        rx_valid_sync2 <= 1'b0;
        rx_last_sync1 <= 1'b0;
        rx_last_sync2 <= 1'b0;
        rx_error_sync1 <= 1'b0;
        rx_error_sync2 <= 1'b0;
        frame_valid_sync1 <= 1'b0;
        frame_valid_sync2 <= 1'b0;
    end else begin
        // two-stage synchronizers (clock domain crossing)
        rx_data_sync1 <= rx_byte_rmii;
        rx_data_sync2 <= rx_data_sync1;
        rx_valid_sync1 <= rx_valid_rmii;
        rx_valid_sync2 <= rx_valid_sync1;
        rx_last_sync1 <= rx_last_rmii;
        rx_last_sync2 <= rx_last_sync1;
        rx_error_sync1 <= rx_error_local_rmii;
        rx_error_sync2 <= rx_error_sync1;
        frame_valid_sync1 <= frame_valid_rmii;
        frame_valid_sync2 <= frame_valid_sync1;
    end
end

// 输出到上层的接收信号（clk域）: 由 clk 域唯一驱动，避免多驱动
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_data <= 8'h00;
        rx_valid <= 1'b0;
        rx_last <= 1'b0;
        rx_error <= 1'b0;
        frame_valid <= 1'b0;
    end else begin
        rx_data <= rx_data_sync2;
        rx_valid <= rx_valid_sync2;
        rx_last <= rx_last_sync2;
        rx_error <= rx_error_sync2;
        frame_valid <= frame_valid_sync2;
    end
end

// --------------------------
// 4. 发送逻辑（并行 -> RMII）
// [保持原有实现，无修改]
// --------------------------
localparam TX_IDLE = 3'd0;
localparam TX_PRE = 3'd1;      // 发送前导码（7字节0x55）
localparam TX_SFD = 3'd2;      // 发送帧起始符（0xD5）
localparam TX_DST_MAC = 3'd3;  // 发送目的MAC
localparam TX_SRC_MAC = 3'd4;  // 发送源MAC
localparam TX_TYPE = 3'd5;     // 发送帧类型
localparam TX_PAYLOAD = 3'd6;  // 发送payload
localparam TX_CRC = 3'd7;      // 发送CRC（4字节）

reg [2:0] tx_state;
reg [5:0] tx_byte_idx;         // 发送字节计数
reg [1:0] tx_bit_idx;          // 2位数据计数（0-1）
reg [7:0] tx_byte_reg;         // 当前发送字节
reg [15:0] tx_len;             // 总发送长度计数
reg [31:0] tx_crc;             // 计算的CRC值

// 发送缓冲区就绪信号（仅在IDLE或发送完成时就绪）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) tx_ready <= 1'b0;
    else tx_ready <= (tx_state == TX_IDLE);
end

// 发送状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_state <= TX_IDLE;
        rmii_txd <= 2'b00;
        rmii_tx_en <= 1'b0;
        tx_byte_idx <= 6'd0;
        tx_bit_idx <= 2'd0;
        tx_byte_reg <= 8'h00;
        tx_len <= 16'd0;
        tx_crc <= 32'h00;
    end else begin
        case (tx_state)
            TX_IDLE: begin
                if (tx_valid) begin
                    // 启动发送，先发送前导码
                    tx_state <= TX_PRE;
                    tx_byte_reg <= 8'h55;  // 前导码
                    tx_byte_idx <= 6'd0;
                    tx_bit_idx <= 2'd0;
                    rmii_tx_en <= 1'b1;
                    tx_len <= 16'd0;
                end else begin
                    rmii_txd <= 2'b00;
                    rmii_tx_en <= 1'b0;
                end
            end

            TX_PRE: begin
                // 发送7字节前导码（0x55）
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];  // 每次发送2位
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin  // 1字节发送完成
                    tx_byte_idx <= tx_byte_idx + 1'b1;
                    tx_len <= tx_len + 1'b1;
                    if (tx_byte_idx == 6'd6) begin  // 7字节前导码完成
                        tx_state <= TX_SFD;
                        tx_byte_reg <= 8'hD5;  // 帧起始符
                        tx_byte_idx <= 6'd0;
                    end
                end
            end

            TX_SFD: begin
                // 发送1字节SFD（0xD5）
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_state <= TX_DST_MAC;
                    tx_byte_reg <= dst_mac[7:0];  // 目的MAC低8位
                    tx_byte_idx <= 6'd1;
                    tx_len <= tx_len + 1'b1;
                end
            end

            TX_DST_MAC: begin
                // 发送6字节目的MAC
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_len <= tx_len + 1'b1;
                    if (tx_byte_idx < 6'd5) begin
                        // 加载下一字节MAC
                        case (tx_byte_idx)
                            6'd1: tx_byte_reg <= dst_mac[15:8];
                            6'd2: tx_byte_reg <= dst_mac[23:16];
                            6'd3: tx_byte_reg <= dst_mac[31:24];
                            6'd4: tx_byte_reg <= dst_mac[39:32];
                            6'd5: tx_byte_reg <= dst_mac[47:40];
                        endcase
                        tx_byte_idx <= tx_byte_idx + 1'b1;
                    end else begin
                        // 目的MAC发送完成，切换到源MAC
                        tx_state <= TX_SRC_MAC;
                        tx_byte_reg <= LOCAL_MAC[7:0];  // 本地MAC低8位
                        tx_byte_idx <= 6'd1;
                    end
                end
            end

            TX_SRC_MAC: begin
                // 发送6字节源MAC（本地MAC）
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_len <= tx_len + 1'b1;
                    if (tx_byte_idx < 6'd5) begin
                        case (tx_byte_idx)
                            6'd1: tx_byte_reg <= LOCAL_MAC[15:8];
                            6'd2: tx_byte_reg <= LOCAL_MAC[23:16];
                            6'd3: tx_byte_reg <= LOCAL_MAC[31:24];
                            6'd4: tx_byte_reg <= LOCAL_MAC[39:32];
                            6'd5: tx_byte_reg <= LOCAL_MAC[47:40];
                        endcase
                        tx_byte_idx <= tx_byte_idx + 1'b1;
                    end else begin
                        // 源MAC发送完成，切换到帧类型
                        tx_state <= TX_TYPE;
                        tx_byte_reg <= eth_type[7:0];  // 帧类型低8位
                        tx_byte_idx <= 6'd0;
                    end
                end
            end

            TX_TYPE: begin
                // 发送2字节帧类型
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_len <= tx_len + 1'b1;
                    if (tx_byte_idx == 6'd0) begin
                        tx_byte_reg <= eth_type[15:8];  // 帧类型高8位
                        tx_byte_idx <= 6'd1;
                    end else begin
                        // 帧类型发送完成，切换到payload
                        tx_state <= TX_PAYLOAD;
                        tx_byte_reg <= tx_data;  // 加载首个payload字节
                    end
                end
            end

            TX_PAYLOAD: begin
                // 发送payload数据
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_len <= tx_len + 1'b1;
                    if (tx_valid) begin
                        // 继续接收上层数据
                        tx_byte_reg <= tx_data;
                        if (tx_last) begin
                            // payload结束，切换到CRC发送
                            tx_state <= TX_CRC;
                            tx_byte_reg <= tx_crc[7:0];  // CRC低8位
                            tx_byte_idx <= 6'd0;
                        end
                    end
                end
                // 此处应实时计算CRC（省略CRC计算逻辑）
            end

            TX_CRC: begin
                // 发送4字节CRC
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_len <= tx_len + 1'b1;
                    if (tx_byte_idx < 6'd3) begin
                        // 加载下一字节CRC
                        case (tx_byte_idx)
                            6'd0: tx_byte_reg <= tx_crc[15:8];
                            6'd1: tx_byte_reg <= tx_crc[23:16];
                            6'd2: tx_byte_reg <= tx_crc[31:24];
                        endcase
                        tx_byte_idx <= tx_byte_idx + 1'b1;
                    end else begin
                        // 发送完成，返回IDLE
                        tx_state <= TX_IDLE;
                        rmii_tx_en <= 1'b0;
                    end
                end
            end
        endcase
    end
end

endmodule

/*module eth_interface #(
    parameter LOCAL_MAC = 48'h00_1A_2B_3C_4D_5E,  // 本地MAC地址
    parameter MAX_FRAME_LEN = 1522,                // 最大帧长度（字节）
    parameter MIN_FRAME_LEN = 64                   // 最小帧长度（字节）
)(
    // 全局时钟与复位
    input  wire         clk,                       // 系统时钟（50MHz，RMII要求）
    input  wire         rst_n,                     // 异步复位（低有效）

    // RMII 物理层接口
    input  wire         rmii_rx_clk,               // RMII接收时钟（50MHz）
    input  wire [1:0]   rmii_rxd,                  // RMII接收数据
    input  wire         rmii_crs_dv,               // RMII载波检测/数据有效
    output wire         rmii_tx_clk,               // RMII发送时钟（50MHz，通常与clk同源）
    output reg  [1:0]   rmii_txd,                  // RMII发送数据
    output reg          rmii_tx_en,                // RMII发送使能

    // 上层数据接口（发送）
    input  wire [7:0]   tx_data,                   // 待发送数据
    input  wire         tx_valid,                  // 发送数据有效
    input  wire         tx_last,                   // 发送帧结束标记
    output reg          tx_ready,                  // 发送缓冲区就绪

    // 上层数据接口（接收）
    output reg  [7:0]   rx_data,                   // 接收数据
    output reg          rx_valid,                  // 接收数据有效
    output reg          rx_last,                   // 接收帧结束标记
    output reg          rx_error,                  // 接收错误（校验错/长度错）

    // 帧信息输出
    output reg  [47:0]  src_mac,                   // 源MAC地址
    output reg  [47:0]  dst_mac,                   // 目的MAC地址
    output reg  [15:0]  eth_type,                  // 以太网类型（如0x0800=IP）
    output reg          frame_valid                // 帧解析有效（通过MAC过滤）
);

// --------------------------
// 1. 时钟与复位处理
// --------------------------
assign rmii_tx_clk = clk;  // 发送时钟与系统时钟同源（50MHz）

// 同步rmii_rx_clk域的复位信号
reg [1:0] rx_rst_sync;
always @(posedge rmii_rx_clk or negedge rst_n) begin
    if (!rst_n) rx_rst_sync <= 2'b00;
    else rx_rst_sync <= {rx_rst_sync[0], 1'b1};
end
wire rx_rst_n = rx_rst_sync[1];

// --------------------------
// 2. 接收逻辑（RMII -> 并行）
// --------------------------
// RMII接收数据为2位宽，需拼接为8位字节
reg [1:0] rx_data_reg;
reg [7:0] rx_byte;
reg [1:0] rx_byte_cnt;  // 0-3：计数2位数据拼接为8位
reg       rx_en;        // 接收使能（同步rmii_crs_dv）

// 同步rmii_crs_dv到接收时钟域
reg [1:0] crs_dv_sync;
always @(posedge rmii_rx_clk or negedge rx_rst_n) begin
    if (!rx_rst_n) crs_dv_sync <= 2'b00;
    else crs_dv_sync <= {crs_dv_sync[0], rmii_crs_dv};
end
wire rx_active = crs_dv_sync[1];

// 拼接2位数据为8位字节
always @(posedge rmii_rx_clk or negedge rx_rst_n) begin
    if (!rx_rst_n) begin
        rx_data_reg <= 2'b00;
        rx_byte_cnt <= 2'd0;
        rx_byte <= 8'h00;
        rx_en <= 1'b0;
    end else begin
        if (rx_active) begin
            rx_data_reg <= rmii_rxd;
            // 每4个2位数据拼接为1个字节（little-endian）
            case (rx_byte_cnt)
                2'd0: rx_byte[1:0] <= rmii_rxd;
                2'd1: rx_byte[3:2] <= rmii_rxd;
                2'd2: rx_byte[5:4] <= rmii_rxd;
                2'd3: rx_byte[7:6] <= rmii_rxd;
            endcase
            rx_byte_cnt <= rx_byte_cnt + 1'b1;
            rx_en <= (rx_byte_cnt == 2'd3);  // 字节拼接完成
        end else begin
            rx_byte_cnt <= 2'd0;
            rx_en <= 1'b0;
        end
    end
end

// --------------------------
// 3. 接收帧解析与过滤
// --------------------------
localparam RX_IDLE = 3'd0;
localparam RX_DST_MAC = 3'd1;  // 接收目的MAC（6字节）
localparam RX_SRC_MAC = 3'd2;  // 接收源MAC（6字节）
localparam RX_TYPE = 3'd3;     // 接收帧类型（2字节）
localparam RX_PAYLOAD = 3'd4;  // 接收 payload
localparam RX_CRC = 3'd5;      // 接收CRC（4字节，忽略）
localparam RX_CHECK = 3'd6;    // 帧校验

reg [2:0] rx_state;
reg [5:0] rx_byte_idx;         // 帧内字节计数
reg [47:0] dst_mac_reg;
reg [47:0] src_mac_reg;
reg [15:0] eth_type_reg;
reg [15:0] frame_len;          // 帧长度计数
reg [31:0] crc_reg;            // CRC接收寄存器（仅占位）

always @(posedge rmii_rx_clk or negedge rx_rst_n) begin
    if (!rx_rst_n) begin
        rx_state <= RX_IDLE;
        rx_byte_idx <= 6'd0;
        dst_mac_reg <= 48'h00;
        src_mac_reg <= 48'h00;
        eth_type_reg <= 16'h00;
        frame_len <= 16'd0;
        crc_reg <= 32'h00;
        frame_valid <= 1'b0;
        rx_error <= 1'b0;
    end else begin
        rx_valid <= 1'b0;
        rx_last <= 1'b0;

        case (rx_state)
            RX_IDLE: begin
                if (rx_en) begin
                    // 开始接收帧，首个字节为目的MAC低8位
                    dst_mac_reg[7:0] <= rx_byte;
                    rx_byte_idx <= 6'd1;
                    frame_len <= 16'd1;
                    rx_state <= RX_DST_MAC;
                end
                frame_valid <= 1'b0;
                rx_error <= 1'b0;
            end

            RX_DST_MAC: begin
                if (rx_en) begin
                    frame_len <= frame_len + 1'b1;
                    // 拼接6字节目的MAC（little-endian）
                    case (rx_byte_idx)
                        6'd1: dst_mac_reg[15:8]  <= rx_byte;
                        6'd2: dst_mac_reg[23:16] <= rx_byte;
                        6'd3: dst_mac_reg[31:24] <= rx_byte;
                        6'd4: dst_mac_reg[39:32] <= rx_byte;
                        6'd5: dst_mac_reg[47:40] <= rx_byte;
                    endcase
                    rx_byte_idx <= rx_byte_idx + 1'b1;
                    if (rx_byte_idx == 6'd5) begin
                        rx_state <= RX_SRC_MAC;  // 目的MAC接收完成
                        rx_byte_idx <= 6'd0;
                    end
                end
            end

            RX_SRC_MAC: begin
                if (rx_en) begin
                    frame_len <= frame_len + 1'b1;
                    // 拼接6字节源MAC
                    case (rx_byte_idx)
                        6'd0: src_mac_reg[7:0]  <= rx_byte;
                        6'd1: src_mac_reg[15:8] <= rx_byte;
                        6'd2: src_mac_reg[23:16] <= rx_byte;
                        6'd3: src_mac_reg[31:24] <= rx_byte;
                        6'd4: src_mac_reg[39:32] <= rx_byte;
                        6'd5: src_mac_reg[47:40] <= rx_byte;
                    endcase
                    rx_byte_idx <= rx_byte_idx + 1'b1;
                    if (rx_byte_idx == 6'd5) begin
                        rx_state <= RX_TYPE;  // 源MAC接收完成
                        rx_byte_idx <= 6'd0;
                    end
                end
            end

            RX_TYPE: begin
                if (rx_en) begin
                    frame_len <= frame_len + 1'b1;
                    // 接收2字节帧类型（如0x0800=IP, 0x0806=ARP）
                    if (rx_byte_idx == 6'd0) eth_type_reg[7:0] <= rx_byte;
                    else eth_type_reg[15:8] <= rx_byte;
                    rx_byte_idx <= rx_byte_idx + 1'b1;
                    if (rx_byte_idx == 6'd1) begin
                        // 检查目的MAC是否匹配（本地MAC或广播MAC）
                        if (dst_mac_reg == LOCAL_MAC || dst_mac_reg == 48'hFF_FF_FF_FF_FF_FF) begin
                            frame_valid <= 1'b1;
                            dst_mac <= dst_mac_reg;
                            src_mac <= src_mac_reg;
                            eth_type <= eth_type_reg;
                            rx_state <= RX_PAYLOAD;
                        end else begin
                            rx_state <= RX_IDLE;  // MAC不匹配，丢弃帧
                        end
                        rx_byte_idx <= 6'd0;
                    end
                end
            end

            RX_PAYLOAD: begin
                if (rx_en) begin
                    frame_len <= frame_len + 1'b1;
                    // 输出payload数据到上层
                    rx_data <= rx_byte;
                    rx_valid <= 1'b1;
                    // 若帧结束（rx_active变为低），进入CRC阶段
                    if (!rx_active) begin
                        rx_state <= RX_CRC;
                        rx_byte_idx <= 6'd0;
                    end
                end
            end

            RX_CRC: begin
                if (rx_en) begin
                    // 接收4字节CRC（暂不校验，实际应与计算值比对）
                    case (rx_byte_idx)
                        6'd0: crc_reg[7:0] <= rx_byte;
                        6'd1: crc_reg[15:8] <= rx_byte;
                        6'd2: crc_reg[23:16] <= rx_byte;
                        6'd3: crc_reg[31:24] <= rx_byte;
                    endcase
                    rx_byte_idx <= rx_byte_idx + 1'b1;
                    if (rx_byte_idx == 6'd3) begin
                        rx_state <= RX_CHECK;
                    end
                end
            end

            RX_CHECK: begin
                // 校验帧长度是否合法
                if (frame_len < MIN_FRAME_LEN || frame_len > MAX_FRAME_LEN) begin
                    rx_error <= 1'b1;
                end
                rx_last <= 1'b1;  // 标记帧结束
                rx_state <= RX_IDLE;
            end
        endcase
    end
end

// 跨时钟域同步接收数据（rmii_rx_clk -> clk）
reg [7:0] rx_data_sync1, rx_data_sync2;
reg       rx_valid_sync1, rx_valid_sync2;
reg       rx_last_sync1, rx_last_sync2;
reg       rx_error_sync1, rx_error_sync2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_data_sync1 <= 8'h00;
        rx_data_sync2 <= 8'h00;
        rx_valid_sync1 <= 1'b0;
        rx_valid_sync2 <= 1'b0;
        rx_last_sync1 <= 1'b0;
        rx_last_sync2 <= 1'b0;
        rx_error_sync1 <= 1'b0;
        rx_error_sync2 <= 1'b0;
    end else begin
        // 两级同步
        rx_data_sync1 <= rx_data;
        rx_data_sync2 <= rx_data_sync1;
        rx_valid_sync1 <= rx_valid;
        rx_valid_sync2 <= rx_valid_sync1;
        rx_last_sync1 <= rx_last;
        rx_last_sync2 <= rx_last_sync1;
        rx_error_sync1 <= rx_error;
        rx_error_sync2 <= rx_error_sync1;
    end
end

// 输出到上层的接收信号（clk域）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_data <= 8'h00;
        rx_valid <= 1'b0;
        rx_last <= 1'b0;
        rx_error <= 1'b0;
    end else begin
        rx_data <= rx_data_sync2;
        rx_valid <= rx_valid_sync2;
        rx_last <= rx_last_sync2;
        rx_error <= rx_error_sync2;
    end
end

// --------------------------
// 4. 发送逻辑（并行 -> RMII）
// --------------------------
localparam TX_IDLE = 3'd0;
localparam TX_PRE = 3'd1;      // 发送前导码（7字节0x55）
localparam TX_SFD = 3'd2;      // 发送帧起始符（0xD5）
localparam TX_DST_MAC = 3'd3;  // 发送目的MAC
localparam TX_SRC_MAC = 3'd4;  // 发送源MAC
localparam TX_TYPE = 3'd5;     // 发送帧类型
localparam TX_PAYLOAD = 3'd6;  // 发送payload
localparam TX_CRC = 3'd7;      // 发送CRC（4字节）

reg [2:0] tx_state;
reg [5:0] tx_byte_idx;         // 发送字节计数
reg [1:0] tx_bit_idx;          // 2位数据计数（0-1）
reg [7:0] tx_byte_reg;         // 当前发送字节
reg [15:0] tx_len;             // 总发送长度计数
reg [31:0] tx_crc;             // 计算的CRC值

// 发送缓冲区就绪信号（仅在IDLE或发送完成时就绪）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) tx_ready <= 1'b0;
    else tx_ready <= (tx_state == TX_IDLE);
end

// 发送状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_state <= TX_IDLE;
        rmii_txd <= 2'b00;
        rmii_tx_en <= 1'b0;
        tx_byte_idx <= 6'd0;
        tx_bit_idx <= 2'd0;
        tx_byte_reg <= 8'h00;
        tx_len <= 16'd0;
        tx_crc <= 32'h00;
    end else begin
        case (tx_state)
            TX_IDLE: begin
                if (tx_valid) begin
                    // 启动发送，先发送前导码
                    tx_state <= TX_PRE;
                    tx_byte_reg <= 8'h55;  // 前导码
                    tx_byte_idx <= 6'd0;
                    tx_bit_idx <= 2'd0;
                    rmii_tx_en <= 1'b1;
                    tx_len <= 16'd0;
                end else begin
                    rmii_txd <= 2'b00;
                    rmii_tx_en <= 1'b0;
                end
            end

            TX_PRE: begin
                // 发送7字节前导码（0x55）
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];  // 每次发送2位
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin  // 1字节发送完成
                    tx_byte_idx <= tx_byte_idx + 1'b1;
                    tx_len <= tx_len + 1'b1;
                    if (tx_byte_idx == 6'd6) begin  // 7字节前导码完成
                        tx_state <= TX_SFD;
                        tx_byte_reg <= 8'hD5;  // 帧起始符
                        tx_byte_idx <= 6'd0;
                    end
                end
            end

            TX_SFD: begin
                // 发送1字节SFD（0xD5）
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_state <= TX_DST_MAC;
                    tx_byte_reg <= dst_mac[7:0];  // 目的MAC低8位
                    tx_byte_idx <= 6'd1;
                    tx_len <= tx_len + 1'b1;
                end
            end

            TX_DST_MAC: begin
                // 发送6字节目的MAC
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_len <= tx_len + 1'b1;
                    if (tx_byte_idx < 6'd5) begin
                        // 加载下一字节MAC
                        case (tx_byte_idx)
                            6'd1: tx_byte_reg <= dst_mac[15:8];
                            6'd2: tx_byte_reg <= dst_mac[23:16];
                            6'd3: tx_byte_reg <= dst_mac[31:24];
                            6'd4: tx_byte_reg <= dst_mac[39:32];
                            6'd5: tx_byte_reg <= dst_mac[47:40];
                        endcase
                        tx_byte_idx <= tx_byte_idx + 1'b1;
                    end else begin
                        // 目的MAC发送完成，切换到源MAC
                        tx_state <= TX_SRC_MAC;
                        tx_byte_reg <= LOCAL_MAC[7:0];  // 本地MAC低8位
                        tx_byte_idx <= 6'd1;
                    end
                end
            end

            TX_SRC_MAC: begin
                // 发送6字节源MAC（本地MAC）
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_len <= tx_len + 1'b1;
                    if (tx_byte_idx < 6'd5) begin
                        case (tx_byte_idx)
                            6'd1: tx_byte_reg <= LOCAL_MAC[15:8];
                            6'd2: tx_byte_reg <= LOCAL_MAC[23:16];
                            6'd3: tx_byte_reg <= LOCAL_MAC[31:24];
                            6'd4: tx_byte_reg <= LOCAL_MAC[39:32];
                            6'd5: tx_byte_reg <= LOCAL_MAC[47:40];
                        endcase
                        tx_byte_idx <= tx_byte_idx + 1'b1;
                    end else begin
                        // 源MAC发送完成，切换到帧类型
                        tx_state <= TX_TYPE;
                        tx_byte_reg <= eth_type[7:0];  // 帧类型低8位
                        tx_byte_idx <= 6'd0;
                    end
                end
            end

            TX_TYPE: begin
                // 发送2字节帧类型
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_len <= tx_len + 1'b1;
                    if (tx_byte_idx == 6'd0) begin
                        tx_byte_reg <= eth_type[15:8];  // 帧类型高8位
                        tx_byte_idx <= 6'd1;
                    end else begin
                        // 帧类型发送完成，切换到payload
                        tx_state <= TX_PAYLOAD;
                        tx_byte_reg <= tx_data;  // 加载首个payload字节
                    end
                end
            end

            TX_PAYLOAD: begin
                // 发送payload数据
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_len <= tx_len + 1'b1;
                    if (tx_valid) begin
                        // 继续接收上层数据
                        tx_byte_reg <= tx_data;
                        if (tx_last) begin
                            // payload结束，切换到CRC发送
                            tx_state <= TX_CRC;
                            tx_byte_reg <= tx_crc[7:0];  // CRC低8位
                            tx_byte_idx <= 6'd0;
                        end
                    end
                end
                // 此处应实时计算CRC（省略CRC计算逻辑）
            end

            TX_CRC: begin
                // 发送4字节CRC
                rmii_txd <= tx_byte_reg[tx_bit_idx*2 +: 2];
                tx_bit_idx <= tx_bit_idx + 1'b1;
                if (tx_bit_idx == 2'd1) begin
                    tx_len <= tx_len + 1'b1;
                    if (tx_byte_idx < 6'd3) begin
                        // 加载下一字节CRC
                        case (tx_byte_idx)
                            6'd0: tx_byte_reg <= tx_crc[15:8];
                            6'd1: tx_byte_reg <= tx_crc[23:16];
                            6'd2: tx_byte_reg <= tx_crc[31:24];
                        endcase
                        tx_byte_idx <= tx_byte_idx + 1'b1;
                    end else begin
                        // 发送完成，返回IDLE
                        tx_state <= TX_IDLE;
                        rmii_tx_en <= 1'b0;
                    end
                end
            end
        endcase
    end
end

// --------------------------
// 5. CRC计算模块（可选，简化）
// --------------------------
// 实际应用中需添加32位CRC计算逻辑（多项式0x04C11DB7）
// 此处省略，可通过实例化专用CRC模块实现

endmodule*/