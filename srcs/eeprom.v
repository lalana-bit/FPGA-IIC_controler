`timescale 1ns / 1ps
// ============================================================
// eeprom_simple_test - 最简顶层：上电 → 写 0xA5 → 亮 LED
//
// 在你现有的 iic_ctrl + eeprom_rw 基础上，只做一件事：
//   向 EEPROM 子地址 0x0000 写入 0xA5
// 完成后拉高 led_done，停止。
//
// 用示波器抓 iic_scl / iic_sda 就能看到一帧完整的 IIC 写时序。
// ============================================================

module eeprom(
    input           clk,        // 板载时钟 (Basys3: 100MHz, iic_ctrl 内按 50MHz 算)
    input           rst_n,      // 复位按键，低有效

    inout           iic_sda,    // IIC 数据线 → PMOD / EEPROM
    output          iic_scl,    // IIC 时钟线
    output          iic_clk,    // 内部 4MHz 调试时钟输出

    output reg      led_done,   // 写完亮 = 事务完成
    output          led_error,  // 保留，未用

    output  [2:0]   iic_dbg_state,  // 当前 FSM 状态（调试用）
    output          iic_done_pulse,
    output          start_signal,
    
    output          scl_r,
    output          sda_r
);
assign scl_r = iic_scl;
assign sda_r = iic_sda;

// ============================================================
// 三状态 FSM
// ============================================================
localparam S_IDLE  = 4'd0;  // 等待上电稳定
localparam S_WRITE = 4'd1;  // 正在写 EEPROM
localparam S_PAUSE = 4'd2;
localparam S_READ  = 4'd4;
localparam S_DONE  = 4'd8;  // 完成，亮灯

reg [3:0] state;

wire [63:0]rd_data;

// 上电延时 ~1ms（50,000 拍 @ 50MHz），等硬件稳定
reg [19:0] delay_cnt;
wire       delay_done = (delay_cnt == 20'd50000);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        delay_cnt <= 0;
    else if (!delay_done)
        delay_cnt <= delay_cnt + 1;
end

// ============================================================
// 控制信号
// ============================================================
reg         start;
reg         wr_en;
reg         rd_en;
wire        complete;       // iic_ctrl 发出的 "事务结束" 脉冲
//wire [2:0]  iic_dbg_state;  // iic_ctrl 内部写状态（调试）

reg [17:0]pause_delay_cnt;
assign start_signal = start;

// ============================================================
// FSM 时序逻辑
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state    <= S_IDLE;
        start    <= 1'b0;
        wr_en    <= 1'b0;
        rd_en    <= 1'b0;
        led_done <= 1'b0;
    end else begin
        case (state)

            S_IDLE: begin
                if (delay_done) begin
                    start <= 1'b1;      // 发起 IIC 事务
                    wr_en <= 1'b1;      // 方向 = 写
                    state <= S_WRITE;
                end
            end

            S_WRITE: begin
                if (complete) begin
                    start    <= 1'b0;   // 撤回，防止再次触发
                    wr_en    <= 1'b0;
                    led_done <= 1'b1;   // ← 亮灯！
                    state    <= S_PAUSE;
                end
            end
            
            S_PAUSE: begin
                pause_delay_cnt <= (pause_delay_cnt == 150000 - 1) ? 18'd0 : pause_delay_cnt + 1;
                if(pause_delay_cnt == 150000 - 1) begin
                    start <= 1'b1;
                    rd_en <= 1'b1;
                    led_done <= 1'b0;
                    state <= S_READ;
                end
            end 
            
            S_READ: begin
                if(complete) begin
                    start <= 1'b0;
                    rd_en <= 1'b0;
                    led_done <= 1'b1;
                    state <= S_DONE;
                end
            end

            S_DONE: begin
                // 永远停在这里，LED 常亮
            end

            default: state <= S_IDLE;
        endcase
    end
end

// ============================================================
// 调试输出
// ============================================================
assign fsm_state     = state;
assign iic_done_pulse = complete;
assign led_error     = 1'b0;

// ============================================================
// 现成封装：eeprom_rw → iic_ctrl
// ============================================================
eeprom_rw #(
    .RW_SPEED ("fast"),
    .RD_MODE  (1),
    .RD_DATA_BYTE_NUM (8),
    .WR_DATA_BYTE_NUM (8)
) u_eeprom (
    .clk        (clk),
    .rst_n      (rst_n),
    .dev_addr   (7'b1010000),   // EEPROM I²C 从机地址
    .start      (start),
    .data_addr  (16'h0000),     // EEPROM 内部子地址
    .wr_en      (wr_en),
    .wr_data    (64'h0123456789ABCDEF),        // 写入数据
    .rd_en      (rd_en),
    .rd_data    (rd_data),
    .iic_scl    (iic_scl),
    .iic_sda    (iic_sda),
    .iic_clk    (iic_clk),
    .complete   (complete),
    .iic_state  (iic_dbg_state)
);

endmodule
