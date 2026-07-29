`timescale 1ns / 1ps

module eeprom(
    input           clk,        
    input           rst_n,      

    inout           iic_sda,    
    output          iic_scl,    
    output          iic_clk,    

    output reg      led_done,   
    output          led_error,  

    output  [2:0]   iic_dbg_state,  
    output          iic_done_pulse,
    output          start_signal,
    
    output          scl_r,
    output          sda_r
);
assign scl_r = iic_scl;
assign sda_r = iic_sda;

localparam S_IDLE  = 4'd0; 
localparam S_WRITE = 4'd1;  
localparam S_PAUSE = 4'd2;
localparam S_READ  = 4'd4;
localparam S_DONE  = 4'd8;  

reg [3:0] state;

wire [63:0]rd_data;

reg [19:0] delay_cnt;
wire       delay_done = (delay_cnt == 20'd50000);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        delay_cnt <= 0;
    else if (!delay_done)
        delay_cnt <= delay_cnt + 1;
end

reg         start;
reg         wr_en;
reg         rd_en;
wire        complete; 

reg [17:0]pause_delay_cnt;
assign start_signal = start;

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
                    start <= 1'b1;      
                    wr_en <= 1'b1;      
                    state <= S_WRITE;
                end
            end

            S_WRITE: begin
                if (complete) begin
                    start    <= 1'b0;   
                    wr_en    <= 1'b0;
                    led_done <= 1'b1;   
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

assign iic_done_pulse = complete;
assign led_error     = 1'b0;

eeprom_rw #(
    .RW_SPEED ("fast"),
    .RD_MODE  (1),
    .RD_DATA_BYTE_NUM (8),
    .WR_DATA_BYTE_NUM (8)
) u_eeprom (
    .clk        (clk),
    .rst_n      (rst_n),
    .dev_addr   (7'b1010000),
    .start      (start),
    .data_addr  (16'h0000), 
    .wr_en      (wr_en),
    .wr_data    (64'h0123456789ABCDEF), 
    .rd_en      (rd_en),
    .rd_data    (rd_data),
    .iic_scl    (iic_scl),
    .iic_sda    (iic_sda),
    .iic_clk    (iic_clk),
    .complete   (complete),
    .iic_state  (iic_dbg_state)
);

endmodule
