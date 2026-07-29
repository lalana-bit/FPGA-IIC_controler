`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/07/28 18:37:01
// Design Name: 
// Module Name: eeprom_rw
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module eeprom_rw
#(
    parameter RW_SPEED = "fast",
    
    parameter RD_MODE = 0,
    parameter RD_DATA_BYTE_NUM = 1,
    
    parameter WR_DATA_BYTE_NUM = 1 //注意：写操作有回卷机制，需要控制写入字节数，最多64
)
(
    input                               clk,
    input                               rst_n,
    
    input      [6:0]                    dev_addr,
    input                               start,
    input      [15:0]                   data_addr,
    
    input                               wr_en,
    input      [WR_DATA_BYTE_NUM*8-1:0] wr_data,
    
    input                               rd_en,
    output     [RD_DATA_BYTE_NUM*8-1:0] rd_data,
    
    output                              iic_scl,
    inout                               iic_sda,
    output                              iic_clk,
    output                              complete,
    output      [2:0]                   iic_state
    );

iic_ctrl 
#(
    .RW_ADDR_BYTE_NUM(2),
    .SPEED(RW_SPEED),
    .WR_DATA_BYTE_NUM(WR_DATA_BYTE_NUM),
    .RD_DATA_BYTE_NUM(RD_DATA_BYTE_NUM),
    .RD_MODE(RD_MODE) //0:current address read;1:random address read
) iic_eeprom_inst(
    .clk(clk),
    .rst_n(rst_n),
    .dev_addr(dev_addr),
    .iic_start(start),
    .wr_en(wr_en),
    .wr_addr(data_addr),
    .wr_data(wr_data),
    .rd_en(rd_en),
    .rd_addr(data_addr),
    .rd_data(rd_data),
    .iic_scl(iic_scl),
    .iic_sda(iic_sda),
    .iic_clk(iic_clk),
    .iic_end(complete),
    .dbg_state(iic_state)
    );
    
endmodule
