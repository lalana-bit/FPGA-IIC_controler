`timescale 1ns / 1ps

module eeprom_rw
#(
    parameter RW_SPEED = "fast", //low:100khz;medium:400khz;fast:1Mhz
    
    parameter RD_MODE = 0,  //0:current address read;1:random address read
    parameter RD_DATA_BYTE_NUM = 1, //读取字节数，不可太多
    
    parameter WR_DATA_BYTE_NUM = 1  //注意：写操作有回卷机制，需要控制写入字节数，最多64（此时为页写入操作）
)
(
    input                               clk,
    input                               rst_n,
    
    input      [6:0]                    dev_addr,     //从机地址
    input                               start,        //启动信号
    input      [15:0]                   data_addr,    //2字节数据地址（current address read模式可不用）
    
    input                               wr_en,        //写使能（与读使能互斥）
    input      [WR_DATA_BYTE_NUM*8-1:0] wr_data,      //写数据
    
    input                               rd_en,        //读使能
    output     [RD_DATA_BYTE_NUM*8-1:0] rd_data,      //读数据缓冲区
    
    output                              iic_scl,      //scl外部连接线
    inout                               iic_sda,      //sda外部连接线
    output                              iic_clk,      //i2c驱动工作时钟（400k、1.6M、4M）
    output                              complete,     //完成标志脉冲
    output      [2:0]                   iic_state     //可接外部调试查看状态机流转
    );

iic_ctrl 
#(
    .RW_ADDR_BYTE_NUM(2), //eeprom寻址需要2字节
    .SPEED(RW_SPEED),
    .WR_DATA_BYTE_NUM(WR_DATA_BYTE_NUM),
    .RD_DATA_BYTE_NUM(RD_DATA_BYTE_NUM),
    .RD_MODE(RD_MODE)
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
