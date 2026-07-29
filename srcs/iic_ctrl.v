module iic_ctrl 
#(
    parameter RW_ADDR_BYTE_NUM = 1,
    parameter SPEED ="fast",
    parameter WR_DATA_BYTE_NUM = 1,
    parameter RD_DATA_BYTE_NUM = 1,
    parameter RD_MODE = 0 //0:current address read;1:random address read
)
(
    input                               clk,
    input                               rst_n,
    
    input      [6:0]                    dev_addr,
    input                               iic_start,
    
    input                               wr_en,
    input      [RW_ADDR_BYTE_NUM*8-1:0] wr_addr,
    input      [WR_DATA_BYTE_NUM*8-1:0] wr_data,
    
    input                               rd_en,
    input      [RW_ADDR_BYTE_NUM*8-1:0] rd_addr,
    output reg [RD_DATA_BYTE_NUM*8-1:0] rd_data,
    
    output                              iic_scl,
    inout                              iic_sda,
    output reg                          iic_clk,
    output                              iic_end,
    output      [2:0]                   dbg_state
    );
    
localparam CLK_FREQ = 50_000_000;
//慢速：400khz,中速：1.6Mhz,快速:4Mhz，对应scl频率分别为100k、400k、1M
localparam FREQ_IIC_CLK = (SPEED == "low") ? 400_000 : (SPEED == "medium") ? 1_600_000 : 4_000_000;
//分频计数
localparam IIC_CLK_CNT_MAX = 50_000_000/FREQ_IIC_CLK;
localparam IIC_CLK_CNT_WIDTH = $clog2(IIC_CLK_CNT_MAX) + 1;
//读写地址位数相同，不细分
localparam RW_ADDR_BYTE_CNT_WIDTH = $clog2(RW_ADDR_BYTE_NUM) + 1;

//写响应个数：从机地址1位+数据地址RW_ADDR_BYTE_NUM位+写数据WR_DATA_BYTE_NUM位
localparam WR_ACK_CNT_MAX = 1+RW_ADDR_BYTE_NUM+WR_DATA_BYTE_NUM;
localparam WR_ACK_CNT_WIDTH = $clog2(WR_ACK_CNT_MAX) + 1;
localparam WR_DATA_CNT_WIDTH = $clog2(WR_DATA_BYTE_NUM) + 1;

//读响应个数：current read模式：从机地址1位+读数据RD_DATA_BYTE_NUM位；random read模式：从机地址1位+读地址RW_ADDR_BYTE_NUM位+从机地址1位+读数据RD_DATA_BYTE_NUM位
localparam RD_ACK_CNT_MAX = !RD_MODE ? 1+RD_DATA_BYTE_NUM : 1+1+RW_ADDR_BYTE_NUM+RD_DATA_BYTE_NUM;
localparam RD_ACK_CNT_WIDTH = $clog2(RD_ACK_CNT_MAX) + 1;
localparam RD_DATA_CNT_WIDTH = $clog2(RD_DATA_BYTE_NUM) + 1;

//写状态位
localparam WR_IDLE = 3'b000;
localparam WR_START = 3'b001;
localparam WR_SEND_SLAVE_ADDR = 3'b010;
localparam WR_WAIT_ACK = 3'b011;
localparam WR_SEND_DATA_ADDR = 3'b100;
localparam WR_SEND_DATA = 3'b101;
localparam WR_STOP = 3'b110;

//读状态位
localparam RD_IDLE = 3'b000;
localparam RD_START = 3'b001;
localparam RD_SEND_SLAVE_ADDR = 3'b010;
localparam RD_WAIT_ACK = 3'b011;
localparam RD_SEND_DATA_ADDR = 3'b100;
localparam RD_SEND_DATA = 3'b101;
localparam RD_STOP = 3'b110;

reg [2:0]state;
reg [2:0]next_state;
reg [2:0]state_rd;
reg [2:0]next_state_rd;
//reg [2:0]last_state;
assign dbg_state = wr_en ? state : state_rd;

reg [7:0]real_dev_addr;

reg [IIC_CLK_CNT_WIDTH-1:0]iic_clk_cnt;

reg [WR_DATA_CNT_WIDTH-1:0]wr_data_cnt;
reg [WR_DATA_CNT_WIDTH-1:0]wr_data_cnt_r;
reg [WR_ACK_CNT_WIDTH-1:0]wr_ack_cnt;

reg [RD_DATA_CNT_WIDTH-1:0]rd_data_cnt;
reg [RD_DATA_CNT_WIDTH-1:0]rd_data_cnt_r;
reg [RD_ACK_CNT_WIDTH-1:0]rd_ack_cnt;
reg rd_start_state_cnt;

reg [RW_ADDR_BYTE_CNT_WIDTH-1:1]rw_addr_byte_cnt;
reg [RW_ADDR_BYTE_CNT_WIDTH-1:1]rw_addr_byte_cnt_r;

reg [1:0]scldiv4_cnt;
reg [2:0]bits_cnt;
reg scl_wr;
reg sda_wr;
reg sample_bit;
reg sda_oe_wr;

// 读状态机独立信号，避免多驱动冲突
reg [7:0]real_dev_addr_rd;
reg [1:0]scldiv4_cnt_rd;
reg [2:0]bits_cnt_rd;
reg scl_rd;
reg sda_rd;
reg [RW_ADDR_BYTE_CNT_WIDTH-1:1]rw_addr_byte_cnt_rd;
reg [RW_ADDR_BYTE_CNT_WIDTH-1:1]rw_addr_byte_cnt_r_rd;
reg sda_oe_rd;

wire wr_end;
wire rd_end;

wire wr_ack_valid;
wire rd_ack_valid;
wire iic_clk_posedge;
wire rd_slave_ack = RD_MODE ? (rd_ack_cnt <= RW_ADDR_BYTE_NUM + 1) : (rd_ack_cnt == 0);

assign iic_scl = wr_en ? scl_wr : scl_rd;

wire sda_val = wr_en ? sda_wr : sda_rd;
wire sda_oe  = wr_en ? sda_oe_wr : sda_oe_rd;
assign iic_sda = sda_oe ? sda_val : 1'bz;
assign iic_end = wr_end || rd_end; 

//分频出iic工作时钟
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        iic_clk_cnt <= 0;
    end else begin
        iic_clk_cnt <= (iic_clk_cnt == IIC_CLK_CNT_MAX) ? 0 : (iic_clk_cnt + 1);
        iic_clk <= (iic_clk_cnt == IIC_CLK_CNT_MAX/2 - 1) || (iic_clk_cnt == IIC_CLK_CNT_MAX) ? !iic_clk : iic_clk;
    end
end

assign iic_clk_posedge = iic_clk_cnt == IIC_CLK_CNT_MAX;

//iic写状态机
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state <= WR_IDLE;
        scl_wr <= 1;
        sda_wr <= 1;
        real_dev_addr <= 0;
        wr_ack_cnt <= 0;
        rw_addr_byte_cnt <= 0;
        rw_addr_byte_cnt_r <= 0;
        wr_data_cnt <= 0;
        wr_data_cnt_r <= 0;
        scldiv4_cnt <= 0;
        bits_cnt <= 0;
        sda_oe_wr <= 1;
    end else begin
        //写使能且在4Mhz时钟域下
        if(wr_en && iic_clk_posedge) begin
            state <= next_state;
            sda_oe_wr <= 1'b1;
            case(state)
                WR_IDLE: begin
                    real_dev_addr <= {dev_addr,1'b0};
                    scl_wr <= 1;
                    sda_wr <= 1;
                    wr_ack_cnt <= 0;
                    rw_addr_byte_cnt <= 0;
                    rw_addr_byte_cnt_r <= 0;
                    wr_data_cnt <= 0;
                    wr_data_cnt_r <= 0;
                    scldiv4_cnt <= 0;
                    bits_cnt <= 0;
                end
                WR_START: begin
                    scldiv4_cnt <= scldiv4_cnt + 1;
                    case(scldiv4_cnt)
                        2'b00: begin
                            sda_wr <= 0;
                            scl_wr <= scl_wr;
                        end
                        2'b01: begin
                            scl_wr <= 0;
                            sda_wr <= sda_wr;
                        end
                        2'b10: begin
                            scl_wr <= 0;
                            sda_wr <= 0;
                            
                        end
                        2'b11: begin
                            scl_wr <= 0;
                            sda_wr <= 0;
                        end
                    endcase
                end
                WR_SEND_SLAVE_ADDR: begin
                    scldiv4_cnt <= scldiv4_cnt + 1;
                    case(scldiv4_cnt)
                        2'b00: begin
                            sda_wr <= real_dev_addr[7'b111 - bits_cnt];
                            scl_wr <= scl_wr;
                        end
                        2'b01: begin
                            scl_wr <= 1;
                            sda_wr <= sda_wr;
                        end
                        2'b10: begin
                            scl_wr <= scl_wr;
                            sda_wr <= sda_wr;
                            
                        end
                        2'b11: begin
                            scl_wr <= 0;
                            sda_wr <= sda_wr;
                            bits_cnt <= bits_cnt + 3'b1;
                        end
                    endcase
                end
                WR_WAIT_ACK: begin
                    sda_oe_wr <= 1'b0;
                    scldiv4_cnt <= scldiv4_cnt + 1;
                    case(scldiv4_cnt)
                        2'b00: begin
                            sda_wr <= 1;
                            scl_wr <= scl_wr;
                        end
                        2'b01: begin
                            scl_wr <= 1;
                        end
                        2'b10: begin
//                            sda_wr <= sda_wr; //等待响应，主机不进行任何操作
                        end
                        2'b11: begin
                            scl_wr <= 0;
                            if(wr_ack_valid) begin
                                wr_ack_cnt <= (wr_ack_cnt == RW_ADDR_BYTE_NUM+WR_DATA_BYTE_NUM) ? 0 : wr_ack_cnt + 1;
                            end
                        end
                    endcase
                end
                WR_SEND_DATA_ADDR: begin
                    scldiv4_cnt <= scldiv4_cnt + 2'b1;
                    case(scldiv4_cnt)
                        2'b00: begin
                            sda_wr <= wr_addr[(RW_ADDR_BYTE_NUM-rw_addr_byte_cnt)*8-1-bits_cnt];
                            scl_wr <= scl_wr;
                        end
                        2'b01: begin
                            scl_wr <= 1;
                            sda_wr <= sda_wr;
                        end
                        2'b10: begin
                            sda_wr <= sda_wr; 
                            scl_wr <= scl_wr;
                            if(bits_cnt == 3'b111) begin
                                rw_addr_byte_cnt <= (rw_addr_byte_cnt == RW_ADDR_BYTE_NUM) ? 0 : rw_addr_byte_cnt + 1;
                            end
                        end
                        2'b11: begin
                            scl_wr <= 0;
                            bits_cnt <= bits_cnt + 1;
                            rw_addr_byte_cnt_r <= rw_addr_byte_cnt;
                            
                        end
                    endcase
                end
                WR_SEND_DATA: begin
                    scldiv4_cnt <= scldiv4_cnt + 2'b1;
                    case(scldiv4_cnt)
                        2'b00: begin
                            sda_wr <= wr_data[(WR_DATA_BYTE_NUM - wr_data_cnt)*8 - bits_cnt - 1];
                            scl_wr <= scl_wr;
                        end
                        2'b01: begin
                            scl_wr <= 1;
                            sda_wr <= sda_wr;
                        end
                        2'b10: begin
                            scl_wr <= scl_wr;
                            sda_wr <= sda_wr;
                            if(bits_cnt == 3'b111) begin
                                wr_data_cnt <= (wr_data_cnt == WR_DATA_BYTE_NUM) ? 0 : wr_data_cnt + 1; 
                            end
                        end
                        2'b11: begin
                            scl_wr <= 0;
                            sda_wr <= sda_wr;
                            bits_cnt <= bits_cnt + 1;
                            wr_data_cnt_r <= wr_data_cnt;
                            
                        end
                    endcase
                end
                WR_STOP: begin
                    scldiv4_cnt <= scldiv4_cnt + 1;
                    case(scldiv4_cnt)
                        2'b00: begin
                            sda_wr <= 0;
                            scl_wr <= scl_wr;
                        end
                        2'b01: begin
                            scl_wr <= 1;
                            sda_wr <= sda_wr;
                        end
                        2'b10: begin
                            scl_wr <= scl_wr;
                            sda_wr <= 1;
                        end
                        2'b11: begin
                            scl_wr <= 0;
                            sda_wr <= sda_wr;
                        end
                    endcase                
                end
                default: begin
                    scl_wr <= scl_wr;
                    sda_wr <= sda_wr;
                    scldiv4_cnt <= scldiv4_cnt;
                    bits_cnt <= bits_cnt;
                end
            endcase
        end
    end
end 

assign wr_end = (state == WR_STOP) && (scldiv4_cnt == 2'b11) && (wr_en && iic_clk_posedge);
assign wr_ack_valid = (wr_en && (state == WR_WAIT_ACK)) ? !iic_sda : 0;

always@(*) begin
    next_state = state;  // 默认保持，消除 latch
    if(wr_en) begin
        case(state)
            WR_IDLE: begin
                if(iic_start)
                    next_state = WR_START;
            end
            WR_START: begin
                if(scldiv4_cnt == 2'b11)
                    next_state = WR_SEND_SLAVE_ADDR;
            end
            WR_SEND_SLAVE_ADDR: begin
                if(scldiv4_cnt == 2'b11 && bits_cnt == 3'b111)
                    next_state = WR_WAIT_ACK;
            end
            WR_WAIT_ACK: begin
                if(scldiv4_cnt == 2'b11 && wr_ack_valid) begin
                    if(wr_ack_cnt == 0)
                        next_state = WR_SEND_DATA_ADDR;
                    else if(wr_ack_cnt == RW_ADDR_BYTE_NUM)
                        next_state = WR_SEND_DATA;
                    else if(wr_ack_cnt < RW_ADDR_BYTE_NUM)
                        next_state = WR_SEND_DATA_ADDR;
                    else if(wr_ack_cnt == RW_ADDR_BYTE_NUM+WR_DATA_BYTE_NUM)
                        next_state = WR_STOP;
                    else if(wr_ack_cnt < RW_ADDR_BYTE_NUM+WR_DATA_BYTE_NUM && wr_ack_cnt > RW_ADDR_BYTE_NUM)
                        next_state = WR_SEND_DATA;
                end
            end
            WR_SEND_DATA_ADDR: begin
                if(scldiv4_cnt == 2'b11 && (rw_addr_byte_cnt_r != rw_addr_byte_cnt))
                    next_state = WR_WAIT_ACK;
            end
            WR_SEND_DATA: begin
                if(scldiv4_cnt == 2'b11 && (wr_data_cnt_r != wr_data_cnt))
                    next_state = WR_WAIT_ACK;
            end
            WR_STOP: begin
                if(scldiv4_cnt == 2'b11)
                    next_state = WR_IDLE;
            end
            default: begin
                if(scldiv4_cnt == 2'b11)
                    next_state = WR_IDLE;
            end
        endcase
    end
end

//iic读状态机
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state_rd <= RD_IDLE;
//        last_state <= RD_IDLE;
        scl_rd <= 1;
        sda_rd <= 1;
        real_dev_addr_rd <= 0;
        rd_ack_cnt <= 0;
        rw_addr_byte_cnt_rd <= 0;
        rw_addr_byte_cnt_r_rd <= 0;
        rd_data_cnt <= 0;
        rd_data_cnt_r <= 0;
        rd_start_state_cnt <= 0;
        scldiv4_cnt_rd <= 0;
        bits_cnt_rd <= 0;
        sda_oe_rd <= 1;
    end else begin
        //读使能且在4Mhz时钟域下
        if(rd_en && iic_clk_posedge) begin
            state_rd <= next_state_rd;
//            last_state <= state;
            sda_oe_rd <= 1'b1;
            case(state_rd)
                RD_IDLE: begin
                    scl_rd <= 1;
                    sda_rd <= 1;
                    if(RD_MODE == 0) begin
                        real_dev_addr_rd <= {dev_addr,1'b1};
                    end else begin
                        real_dev_addr_rd <= {dev_addr,1'b0};
                    end
                    rd_ack_cnt <= 0;
                    rw_addr_byte_cnt_rd <= 0;
                    rw_addr_byte_cnt_r_rd <= 0;
                    rd_data_cnt <= 0;
                    rd_data_cnt_r <= 0;
                    scldiv4_cnt_rd <= 0;
                    bits_cnt_rd <= 0;
                    rd_start_state_cnt <= 0;
                end
                RD_START: begin
                    scldiv4_cnt_rd <= scldiv4_cnt_rd + 1;
                    case(scldiv4_cnt_rd)
                        2'b00: begin
                            sda_rd <= 1;
                            scl_rd <= 0;
                            if(rd_start_state_cnt == 1) begin
                                real_dev_addr_rd <= {dev_addr,1'b1};
                                rd_start_state_cnt <= 0;
                            end else begin
                                rd_start_state_cnt <= rd_start_state_cnt + 1;
                            end
                        end
                        2'b01: begin
                            scl_rd <= 1;
                            sda_rd <= sda_rd;
                        end
                        2'b10: begin
                            scl_rd <= scl_rd;
                            sda_rd <= 0;
                            
                        end
                        2'b11: begin
                            scl_rd <= 0;
                            sda_rd <= sda_rd;
                        end
                    endcase
                end
                RD_SEND_SLAVE_ADDR: begin
                    scldiv4_cnt_rd <= scldiv4_cnt_rd + 1;
                    case(scldiv4_cnt_rd)
                        2'b00: begin
                            sda_rd <= real_dev_addr_rd[7'b111 - bits_cnt_rd];
                            scl_rd <= scl_rd;
                        end
                        2'b01: begin
                            scl_rd <= 1;
                            sda_rd <= sda_rd;
                        end
                        2'b10: begin
                            scl_rd <= scl_rd;
                            sda_rd <= sda_rd;
                            
                        end
                        2'b11: begin
                            scl_rd <= 0;
                            sda_rd <= sda_rd;
                            bits_cnt_rd <= bits_cnt_rd + 3'b1;
                        end
                    endcase
                end
                RD_WAIT_ACK: begin
                    sda_oe_rd <= rd_slave_ack ? 1'b0 : 1'b1;
                    scldiv4_cnt_rd <= scldiv4_cnt_rd + 1;
                    case(scldiv4_cnt_rd)
                        2'b00: begin
                            if(rd_data_cnt == RD_DATA_BYTE_NUM) begin
                                sda_rd <= 1;
                            end else begin
                                sda_rd <= 0;
                            end
                            scl_rd <= scl_rd;
                        end
                        2'b01: begin
                            scl_rd <= 1;
                            sda_rd <= sda_rd;
                        end
                        2'b10: begin
                            scl_rd <= scl_rd;
                            sda_rd <= sda_rd;
                        end
                        2'b11: begin
                            scl_rd <= 0;
                            if(rd_ack_valid) begin
                                rd_ack_cnt <= (rd_ack_cnt == RD_ACK_CNT_MAX - 1) ? 0 : rd_ack_cnt + 1;
                            end
                        end
                    endcase
                end
                RD_SEND_DATA_ADDR: begin
                    scldiv4_cnt_rd <= scldiv4_cnt_rd + 2'b1;
                    case(scldiv4_cnt_rd)
                        2'b00: begin
                            sda_rd <= rd_addr[(RW_ADDR_BYTE_NUM-rw_addr_byte_cnt_rd)*8-1-bits_cnt_rd];
                            scl_rd <= scl_rd;
                        end
                        2'b01: begin
                            scl_rd <= 1;
                            sda_rd <= sda_rd;
                        end
                        2'b10: begin
                            sda_rd <= sda_rd; 
                            scl_rd <= scl_rd;
                            if(bits_cnt_rd == 3'b111) begin
                                rw_addr_byte_cnt_rd <= (rw_addr_byte_cnt_rd == RW_ADDR_BYTE_NUM) ? 0 : rw_addr_byte_cnt_rd + 1;
                            end
                        end
                        2'b11: begin
                            scl_rd <= 0;
                            bits_cnt_rd <= bits_cnt_rd + 1;
                            rw_addr_byte_cnt_r_rd <= rw_addr_byte_cnt_rd;
                            
                        end
                    endcase
                end
                RD_SEND_DATA: begin
                    sda_oe_rd <= 1'b0;
                    scldiv4_cnt_rd <= scldiv4_cnt_rd + 2'b1;
                    case(scldiv4_cnt_rd)
                        2'b00: begin
                            scl_rd <= scl_rd;
                        end
                        2'b01: begin
                            scl_rd <= 1;
                        end
                        2'b10: begin
                            scl_rd <= scl_rd;
                            rd_data[(RD_DATA_BYTE_NUM-rd_data_cnt)*8-1-bits_cnt_rd] <= iic_sda;
                            if(bits_cnt_rd == 3'b111) begin
                                rd_data_cnt <= (rd_data_cnt == RD_DATA_BYTE_NUM) ? 0 : rd_data_cnt + 1; 
                            end
                        end
                        2'b11: begin
                            scl_rd <= 0;
                            bits_cnt_rd <= bits_cnt_rd + 1;
                            rd_data_cnt_r <= rd_data_cnt;
                            
                        end
                    endcase
                end
                RD_STOP: begin
                    scldiv4_cnt_rd <= scldiv4_cnt_rd + 1;
                    case(scldiv4_cnt_rd)
                        2'b00: begin
                            sda_rd <= 0;
                            scl_rd <= scl_rd;
                        end
                        2'b01: begin
                            scl_rd <= 1;
                            sda_rd <= sda_rd;
                        end
                        2'b10: begin
                            scl_rd <= scl_rd;
                            sda_rd <= 1;
                        end
                        2'b11: begin
                            scl_rd <= 0;
                            sda_rd <= sda_rd;
                        end
                    endcase                
                end
                default: begin
                    scl_rd <= scl_rd;
                    sda_rd <= sda_rd;
                    scldiv4_cnt_rd <= scldiv4_cnt_rd;
                    bits_cnt_rd <= bits_cnt_rd;
                end
            endcase
        end
    end
end 

assign rd_end = (state_rd == RD_STOP) && (scldiv4_cnt_rd == 2'b11) && rd_en && iic_clk_posedge;
assign rd_ack_valid = (rd_en && state_rd == RD_WAIT_ACK && scldiv4_cnt_rd != 2'b0) ? (rd_slave_ack ? !iic_sda : !sda_rd) : 0;

always@(*) begin
    next_state_rd = state_rd;  // 默认保持，消除 latch
    if(rd_en) begin
        case(state_rd)
            RD_IDLE: begin
                if(iic_start)
                    next_state_rd = RD_START;
            end
            RD_START: begin
                if(scldiv4_cnt_rd == 2'b11)
                    next_state_rd = RD_SEND_SLAVE_ADDR;
            end
            RD_SEND_SLAVE_ADDR: begin
                if(scldiv4_cnt_rd == 2'b11 && bits_cnt_rd == 3'b111)
                    next_state_rd = RD_WAIT_ACK;
            end
            RD_WAIT_ACK: begin
                if(scldiv4_cnt_rd == 2'b11 && rd_ack_valid) begin
                    if(RD_MODE == 0) begin
                        if(rd_ack_cnt == 0)
                            next_state_rd = RD_SEND_DATA;
                        else if(rd_ack_cnt < RD_ACK_CNT_MAX - 1)
                            next_state_rd = RD_SEND_DATA;
                    end else begin
                        if(rd_ack_cnt == 0)
                            next_state_rd = RD_SEND_DATA_ADDR;
                        else if(rd_ack_cnt < RW_ADDR_BYTE_NUM)
                            next_state_rd = RD_SEND_DATA_ADDR;
                        else if(rd_ack_cnt == RW_ADDR_BYTE_NUM)
                            next_state_rd = RD_START;
                        else if(rd_ack_cnt == RW_ADDR_BYTE_NUM + 1)
                            next_state_rd = RD_SEND_DATA;
                        else if(rd_ack_cnt > RW_ADDR_BYTE_NUM + 1 && rd_ack_cnt < RD_ACK_CNT_MAX - 1)
                            next_state_rd = RD_SEND_DATA;
                    end
                end else if(scldiv4_cnt_rd == 2'b11 && !rd_ack_valid) begin
                    if(rd_ack_cnt == RD_ACK_CNT_MAX - 1)
                        next_state_rd = RD_STOP;
                end
            end
            RD_SEND_DATA_ADDR: begin
                if(scldiv4_cnt_rd == 2'b11 && (rw_addr_byte_cnt_r_rd != rw_addr_byte_cnt_rd))
                    next_state_rd = RD_WAIT_ACK;
            end
            RD_SEND_DATA: begin
                if(scldiv4_cnt_rd == 2'b11 && (rd_data_cnt_r != rd_data_cnt))
                    next_state_rd = RD_WAIT_ACK;
            end
            RD_STOP: begin
                if(scldiv4_cnt_rd == 2'b11)
                    next_state_rd = RD_IDLE;
            end
            default: begin
                if(scldiv4_cnt_rd == 2'b11)
                    next_state_rd = RD_IDLE;
            end
        endcase
    end
end

//always@(posedge clk or negedge rst_n) begin
//    if(!rst_n) begin
//        rd_end <= 0;
//    end else begin
//        if(rd_en && iic_clk_posedge && scldiv4_cnt == 2'b00 && state == RD_WAIT_ACK) begin
//            rd_end <= !sda;
//        end
//    end
//end
    
endmodule
