module mem_test
#(
	parameter MEM_DATA_BITS = 64,
	parameter ADDR_BITS = 24
)
(
	input rst,                                         /*复位*/
	input mem_clk,                                     /*接口时钟*/
	output reg rd_burst_req,                          /*读请求*/
	output reg wr_burst_req,                          /*写请求*/
	output reg[9:0] rd_burst_len,                     /*读数据长度*/
	output reg[9:0] wr_burst_len,                     /*写数据长度*/
	output reg[ADDR_BITS - 1:0] rd_burst_addr,        /*读首地址*/
	output reg[ADDR_BITS - 1:0] wr_burst_addr,        /*写首地址*/
	input rd_burst_data_valid,                         /*读出数据有效*/
	input wr_burst_data_req,                           /*写数据信号*/
	input[MEM_DATA_BITS - 1:0] rd_burst_data,          /*读出的数据*/
	output[MEM_DATA_BITS - 1:0] wr_burst_data,         /*写入的数据*/
	input rd_burst_finish,                             /*读完成*/
	input wr_burst_finish,                             /*写完成*/
	output reg error
);
localparam IDLE = 3'd0;
localparam MEM_READ = 3'd1;
localparam MEM_WRITE  = 3'd2;

reg[2:0] state;
reg[7:0] wr_cnt;
reg[MEM_DATA_BITS - 1:0] wr_burst_data_reg;
assign wr_burst_data = wr_burst_data_reg;   //只要右侧表达式中的操作数有事件发生，就会计算右侧表达式的值
reg[7:0] rd_cnt;
always@(posedge mem_clk or posedge rst)  //posedge rst表示ui_clk已经复位完成，此后一直为低电平，所以在init_calib_complete为高时，其一直为低
begin
	if(rst)
		error <= 1'b0;
	else
		error <= (state == MEM_READ) && rd_burst_data_valid && (rd_burst_data != {(MEM_DATA_BITS/8){rd_cnt}});
end

always@(posedge mem_clk or posedge rst)
begin
	if(rst)
	begin
		wr_burst_data_reg <= {MEM_DATA_BITS{1'b0}};  //复制操作
		wr_cnt <= 8'd0;
	end
	else if(state == MEM_WRITE)
	begin
		if(wr_burst_data_req)
			begin
				wr_burst_data_reg <= {(MEM_DATA_BITS/8){wr_cnt}};
				wr_cnt <= wr_cnt + 8'd1;
			end
		else if(wr_burst_finish)
			wr_cnt <= 8'd0;
	end
end

always@(posedge mem_clk or posedge rst)
begin
	if(rst)
	begin
		rd_cnt <= 8'd0;
	end
	else if(state == MEM_READ)
	begin
		if(rd_burst_data_valid)
			begin
				rd_cnt <= rd_cnt + 8'd1;
			end
		else if(rd_burst_finish)
			rd_cnt <= 8'd0;
	end
	else
		rd_cnt <= 8'd0;
end

always@(posedge mem_clk or posedge rst)
begin
	if(rst)
	begin
		state <= IDLE;
		wr_burst_req <= 1'b0;
		rd_burst_req <= 1'b0;
		rd_burst_len <= 10'd128;
		wr_burst_len <= 10'd128;
		rd_burst_addr <= 0;
		wr_burst_addr <= 0;
	end
	else
	begin
		case(state)
			IDLE:
			begin
				state <= MEM_WRITE;   //在ddr3初始化时，state为MEM_WRITE
				wr_burst_req <= 1'b1;
				wr_burst_len <= 10'd128;
			end
			MEM_WRITE:
			begin
				if(wr_burst_finish)
				begin
					state <= MEM_READ;
					wr_burst_req <= 1'b0;
					rd_burst_req <= 1'b1;
					rd_burst_len <= 10'd128;
					rd_burst_addr <= wr_burst_addr;
				end
			end
			MEM_READ:
			begin
				if(rd_burst_finish)
				begin
					state <= MEM_WRITE;
					wr_burst_req <= 1'b1;
					wr_burst_len <= 10'd128;
					rd_burst_req <= 1'b0;
					wr_burst_addr <= wr_burst_addr + 128;
				end
			end
			default:
				state <= IDLE;
		endcase
	end
end

endmodule