`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/03/31 16:22:12
// Design Name: 
// Module Name: project
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


module project(input clk, output [47:0] dout_tdata, output dout_tvalid, output [55:0] x,
              output [55:0] y,output [55:0] z, output [23:0] x_, output [34:0] y_, output [23:0] z_, output [44:0] tmp1,
			  output [55:0] tmp2, output [44:0] tmp3, 
			  output [25:0] weight_y00, output [25:0] weight_x00, output [25:0] weight_y01, output [25:0] weight_x01, 
              output [25:0] weight_y10, output [25:0] weight_x10, output [25:0] weight_y11, output [25:0] weight_x11);

reg [15:0] phase_tdata;
reg [23:0] coe = 24'b0_0_0000_0000_0001_1000_0011_01;   //22位小数  2707.47
reg [34:0] u; //35位中间结果
reg [34:0] v;
reg signed [23:0] x_; //16位sin(u)  //14位小数位
reg signed [34:0] y_; //即为v
reg signed [23:0] z_; //16位cos(u)
reg [44:0] tmp1; //21 + 24 =45
reg [55:0] tmp2; //21 + 35 = 56
reg [44:0] tmp3; //21 + 16 =37
reg[55:0] a;
reg[55:0] b;
reg[55:0] c;
reg [44:0] a1;
reg [44:0] b1;
reg [44:0] c1;
reg [55:0] x;
reg [55:0] y;
reg [55:0] z;
reg [25:0] weight_x00;
reg [25:0] weight_y00;
reg [25:0] weight_x01;
reg [25:0] weight_y01;
reg [25:0] weight_x10;
reg [25:0] weight_y10;
reg [25:0] weight_x11;
reg [25:0] weight_y11;
reg signed [10:0] dst_tl_x = -543;
reg signed [10:0] dst_br_x = 542;
reg signed [10:0] dst_tl_y = -550;
reg signed [10:0] dst_br_y = 549;

//内参矩阵的逆
reg signed [20:0] k_inv0 = 21'b0_1010_1001_0011_0111_1000;  //21位定点数，一位符号位，12位整数位，八位小数位
reg signed [20:0] k_inv1 = 21'b0_0000_0000_0000_0000_0000;
reg signed [20:0] k_inv2 = 21'b0_0010_0010_0110_1000_0000;
reg signed [20:0] k_inv3 = 21'b0_0000_0000_0000_0000_0000;
reg signed [20:0] k_inv4 = 21'b0_1010_1001_0011_0111_1000;
reg signed [20:0] k_inv5 = 21'b0_0010_0010_0110_1000_0000;
reg signed [20:0] k_inv6 = 21'b0_0000_0000_0000_0000_0000;
reg signed [20:0] k_inv7 = 21'b0_0000_0000_0000_0000_0000;
reg signed [20:0] k_inv8 = 21'b0_0000_0000_0001_0000_0000;

always@(posedge clk)
	begin
	    if(dst_tl_y <= dst_br_y)
		begin
			if(dst_tl_x <= dst_br_x)
				begin
					u = $signed(dst_tl_x) * $signed(coe); 
					v = $signed(dst_tl_y) * $signed(coe);  //11+24 = 35
					phase_tdata[15:0] = u[24:9];
					x_= dout_tdata[47:24];
					y_= v;
					z_= dout_tdata[23:0];
					tmp1 = $signed(k_inv3) * $signed(x_);
					tmp2 = $signed(k_inv4) * $signed(y_);
					tmp3 = $signed(k_inv5) * $signed(z_);
					a=$signed(k_inv1) * $signed(y_);
					b=$signed(k_inv4) * $signed(y_);
					c=$signed(k_inv7) * $signed(y_);
					x =($signed(k_inv0) * $signed(x_)) + ($signed(k_inv1) * $signed(y_)) + ($signed(k_inv2) * $signed(z_));  //x的位数为 21 + 35 = 56位， 其中小数位为 8 + 22 = 30位
					y =($signed(k_inv3) * $signed(x_)) + ($signed(k_inv4) * $signed(y_)) + ($signed(k_inv5) * $signed(z_));  //x的位数为 21 + 35 = 56位， 其中小数位为 8 + 22 = 36位
					z =($signed(k_inv6) * $signed(x_)) + ($signed(k_inv7) * $signed(y_)) + ($signed(k_inv8) * $signed(z_));  //x的位数为 21 + 35 = 56位， 其中小数位为 8 + 22 = 36位
					weight_y00 = y[55:30]; //floor
					weight_x00 = x[55:30]; //floor
					weight_y01 = y[55:30] + 1;  //ceil
					weight_x01 = x[55:30]; //floor
					weight_y10 = y[55:30]; //floor
					weight_x10 = x[55:30] + 1; //ceil
					weight_y11 = y[55:30] + 1; //ceil
					weight_x11 = x[55:30] + 1; //ceil
					
					dst_tl_x = dst_tl_x + 1;
					//
				end
		end	
		if(dst_tl_x == dst_br_x)
			begin
			    dst_br_x = dst_tl_x;
				dst_tl_y = dst_tl_y + 1;
			end
	end

cordic_0 uut(
				.s_axis_phase_tvalid(1'b1),  // input wire s_axis_phase_tvalid
				.s_axis_phase_tdata(phase_tdata),    // input wire [15 : 0] s_axis_phase_tdata
				.m_axis_dout_tvalid(dout_tvalid),    // output wire m_axis_dout_tvalid
				.m_axis_dout_tdata(dout_tdata)      // output wire [31 : 0] m_axis_dout_tdata
				);	
endmodule
