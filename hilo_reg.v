`timescale 1ns / 1ps



module hilo_reg(
	input wire clk,rst,we,
	input wire[31:0] hi,lo,
	output reg[31:0] hi_o,lo_o
    );
	
	always @(posedge clk) begin
		if(rst) begin
			hi_o <= 0;
			lo_o <= 0;
		end else if (we) begin
			hi_o <= hi;
			lo_o <= lo;
		end
	end
endmodule
