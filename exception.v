`timescale 1ns / 1ps

`include "defines.vh"
module exception(
	input wire rst,
	input wire[`ExceptionBus] exception,
	input wire [`GeneralBus] instrGF,
	input wire isADEL,isADES,
	input wire[`GeneralBus] StatusCP0_i,CaseCP0_i,
	output reg[`GeneralBus] exceptionType_o
    );

	always @(*) begin
		if(rst) begin
			exceptionType_o <= `ZeroWord;
		end else begin 
			exceptionType_o <= `ZeroWord;
			if(((CaseCP0_i [15:8] & StatusCP0_i[15:8]) != 8'd0) && (instrGF) && (StatusCP0_i[0] == 1'b1) && (StatusCP0_i[1] == 1'b0)) 
			begin	
				exceptionType_o <= `ZeroWord + 1;
			end else if(exception[7] == 1'b1 || isADEL) begin
				exceptionType_o <= `ZeroWord + 4;
			end else if(isADES) begin
				exceptionType_o <= `ZeroWord + 5; 
			end else if(exception[6] == 1'b1) begin
				exceptionType_o <= `ZeroWord + 8; 
			end else if(exception[5] == 1'b1) begin
				exceptionType_o <= `ZeroWord + 9; 
			end else if(exception[4] == 1'b1) begin
				exceptionType_o <= `ZeroWord + 32'h0e; 
			end else if(exception[3] == 1'b1) begin
				exceptionType_o <= `ZeroWord + 32'h0a; 
			end else if(exception[2] == 1'b1) begin
				exceptionType_o <= `ZeroWord + 32'h0c;
		end
	end
	end
	
endmodule

