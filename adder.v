`timescale 1ns / 1ps


module adder #(parameter WIDTH=32)(
	input wire[WIDTH-1:0] opA,opB,
	output wire[WIDTH-1:0] res
    );

	assign res = opA + opB;
	
endmodule
