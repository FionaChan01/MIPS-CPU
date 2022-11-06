`timescale 1ns / 1ps

`include "defines.vh"

module eqcmp(
	input wire [31:0] opA,opB,
	input wire [5:0] op,
	input wire [4:0] sel,
	output reg res
    );
	
	always @(*) begin
		case(op)
			`EXE_BEQ: res = (opA==opB);
			`EXE_BGTZ: res = ($signed(opA)>0);
			`EXE_BLEZ: res = ($signed(opA)<=0);
			`EXE_BNE: res = (opA!=opB);
			`EXE_REGIMM_INST:
			case(sel)
				`EXE_BLTZ,`EXE_BLTZAL: res = ($signed(opA)<0);
				`EXE_BGEZ,`EXE_BGEZAL: res = ($signed(opA)>=0);
			endcase
		endcase
	end

endmodule