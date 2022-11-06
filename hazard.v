`timescale 1ns / 1ps



`define EP 32'hBFC00380
`define A 32'h00000001
`define B 32'h00000004
`define C 32'h00000005
`define D 32'h00000008
`define E 32'h00000009
`define F 32'h0000000a
`define G 32'h0000000c
`define H 32'h0000000d
`define I 32'h0000000e
module hazard(
	//fetch stage
	output stallF,
	output flushF,
	//decode stage
	input  [4:0] rsD,rtD,
	input  branchD,
	input  pcsrcD,
	input  jumpD, jalD, jrD,
	output forwardaD,forwardbD,
	output stopD,
	output refreshD,
	//execute stage
	input  [4:0] rsE,rtE,
	input  [4:0] writeRegFinalE,
	input  regwriteE,
	input  memtoregE,
	input  cp0toregE,
	input  div_stallE,
	output reg [1:0] forwardaE,forwardbE,
	output stopE,
	output refreshE,
	//mem stage
	input  [4:0] writeregM,
	input  regwriteM,
	input  [31:0] exceptionTypeM,
	output stopM,
	output refreshM,
	input  [31:0] cp0_epcM,
	output reg [31:0] newpcM,
	//write back stage
	input  [4:0] writeregW,
	input  regwriteW,
	
	output refreshW,
	
	input  instructionStall,
	input  dataStall
    );

    wire branchstall; 
	wire jrstall,jalrstall;
	wire mfc0stall, flush_except;

	assign forwardaD = (rsD != 0 & regwriteM & rsD == writeregM); 
	assign forwardbD = (rtD != 0 & regwriteM & rtD == writeregM); 

	// hilo - Branch/jr/jalr    Load - Branch/jr/jalr   
	assign jrstall = jrD & ((regwriteE & (writeRegFinalE == rsD)) | (regwriteM & (writeregM == rsD)));
	assign jalrstall = jrD & jalD &  ((regwriteE & (writeRegFinalE == rsD)) | (regwriteM & (writeregM == rsD)));
	assign branchstall = branchD & ((regwriteE & (writeRegFinalE == rsD | writeRegFinalE == rtD)) | (regwriteM & (writeregM == rsD | writeregM == rtD)));


	always @(*) begin  
		if(rsE != 0) begin
			if(rsE == writeregM & regwriteM)
				forwardaE = 2'b10;
			else if(rsE == writeregW & regwriteW) //????????????????????，??????????????????????????????，
				forwardaE = 2'b01;
			else forwardaE = 2'b00; // ?????????
		end
		else forwardaE = 2'b00; // ?????????
	end

	always @(*) begin // ????R???????rt???????????? 
		if(rtE != 0) begin
			if(rtE == writeregM & regwriteM) //???????????????????，
				forwardbE = 2'b10;
			else if(rtE == writeregW & regwriteW) //????????????????????，??????????????????????????????，
				forwardbE = 2'b01;
			else forwardbE = 2'b00; // ?????????
		end
		else forwardbE = 2'b00; // ?????????
	end


    wire lwstall;         

    assign lwstall = ( (rsD==rtE) | (rtD==rtE) ) & memtoregE; 
	assign mfc0stall = ( (rsD ==  rtE| rtD == rtE) ) & cp0toregE;


    assign stallF = stopD;
    assign stopD = lwstall | branchstall | jalrstall | jrstall | instructionStall | mfc0stall | dataStall | div_stallE;
    assign stopE = div_stallE | dataStall;
    assign stopM = dataStall;
    
    

	assign flush_except = (exceptionTypeM != 32'b0);

	assign flushF = flush_except;
	assign refreshD = flush_except;
	assign refreshE = lwstall | flush_except | branchstall | mfc0stall;
	assign refreshM = flush_except;
	assign refreshW = flush_except | dataStall;
    
    always @(*) begin
		if(exceptionTypeM != 32'b0) begin
		
			case (exceptionTypeM)
				`A:begin 
					newpcM <= `EP;
				end
				`B:begin 
					newpcM <= `EP;

				end
				`C:begin 
					newpcM <= `EP;

				end
				`D:begin 
					newpcM <= `EP;
					
				end
				`E:begin 
					newpcM <= `EP;

				end
				`F:begin 
					newpcM <= `EP;

				end
				`G:begin 
					newpcM <= `EP;

				end
				`H:begin 
					newpcM <= `EP;

				end
				`I:begin 
					newpcM <= cp0_epcM;
				end
				default : newpcM <= `EP;
			endcase
		end
	end
    
endmodule
