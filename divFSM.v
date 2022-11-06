module divFSM (
    input wire clk,rst,
    input wire ReadyDivE,ValidDivE,
    output reg StallDivE, BeginDivE
);

//除法的有限状态机（判断除法处于什么状态）
	always@(*)begin
		if(rst) begin
			StallDivE = 0;  
		end
		else if(ValidDivE)begin
			if(ReadyDivE == 1'b0)begin
				BeginDivE = 1'b1;
				StallDivE = 1'b1;
			end
			else if(ReadyDivE == 1'b1)begin
				BeginDivE = 1'b0;
				StallDivE = 1'b0;
			end
			else begin
				BeginDivE = 1'b0;
				StallDivE = 1'b0;
			end
		end
	end
    
endmodule