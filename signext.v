`timescale 1ns / 1ps



module signext(
    input wire [15:0] a,
    input wire [1:0] type,
    output reg [31:0] y
    );
    
    always @(*) begin
        case(type)
            2'b11: y = { 16'b0,a };
            default: y = { {16{a[15]}},a };
        endcase
    end
    
endmodule
