`timescale 1ns / 1ps


module sl2 #(WIDTH=32)(
    input [WIDTH-1:0] a,
    output [WIDTH-1:0] y
    );

    // 左移两位
    assign y = { a[WIDTH-3:0], 2'b00 };

endmodule
