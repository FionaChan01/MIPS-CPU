`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/1/5 PM 9:23
// Design Name: 
// Module Name: axi_interface
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


module axi_interface(
    input wire clk,
    input wire resetn,
    
    //cache port
    input wire[31:0] data_mem_addr,
    input wire data_mem_access,
    input wire data_mem_write,
    input wire[1:0] data_mem_size,
    input wire[3:0] data_mem_sel,
    output wire data_mem_ready,
    input wire[31:0] data_mem_st_data,
    output wire[31:0] data_mem_w_data,

    input wire flush,

    // axi port //
    //axi 读请求 地址 通道
    output wire[3:0] axi_read_addr_id,      //axi读取请求的id, 固定为 4'b0
    output wire[31:0] axi_read_addr_addr,   //axi读取请求的地址
    output wire[7:0] axi_read_addr_len,     //读取请求的传输长度(beats), 固定为 4'b0
    output wire[2:0] axi_read_addr_size,    //每个节拍传输的字节数(bytes per beats)
    output wire[1:0] axi_read_addr_burst,   //传输类型,  固定为 2'b01
    output wire[1:0] axi_read_addr_lock,    //原子锁, 固定为 2'b0
    output wire[3:0] axi_read_addr_cache,   //cache 属性, 固定为 4'b0
    output wire[2:0] axi_read_addr_prot,    //保护属性, 固定为 3'b0
    output wire axi_read_addr_valid,        //读请求地址握手信号，读请求地址有效
    input wire axi_read_addr_ready,         //读请求地址握手信号，slave端准备好接受数据传输
   
    //axi 读请求 数据 通道              
    input wire[3:0] axi_read_data_id,            //与axi_read_addr_id相等, 4'b0
    input wire[31:0] axi_read_data_data,         //slave传来的数据
    input wire[1:0] axi_read_data_resp,          //读请求顺利完成, 可以忽略
    input wire axi_read_data_last,               //此次读请求最后一拍数据的指示信号, 可以忽略
    input wire axi_read_data_valid,              //读请求数据有效
    output wire axi_read_data_ready,             //master准备好接受数据传输
    
    //axi 写请求 地址 通道           
    output wire[3:0] axi_write_addr_id,      //axi写请求的id, 固定为 4'b0
    output wire[31:0] axi_write_addr_addr,   //写请求的地址
    output wire[3:0] axi_write_addr_len,     //写请求的传输长度(beats), 固定为 4'b0
    output wire[2:0] axi_write_addr_size,    //每个节拍传输的字节数(bytes per beats)
    output wire[1:0] axi_write_addr_burst,   //传输类型,  固定为 2'b01
    output wire[1:0] axi_write_addr_lock,    //原子锁, 固定为 2'b0
    output wire[3:0] axi_write_addr_cache,   //cache 属性, 固定为 4'b0
    output wire[2:0] axi_write_addr_prot,    //保护属性, 固定为 3'b0
    output wire axi_write_addr_valid,        //写请求地址握手信号，写请求地址有效
    input wire axi_write_addr_ready,         //写请求地址握手信号，slave端准备好接受数据传输
   
    //axi 写请求 数据 通道     
    output wire[3:0] axi_write_data_id,       //与axi_write_addr_id相等, 4'b0
    output wire[31:0] axi_write_data_data,    //写出的数据
    output wire[3:0] axi_write_data_strb,     //字节选通位
    output wire axi_write_data_last,          //此次写请求最后一排数据的指示信号, 1'b1
    output wire axi_write_data_valid,         //写请求数据有效
    input wire axi_write_data_ready,          //slave端准备好接受数据传输
    
    //axi 写请求 响应 通道              
    input  wire[3:0] bid,       //id号
    input  wire[1:0] bresp,     //写请求成功完成
    input wire bvalid,          //写请求响应有效
    output wire bready          //master准备好接受写响应

    );
    wire read_finish;
	wire write_finish;

    wire read = data_mem_access && ~data_mem_write;
    wire write = data_mem_access && data_mem_write;
    
    reg [3:0] write_wen;
    reg  read_req;
	reg  write_req;
	reg  [1:0]  read_size;
	reg  [1:0]  write_size;
	reg  [31:0] read_addr;
	reg  [31:0] write_addr;
	reg  [31:0] write_data;
	reg  read_addr_finish;
	reg  write_addr_finish;
	reg  write_data_finish;
		
    reg flush_reg;

    always @(posedge clk) begin
        flush_reg <= ~resetn ? 1'b0:
                    flush ? 1'b1:
                    1'b0; 
    end

    always @(posedge clk) begin

        read_req   <= (~resetn) ? 1'b0 :
				 	  (read && ~read_req) ? 1'b1 :
			 		  (read_finish) ? 1'b0 : 
					  read_req;

        read_addr <= (~resetn || read_finish) ? 32'hffffffff : 
	                 (read && ~read_req || flush_reg) ? data_mem_addr :
	                 read_addr;

        read_size  <= (~resetn) ? 2'b00 :
					  (read) ? data_mem_size :
					  read_size;

        write_req  <= (~resetn) ? 1'b0 :
					  (write && ~write_req) ? 1'b1 :
					  (write_finish) ? 1'b0 : 
					  write_req;

        write_addr <= (~resetn || write_finish) ? 32'hffffffff : 
                      (write && ~write_req) ? data_mem_addr :                     
                      write_addr;

        write_size <= (~resetn) ? 2'b00 :
					  (write) ? data_mem_size :
					  write_size;

        write_wen <= (~resetn) ? 4'b0000 :
                    (write) ? data_mem_sel:
                    write_wen;

        write_data <= (~resetn) ? 32'b0 :
                    (write) ? data_mem_st_data:
                    write_data;
    end

    always @(posedge clk) begin
		read_addr_finish  <= (~resetn) ? 1'b0 :
		                     (read_req && axi_read_addr_valid && axi_read_addr_ready) ? 1'b1 :
						 	 (read_finish) ? 1'b0 :
					 		 read_addr_finish;
		write_addr_finish <= (~resetn) ? 1'b0 :
							 (write_req && axi_write_addr_valid && axi_write_addr_ready) ? 1'b1 :
							 (write_finish) ? 1'b0 :
							 write_addr_finish;
		write_data_finish <= (~resetn) ? 1'b0 :
							 (write_req && axi_write_data_valid && axi_write_data_ready) ? 1'b1 :
							 (write_finish) ? 1'b0 :
							 write_data_finish;
	end


    assign data_mem_ready = read_req && read_finish && ~flush_reg|| write_req && write_finish;
	
	assign data_mem_w_data = axi_read_data_data;	

    assign read_finish = read_addr_finish && axi_read_data_valid && axi_read_data_ready;
	assign write_finish = write_addr_finish && bvalid && bready;
		
	
	assign axi_read_addr_id = 4'b0;
	assign axi_read_addr_addr = read_addr;
    assign axi_read_addr_len = 8'b0;
	assign axi_read_addr_size = read_size;
    assign axi_read_addr_burst = 2'b01;
    assign axi_read_addr_lock = 2'b0;
    assign axi_read_addr_cache = 4'b0;
    assign axi_read_addr_prot = 3'b0;
	assign axi_read_addr_valid = read_req && ~read_addr_finish && ~flush && ~flush_reg;

	assign axi_read_data_ready = 1'b1;
	
	assign axi_write_addr_id = 4'b0;
	assign axi_write_addr_addr = write_addr;
    assign axi_write_addr_len = 8'b0;
	assign axi_write_addr_size = write_size;
    assign axi_write_addr_burst = 2'b01;
    assign axi_write_addr_lock = 2'b0;
    assign axi_write_addr_cache = 4'b0;
    assign axi_write_addr_prot = 3'b0;
	assign axi_write_addr_valid = write_req && ~write_addr_finish;

	assign axi_write_data_id = 4'b0;
	assign axi_write_data_data = write_data;
	assign axi_write_data_strb = write_wen;
    assign axi_write_data_last = 1'b1;
	assign axi_write_data_valid = write_req && ~write_data_finish;

	assign bready = 1'b1;
    
endmodule
