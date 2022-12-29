# MIPS-CPU

This repository includes a verilog implementation of a CPU capable of executing 52 base instructions and 5 special instructions in MIPS. It mainly consists of datapath, hazard, AXI, and exception handling modules

## Datapath

![img](https://raw.githubusercontent.com/FionaChan01/MIPS-CPU/main/images/datapath.png)

A datapath is a portion of a computer's hardware that is responsible for performing operations on data. It is essentially a collection of circuits and components that are designed to carry out a specific set of tasks, such as arithmetic calculations, logical operations, and memory access. The datapath is an integral part of the computer's central processing unit (CPU), and it works in conjunction with the control unit to execute instructions stored in memory.

A datapath typically consists of a number of functional units, such as an arithmetic logic unit (ALU) for performing arithmetic and logical operations, registers for storing data temporarily, and a bus for transferring data between the various units. The control unit sends signals to the datapath to indicate which operation should be performed, and the datapath executes the operation using the data stored in the registers.

The design of the datapath plays a crucial role in the performance of a computer, as it determines the speed at which operations can be carried out and the types of operations that can be performed. In modern computers, the datapath is often highly complex and can consist of multiple levels of cache memory, pipelines, and other optimization techniques to maximize performance.

```verilog
module datapath(
	input wire clk,rst,
	//--------------------------------IF-------------------------------
	input wire pcEretD, pcJudge, //Test
    input wire [`GeneralBus]instrGF,
	input wire [`GeneralBus] instruF,
	input wire [`GeneralBus] pcBrD, //Test
	output wire [`GeneralBus] pcF,

	//--------------------------------MEM-------------------------------
	input wire [3:0] signalselM,
	output wire [`GeneralBus] aluoutResM,exceptionTypeM,
	output reg [`GeneralBus] writedataactualM,
	output wire [`selectionactualBus] selectionactualM,
	input wire isStopAll,
	input wire isRefreshAll,
	input  wire [`GeneralBus] dataReadM,
    output wire memenM,

	//--------------------------------WB-------------------------------
	input wire pctW,//Test
	output wire [`AluRegBus] writeregW,
	output wire [`GeneralBus] pcW,resultW,

	//--------------------------------EXCEPTION-------------------------------
	input wire [`opFunctBus] interrupt,
	input wire instructionStall,
	input wire dataStall,
	input wire sofExp,
	input wire stallRequest
    );
```

The module has four main sections: IF (instruction fetch), MEM (memory access), WB (write back), and EXCEPTION.

In the IF section, there are inputs for the clock signal (clk), reset signal (rst), and various control signals (pcEretD, pcJudge, etc.). There are also inputs for instruction data (instrGF, instruF, pcBrD) and an output for the program counter (pcF).

In the MEM section, there are inputs for various control signals (signalselM, isStopAll, isRefreshAll, dataReadM) and outputs for various data signals (aluoutResM, exceptionTypeM, writedataactualM, selectionactualM). There is also an output signal (memenM) that enables memory access.

In the WB section, there is an input for a control signal (pctW) and outputs for various data signals (writeregW, pcW, resultW).

In the EXCEPTION section, there are inputs for various control signals (interrupt, instructionStall, dataStall, sofExp, stallRequest). These signals are related to handling exceptions or errors that may occur during execution.

```verilog
//--------------------------------IF-------------------------------
	wire isInDelaySlotF;
	wire stopFetch, refreshF;
//--------------------------------FD-------------------------------
	wire [`GeneralBus] NextPcD,pcAdd4F;
	wire [`GeneralBus] branchPcD,brNextPcD,PcOfJumpD;
//--------------------------------ID-------------------------------
	wire regdstD,regwriteD,memenD;
	wire refreshD,stopD,IsDelaySlot_D,invalidD,equalD;
	wire pcsrcD,branchD,jumpD, jalD, jrD, balD;
	wire PCbranchLogin;
	wire syscallD, breakD, eretD;
	wire [`GeneralBus] signimmD,signimmshD,pcAdd8D;
    wire [`AluOpBus] alucontrolD;
	wire branchTes;
    wire HiloWriteEnaD;
	wire [`exceptBus] exceptionF;
	wire [`GeneralBus] src_a_D,src_a2_D;
	wire [`GeneralBus] pcD,pcAdd4D,instrD;
	wire [`GeneralBus] srca2aD,srca2bD;
	wire srcb2aD,srcb2bD;
	wire  forwardaD,forwardbD;
	wire [`AluRegBus] rdD,saD,rtD,rsD;
	wire [`GeneralBus] src_b_D,src_b2_D;
	
//--------------------------------EXE-------------------------------
	wire memwriteE;
	wire memenE;
	wire memtoregE;
	wire [`RegBus] configRegCP0,pridRegCP0,badadRegCP0;
	wire [`AluRegBus] rsE,rtE,rdE,saE;
	wire [`AluRegBus] writeRegInE,writeRegFinalE;
	wire [`multBus] ResMultE,ResDIvE;
	wire StallDivE,ValidDivE,SignDivE,ReadyDivE,BeginDivE;
	wire EwrightA,EwrightB;
	wire [`exceptBus] execeptionE;
	wire isDelaySlotInstrE;
	wire overflow;
	wire stopE;
	wire refreshE;
	wire [`opFunctBus] opD,functD;
	wire [`exceptBus] execeptionD;
	wire [`RegBus] compareRegCP0;
	reg [`controlsignalBus] controlsD;
    reg [`hilo_cpBus]  HiloCP0D;
	wire[`AluOpBus] alucontrolE;
	wire [`RegBus] countRegCP0,epcRegCP0;
	wire [`forwardBus] forwardaE,forwardbE;	
	assign isStopAll = stopE | stopFetch;
	assign isRefreshAll = refreshE | refreshF | refreshD;
    assign pcJudge = (|exceptM) & !stopFetch;
//--------------------------------MEM-------------------------------
	wire [`GeneralBus] pcM;
	wire [`AluRegBus] rdM,writeregM;
	wire [`GeneralBus] writedataM,resultM,aluresultM;
	reg [`GeneralBus] readdataactualM;
	wire [`GeneralBus] hiwritedatamdM,lowritedatamdM;
	wire hilo_writeM ;
	wire memtoregD,memwriteD,alusrcD;
	wire [`multBus] multresultM,divresultM,cp0timer_int;
	reg [`selectionactualBus] selM;
	wire [`exceptBus] exceptM;
	wire cp0toregE, alusrcE, regdstE, regwriteE, jalE,balE,hilo_writeE;
	wire [`GeneralBus] pcE,aluoutE,pcAdd8E,signimmE;
	wire [`GeneralBus] src_a_E,src_a2_E,srcbE,src_b2_E,srcb3E;
	wire except_enM,is_in_delayslotM;
	reg adelM,adesM;
	wire ifMtc0,ifMfc0;
	wire [`GeneralBus] wrongAdrM,newpcM;
	wire [`GeneralBus] hiwritedataM,lowritedataM;
	wire [`GeneralBus] aluresult_hiM,looutM,hioutM;
	wire [`RegBus] CP0Status_o, CP0Cause_o, CP0Data_o;
	wire memtoregM,stopM,refreshM,regwriteM,CP0WenM,CP02GPRM;
	wire [`AluOpBus] alucontrolM;
	reg [`lhdataBus] lhdatatempM;
	reg isStopAllM;
	reg isRefreshAllM;
	reg [`exceptBus] lbdatatempM;
	reg numextendM;
	
```

In the IF stage, there are input signals for the clock (clk) and reset (rst) and various control signals (stopFetch, refreshF, etc.).

In the FD stage, there are input and output signals for the program counter (NextPcD, pcAdd4F, branchPcD, brNextPcD, PcOfJumpD) and various control signals (equalD, invalidD, IsDelaySlot_D, etc.).

In the ID stage, there are input and output signals for instruction data (instrD, srca2aD, srca2bD, etc.), control signals (regdstD, regwriteD, memenD, etc.), and data signals for registers (rdD, saD, rtD, rsD).

In the EXE stage, there are input and output signals for various control signals (memwriteE, memenE, memtoregE, etc.), data signals for registers (rsE, rtE, rdE, saE), and signals for the result of various operations (ResMultE, ResDIvE, etc.).

In the MEM stage, there are input and output signals for various control signals (memtoregD, memwriteD, alusrcD, etc.), data signals for registers (rdM, writeregM), and signals for the result of memory accesses (writedataM, resultM, aluresultM).

There are also various other signals in the code, such as signals for handling exceptions (exceptM, except_enM, etc.), signals for handling the program counter (pcJudge, etc.), and signals for handling registers (HiloCP0D, HiloWriteEnaD, etc.).

## Hazard

```verilog
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
			else if(rsE == writeregW & regwriteW) //????????????????????¡¤??????????????????????????????¡¤
				forwardaE = 2'b01;
			else forwardaE = 2'b00; // ?????????
		end
		else forwardaE = 2'b00; // ?????????
	end

	always @(*) begin // ????R???????rt???????????? 
		if(rtE != 0) begin
			if(rtE == writeregM & regwriteM) //???????????????????¡¤
				forwardbE = 2'b10;
			else if(rtE == writeregW & regwriteW) //????????????????????¡¤??????????????????????????????¡¤
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
```

This code defines a module called "hazard." The module has input and output signals that correspond to different stages of a digital system's pipeline: fetch (stallF, flushF), decode (rsD, rtD, branchD, pcsrcD, etc.), execute (rsE, rtE, writeRegFinalE, etc.), memory (writeregM, regwriteM, etc.), and write back (writeregW, regwriteW).

The module implements a hazard detection and prevention system for the digital system. It has a number of logic blocks that check for various hazards, such as data hazards, control hazards, and structural hazards. If a hazard is detected, the module will set various output signals to indicate that the pipeline should be stalled or flushed.

For example, the module has a logic block that checks for data hazards by comparing the values of the rsD and rtD input signals (which hold the source register values for the decode stage) with the values of the writeregM and writeregW input signals (which hold the destination register values for the memory and write back stages). If a match is found, the module will set the forwardaD and forwardbD output signals to indicate that data forwarding should be used to resolve the hazard.

The module also has logic blocks that check for control hazards (such as branches and jumps) and structural hazards (such as resource conflicts). If a hazard is detected, the module will set various output signals (stopD, refreshD, stopE, refreshE, etc.) to indicate that the pipeline should be stopped or flushed.

## Exception

![img](https://raw.githubusercontent.com/FionaChan01/MIPS-CPU/main/images/exception.png)

Exception handling in a MIPS CPU refers to the process of responding to an exceptional event or condition that occurs while the CPU is executing instructions. An exception is a interrupt or error that occurs during the execution of a program and requires the CPU to take special action.

In a MIPS CPU, exceptions are handled using a special mechanism called an exception vector. The exception vector is a table of addresses that corresponds to different types of exceptions. When an exception occurs, the CPU uses the exception vector to determine the address of the handler routine for the specific type of exception that occurred. The handler routine is then executed to handle the exception.

Some common examples of exceptions that might occur in a MIPS CPU include:

- Interrupts: Interrupts are events that occur asynchronously to the normal execution of the program. They can be triggered by external devices such as keyboards, mice, or networking hardware, or by internal events such as timer overflows. When an interrupt occurs, the CPU stops executing the current instruction and jumps to the address of the interrupt handler routine to process the interrupt.
- Memory access violations: If a program tries to access memory that it does not have permission to access (e.g. trying to read from a protected memory location), a memory access violation exception will be triggered.
- Arithmetic errors: If a program tries to perform an invalid arithmetic operation (e.g. dividing by zero), an arithmetic error exception will be triggered.
- Illegal instruction: If a program tries to execute an instruction that is not supported by the CPU, an illegal instruction exception will be triggered.

Overall, exception handling is an important part of the operation of a MIPS CPU, as it allows the CPU to respond to exceptional events and continue executing instructions correctly.

```verilog
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

```

This code defines a module handling exceptions in CPU:

- `rst`: a reset signal
- `exception`: a bus of bits representing various exceptions
- `instrGF`: an instruction signal
- `isADEL`: a signal indicating whether an address error occurred in the load
- `isADES`: a signal indicating whether an address error occurred in the store
- `StatusCP0_i`: a bus of bits representing the status of a coprocessor
- `CaseCP0_i`: a bus of bits representing the case of the coprocessor

The output of the module is:

- `exceptionType_o`: a bus of bits representing the type of exception that occurred

The module has an always block with a sensitivity list of `(*).` This means that the block will execute whenever any of the variables in the sensitivity list change. The sensitivity list is empty, so the block will execute on every clock cycle.

Inside the always block, the module has a number of conditional statements that check the values of the input signals. If certain conditions are met, the output `exceptionType_o` is assigned a value corresponding to a particular type of exception. If none of the conditions are met, the output is assigned a default value of ``ZeroWord`.

The `include "defines.vh"` line at the top of the code indicates that the file "defines.vh" should be included in the current Verilog source file. This file is likely to contain definitions of various constants and macros that are used in the module. The `timescale` directive sets the time scaling for the simulation, specifying the time unit and the precision. In this case, the time unit is 1 nanosecond and the precision is 1 picosecond.

## AXI

AXI stands for Advanced eXtensible Interface. It is a high-performance, modular interface for system-on-chip (SoC) designs that is widely used in the design of microprocessors and other digital devices.

In the context of a MIPS CPU, AXI may be used as an interface between the CPU and other components in the system, such as memory, peripherals, and coprocessors. It is a bus-based interface, which means that it provides a communication channel between different components in the system over which data can be transferred. AXI allows for high-speed data transfer between different components, and it is designed to be scalable, flexible, and easy to use.

There are different versions of the AXI interface, each with its own set of features and capabilities. The version used in a MIPS CPU would depend on the specific requirements of the system and the capabilities of the CPU.

```verilog
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
```


 
