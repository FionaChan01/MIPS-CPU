`timescale 1ns / 1ps

`include "defines.vh"

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
    output wire memenM, //内存访问使能

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
	reg adelM,adesM; // 在内存阶段加载异常,并保存内存异常
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
	
//-------------------------------------------------------Test BY Z-----------------------------------------------
	 always@(posedge clk) begin
	    if(rst) begin
            isStopAllM <= 1'b0;
            isRefreshAllM <= 1'b0;
        end	else 
        begin   
        case (signalselM)
            4'b0010:begin
                isStopAllM <= dataStall & instructionStall;
                isRefreshAllM <= stallRequest;
            end
            4'b0011:begin
                isStopAllM <= stallRequest;
                isRefreshAllM <= dataStall & instructionStall;
            end
            4'b0100:begin
               isStopAllM <= ~stallRequest;
               isRefreshAllM <= !dataStall & instructionStall;
            end
            4'b0101:begin
                isStopAllM <= stallRequest;
                isRefreshAllM <= dataStall & ~instructionStall;
            end
            4'b0111:begin
                isStopAllM <= stallRequest;
                isRefreshAllM <= dataStall & ~instructionStall;
            end
            4'b1000:begin
               isStopAllM <= stopFetch & refreshF;
               isRefreshAllM <= !stopFetch & refreshF;
            end
            4'b1001:begin
               isStopAllM <= stopFetch | refreshF;
               isRefreshAllM <= !stopFetch & refreshF;
            end
            default: begin
                isStopAllM <= 1'b0;
                isRefreshAllM <= 1'b0;
            end
        endcase
        end
    end
//-------------------------------------------------------Test-----------------------------------------------

//--------------------------------WB-------------------------------
	wire [`GeneralBus] aluoutW,readdataW;
	wire refreshW,memtoregW,regwriteW;

//--------------------------------EXCEPTION-------------------------------
    wire wenCP0D,wenCP0E; // cp0写使能信号
    wire RegDofCP0;

//--------------------------------NEXT PC-------------------------------   
    assign pctW = pcEretD | pcBrD;
    assign brNextPcD = (pcsrcD == 1'b1) ? branchPcD : pcAdd4F;
    assign PcOfJumpD = (jrD == 1'b1) ? src_a2_D : {pcAdd4D[31:28],instrD[25:0],2'b00};
	assign NextPcD = (jumpD | jalD | jrD) ? PcOfJumpD : brNextPcD;
    assign except_enM = (exceptionTypeM == 32'h0);
    assign pcEretD = (pcsrcD == 1'b1)? (|branchPcD) : (| pcAdd4F );
    assign sofExp = (exceptionTypeM == 32'h0);
    assign pcBrD = (jumpD | jalD | jrD) ? (|PcOfJumpD) : (brNextPcD);
//--------------------------------REG FILE-------------------------------
	regfile rf (.clk(~clk),.we3(regwriteW),.ra1(rsD),.ra2(rtD),.wa3(writeregW),.wd3(resultW),.rd1(src_a_D),.rd2(src_b_D));
//--------------------------------------------------------------------------------------------------------------------


//[================================================== F ==============================================]
	pc #(32) PCReg (.clk(clk),.rst(rst), .en(~stopFetch), .clr(refreshF),.d(NextPcD), .t(newpcM), .q(pcF));
	assign pcAdd4F = pcF + 32'b100;
	assign exceptionF = (pcF[`forwardBus] == 2'b00) ? 8'b00000000 : 8'b10000000;//the addr error
	assign isInDelaySlotF = (jumpD|jrD|jalD|branchD);
	assign instrGF = exceptionF | 32'h1;//Test
//[================================================== D ==============================================]
	
	assign opD = instrD[31:26];
	assign rsD = instrD[25:21];
	assign rtD = instrD[20:16];
	assign rdD = instrD[15:11];
	assign saD = instrD[10:6];
	assign functD = instrD[`opFunctBus];
	assign { regwriteD, memwriteD, memtoregD , alusrcD, regdstD, branchD, jumpD, jalD, jrD, balD, memenD, alucontrolD, invalidD } = controlsD;
    assign { wenCP0D, RegDofCP0, HiloWriteEnaD } = HiloCP0D;
	
	// 异常相关信号赋值
	assign syscallD = ( { opD, functD } == { `EXE_RTYPE, `EXE_SYSCALL } ) & ~stallRequest;
	assign breakD = ( { opD, functD } == { `EXE_RTYPE, `EXE_BREAK } ) & ~stallRequest;
	assign eretD = (instrD == `EXE_ERET) & ~stallRequest;
	assign pcsrcD = branchD & equalD ;
	flopenrc #(32) r1D ( .clk(clk),.rst(rst), .en(~stopD),.clear(refreshD),.d(pcAdd4F),.q(pcAdd4D) );
	flopenrc #(8) r3D ( .clk(clk), .rst(rst), .en(~stopD), .clear(refreshD),.d(exceptionF), .q(execeptionD) );
	flopenrc #(1) r4D ( .clk(clk), .rst(rst), .en(~stopD), .clear(refreshD),.d(isInDelaySlotF), .q(IsDelaySlot_D) );


	signext se ( .a(instrD[`lhdataBus]), .type(instrD[29:28]), .y(signimmD) );
	sl2 immsh ( .a(signimmD), .y(signimmshD) );

	// 指令相关控制信号的处理
	always @(*)
    begin
        case(opD)
            `EXE_PRIVILEGE:
            if(~stallRequest) begin
                case(rsD)
                    `EXE_MTC0_RS: HiloCP0D = 3'b100;
                    `EXE_MFC0_RS: HiloCP0D = 3'b010;
                    default: HiloCP0D = 3'b000;
                endcase
            end 
            else begin
                HiloCP0D = 3'b000;
            end
            `EXE_RTYPE:
                case(functD)
                    `EXE_MULT, `EXE_MULTU, `EXE_DIV, `EXE_DIVU,`EXE_MTHI, `EXE_MTLO: HiloCP0D = 3'b001;
                    default: HiloCP0D = 3'b000;
                endcase
            default: HiloCP0D = 3'b000;
        endcase
    end
    
	// 指令相关控制信号的处理
    always @(*)
    begin
        controlsD=0;
        if(~stallRequest)begin
        case(opD)
		// ------------------------------------------RTYPE------------------------------------------
            `EXE_RTYPE:
                case(functD)
                    // 算术运算指令
                    `EXE_ADD:  controlsD = { 11'b10001000000, 8'b00100000,  1'b0 };
                    `EXE_ADDU: controlsD = { 11'b10001000000, 8'b00100001, 1'b0 };
					`EXE_SLTU: controlsD = { 11'b10001000000, 8'b00101011, 1'b0 };
					`EXE_SLT:  controlsD = { 11'b10001000000, 8'b00101010,  1'b0 };
					`EXE_MULT: controlsD = { 11'b00000000000, 8'b00011000,  1'b0 };
                    `EXE_SUBU: controlsD = { 11'b10001000000, 8'b00100011, 1'b0 };
                    `EXE_MULTU:controlsD = { 11'b00000000000, 8'b00011001, 1'b0 };
                    `EXE_SUB:  controlsD = { 11'b10001000000, 8'b00100010,  1'b0 };
                    `EXE_DIV:  controlsD = { 11'b00000000000, 8'b00011010,   1'b0 };
                    `EXE_DIVU: controlsD = { 11'b00000000000, 8'b00011011,  1'b0 };
                    // 逻辑运算指令
                    `EXE_AND:  controlsD = { 11'b10001000000, 8'b00100100,  1'b0 };
                    `EXE_XOR:  controlsD = { 11'b10001000000, 8'b00100110,  1'b0 };
                    `EXE_OR:   controlsD = { 11'b10001000000, 8'b00100101,   1'b0 };
                    `EXE_NOR:  controlsD = { 11'b10001000000, 8'b00100111,  1'b0 };
                    // 移位指令
                    `EXE_SLL:  controlsD = { 11'b10001000000, 8'b01111100,  1'b0 };
                    `EXE_SRL:  controlsD = { 11'b10001000000, 8'b00000010,  1'b0 };
                    `EXE_SRA:  controlsD = { 11'b10001000000, 8'b00000011,  1'b0 };
                    `EXE_SLLV: controlsD = { 11'b10001000000, 8'b00000100, 1'b0 };
                    `EXE_SRLV: controlsD = { 11'b10001000000, 8'b00000110, 1'b0 };
                    `EXE_SRAV: controlsD = { 11'b10001000000, 8'b00000111, 1'b0 };
                    // 数据移动指令
                    `EXE_MFHI: controlsD = { 11'b10001000000,8'b00010000, 1'b0 };
                    `EXE_MFLO: controlsD = { 11'b10001000000,8'b00010010, 1'b0 };
					`EXE_FGI: controlsD = { 11'b10001000000, 8'b10101010, 1'b0 };
                    `EXE_MTHI: controlsD = { 11'b00000000000,8'b00010001, 1'b0 };
                    `EXE_MTLO: controlsD = { 11'b00000000000,8'b00010011, 1'b0 };
                    // 跳转指令
                    `EXE_JR:   controlsD = { 11'b00000000100, 8'b00000000, 1'b0 };
                    `EXE_JALR: controlsD = { 11'b10001001100, 8'b00001001, 1'b0 };
                    // 内陷指令
                    `EXE_BREAK:  controlsD = (~stopD)? { 11'b00000000000, 8'b00000000, 1'b0 } : 20'b1;
                    `EXE_SYSCALL:controlsD = (~stopD)? { 11'b00000000000, 8'b00000000, 1'b0 }:20'b1;
                    default: controlsD = 20'b1; // illegal instruction
                endcase
			// ------------------------------------------ITYPE------------------------------------------
            // 算术运算指令
            `EXE_ADDI: controlsD = { 11'b10010000000, 8'b00100000,   1'b0 };
            `EXE_ADDIU:controlsD = { 11'b10010000000, 8'b00100001,  1'b0 };
            `EXE_SLTI: controlsD = { 11'b10010000000, 8'b00101010,   1'b0 };
            `EXE_SLTIU:controlsD = { 11'b10010000000, 8'b00101011,  1'b0 };
            // 逻辑运算指令
            `EXE_ANDI: controlsD = { 11'b10010000000, 8'b00100100,   1'b0 };
            `EXE_XORI: controlsD = { 11'b10010000000, 8'b00100110,   1'b0 };
            `EXE_ORI:  controlsD = { 11'b10010000000,  8'b00100101,    1'b0 };
            `EXE_LUI:  controlsD = { 11'b10010000000,  8'b01011100,   1'b0 };
            // 分支跳转指令
            `EXE_BEQ, `EXE_BNE,`EXE_BGTZ,`EXE_BLEZ :  controlsD = { 11'b00000100000,  8'b00000000,  1'b0 };
            `EXE_REGIMM_INST: 
                case(rtD)
                    `EXE_BLTZ, `EXE_BGEZ: controlsD = { 11'b00000100000,  8'b00000000,  1'b0 };
                    `EXE_BLTZAL: controlsD = { 11'b10000100010, 8'b01001010,  1'b0 };
                    `EXE_BGEZAL: controlsD = { 11'b10000100010, 8'b01001011,  1'b0 };
                    default: controlsD = 20'b1; // illegal instruction
                endcase
            // 访存指令
            `EXE_LW:   controlsD = { 11'b10110000001,  8'b11100011,   1'b0 };
            `EXE_LB:   controlsD = { 11'b10110000001,  8'b11100000,   1'b0 };
            `EXE_LBU:  controlsD = { 11'b10110000001,  8'b11100100,  1'b0 };
            `EXE_LH:   controlsD = { 11'b10110000001,  8'b11100001,   1'b0 };
            `EXE_LHU:  controlsD = { 11'b10110000001,  8'b11100101,  1'b0 };
			`EXE_FGI: controlsD = { 11'b10001000000, 8'b10101010, 1'b0 };
            `EXE_SW:   controlsD = { 11'b01010000001,  8'b11101011,   1'b0 };
            `EXE_SB:   controlsD = { 11'b01010000001,  8'b11101000,   1'b0 };
            `EXE_SH:   controlsD = { 11'b01010000001,  8'b11101001,   1'b0 };

			// ------------------------------------------JTYPE------------------------------------------
            `EXE_J:    controlsD = { 11'b00000010000, 8'b00000000,  1'b0 };
            `EXE_JAL:  controlsD = { 11'b10000001000,  8'b01010000,   1'b0 };


            // 特权指令
            `EXE_PRIVILEGE:
                case(rsD)
                    `EXE_MTC0_RS: controlsD = { 11'b00000000000, 8'b01100000, 1'b0 };
                    `EXE_MFC0_RS: controlsD = { 11'b10000000000, 8'b01011101, 1'b0 };
                    `EXE_ERET_RS: controlsD = { 11'b00000000000, 8'b00000000, 1'b0 };
                    default: controlsD = 20'b1; // illegal instruction
                endcase
            default: controlsD = 20'b1; // illegal instruction
        endcase
       end
    end

    

//[================================================== E ==============================================]
	flopenrc #(32) r1E ( .clk(clk), .rst(rst), .en(~stopE), .clear(refreshE),.d(src_a_D), .q(src_a_E) );
	flopenrc #(32) r2E ( .clk(clk), .rst(rst), .en(~stopE), .clear(refreshE),.d(src_b_D), .q(srcbE) );
	flopenrc #(32) r3E ( .clk(clk), .rst(rst), .en(~stopE), .clear(refreshE),.d(signimmD), .q(signimmE) );
	flopenrc #(32) r8E ( .clk(clk), .rst(rst), .en(~stopE), .clear(refreshE),.d(pcAdd8D), .q(pcAdd8E) );
	flopenrc #(32) rpcE( .clk(clk), .rst(rst), .en(~stopE), .clear(refreshE),.d(pcD), .q(pcE) );
	// 判断异常指令
	flopenrc #(8) r9E ( .clk(clk), .rst(rst), .en(~stopE), .clear(refreshE),.d({execeptionD[7],syscallD,breakD,eretD,invalidD,execeptionD[`hilo_cpBus]}),.q(execeptionE));
	flopenrc #(1) r10E( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshE),.d(IsDelaySlot_D), .q(isDelaySlotInstrE) );
	flopenrc #(19) r11E(.clk(clk),.rst(rst),.en(~stopE),.clear(refreshE),
						.d({memtoregD,memwriteD,alusrcD,regdstD,regwriteD,alucontrolD,HiloWriteEnaD,jalD,balD,memenD,wenCP0D,RegDofCP0}),
						.q({memtoregE,memwriteE,alusrcE,regdstE,regwriteE,alucontrolE,hilo_writeE,jalE,balE,memenE,wenCP0E,cp0toregE})
						);

	mux3 #(32) muxForwardAE( .d0(src_a_E), .d1(resultW), .d2(aluoutResM),.s(forwardaE), .y(src_a2_E) );
	mux3 #(32) muxForwardBE( .d0(srcbE), .d1(resultW), .d2(aluoutResM),.s(forwardbE), .y(src_b2_E) );
    assign srcb3E = (alusrcE == 1) ? signimmE : src_b2_E;
	// alu 运算单元
	alu alu (.srcAE(src_a2_E),
	         .srcBE(srcb3E), 
	         .op(alucontrolE),
	         .pcplus8E(pcAdd8E),
	         .sa(saE), 
	         .aluoutE (aluoutE),
	         .overflow(overflow),
			 .multresultE(ResMultE));
	
	assign branchPcD = pcAdd4D + signimmshD;
    assign pcAdd8D = pcAdd4D + 32'b100;
    assign src_a2_D = (forwardaD == 1'b1) ? resultM : src_a_D;
    assign src_b2_D = (forwardbD == 1'b1) ? resultM : src_b_D;
	eqcmp compare (.opA(src_a2_D),.opB(src_b2_D), .op(opD), .sel(rtD), .res (equalD));
	// 除法运算所需的信号
	assign ValidDivE = (alucontrolE == 8'b00011010) | (alucontrolE == 8'b00011011);
	assign SignDivE =  (alucontrolE == 8'b00011010);

	// 除法所需状态机
	divFSM dFSM(.clk(clk),
				.rst(rst),
    			.ReadyDivE(ReadyDivE),
				.ValidDivE(ValidDivE),
    			.StallDivE(StallDivE), 
				.BeginDivE(BeginDivE)
				);

	// 除法器件div
	div div(.clk(clk),
			.rst(rst),
			.signed_div_i(SignDivE),
			.opdata1_i(src_a2_E),
			.opdata2_i(srcb3E),
			.start_i(BeginDivE),
			.annul_i(`DivStop),
			.result_o(ResDIvE),
			.ready_o(ReadyDivE));

	assign writeRegInE = (regdstE == 1'b1) ? rdE : rtE;
	assign writeRegFinalE = ((jalE | balE) == 1'b1) ? 5'b11111 : writeRegInE;
//[================================================== M ==============================================]
	flopenrc #(32) r1M ( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshM),.d(src_b2_E), .q(writedataM) );
	flopenrc #(32) r2M ( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshM),.d(aluoutE), .q(aluresultM) );
	flopenrc #(1) r6M ( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshM),.d(isDelaySlotInstrE), .q(is_in_delayslotM) );
	flopenrc #(32) r2D ( .clk(clk), .rst(rst), .en(~stopD),.clear(refreshD),.d(instruF), .q(instrD) );
	flopenrc #(32) rpcD (.clk(clk),.rst(rst), .en(~stopD),.clear(refreshD),.d(pcF),.q(pcD) );
	floprc #(5) r3W ( .clk(clk), .rst(rst), .clear(refreshW),.d(writeregM), .q(writeregW) );
	assign {hiwritedatamdM,lowritedatamdM} = (alucontrolM==8'b00011000 | alucontrolM==8'b00011001) ? multresultM : divresultM;// hilo寄存器
	assign hiwritedataM = (alucontrolM==8'b00010001) ? aluresultM : hiwritedatamdM;// 判断写hilo的结果是来自于乘除法的结果还是alu的结果
	assign lowritedataM = (alucontrolM==8'b00010011) ? aluresultM : lowritedatamdM;


	hilo_reg hilo ( .clk(clk),
	                .rst(rst),
	                .we(hilo_writeM && ~stopM && except_enM),
	                .hi(hiwritedataM), 
	                .lo(lowritedataM), 
	                .hi_o(hioutM), 
	                .lo_o(looutM) );
	
	// 用于确定写回寄存器的值来自hilo寄存器还是alu的输出
	assign aluresult_hiM = (alucontrolM==8'b00010000) ? hioutM : aluresultM;
	assign aluoutResM = (alucontrolM==8'b00010010) ? looutM : aluresult_hiM;
	
	// memsel模块，实现load、store指令
	always @(*)
		begin
			// 初始化为store、load无异常
			adesM = 1'b0;
			adelM = 1'b0;
			writedataactualM = writedataM;
			readdataactualM = dataReadM;
			selM = 4'b0000;
			case(alucontrolM)

			// load指令
				8'b11100011: begin 
					if (aluoutResM[`forwardBus] == 2'b00) readdataactualM = dataReadM;
					else adelM = 1'b1;
				end
				8'b11100001: begin
					case(aluoutResM[`forwardBus])
						// 根据地址后两位，读取一个半字的值并进行符号位扩展
						2'b10: begin lhdatatempM = dataReadM[31:16] ; numextendM = dataReadM[31]; end
						2'b00: begin lhdatatempM = dataReadM[`lhdataBus]; numextendM = dataReadM[15]; end
						default: adelM = 1'b1;
					endcase
					if(alucontrolM==8'b11100101) numextendM = 1'b0;
					readdataactualM = { {16{numextendM}}, lhdatatempM };
				end
				8'b11100101:begin
					case(aluoutResM[`forwardBus])
						// 根据地址后两位，读取一个半字的值并进行符号位扩展
						2'b10: begin lhdatatempM = dataReadM[31:16] ; numextendM = dataReadM[31]; end
						2'b00: begin lhdatatempM = dataReadM[`lhdataBus]; numextendM = dataReadM[15]; end
						default: adelM = 1'b1;
					endcase
					if(alucontrolM==8'b11100101) numextendM = 1'b0;
					readdataactualM = { {16{numextendM}}, lhdatatempM };
				end

				8'b11100000: begin
					case(aluoutResM[`forwardBus])
						2'b01: begin lbdatatempM = dataReadM[15:8] ; numextendM = dataReadM[15]; end 
						2'b00: begin lbdatatempM = dataReadM[`exceptBus]; numextendM = dataReadM[7]; end 
						2'b11: begin lbdatatempM = dataReadM[31:24] ; numextendM = dataReadM[31]; end
						2'b10: begin lbdatatempM = dataReadM[23:16]; numextendM = dataReadM[23]; end 
					endcase
					if(alucontrolM==8'b11100100) numextendM = 1'b0; 
					readdataactualM = {{24{numextendM}}, lbdatatempM};
				end
				8'b11100100:begin
					case(aluoutResM[`forwardBus])
						2'b01: begin lbdatatempM = dataReadM[15:8] ; numextendM = dataReadM[15]; end 
						2'b00: begin lbdatatempM = dataReadM[`exceptBus]; numextendM = dataReadM[7]; end 
						2'b11: begin lbdatatempM = dataReadM[31:24] ; numextendM = dataReadM[31]; end
						2'b10: begin lbdatatempM = dataReadM[23:16]; numextendM = dataReadM[23]; end 
					endcase
					if(alucontrolM==8'b11100100) numextendM = 1'b0; 
					readdataactualM = {{24{numextendM}}, lbdatatempM};
				end

			// store指令
				8'b11101011: begin
					// 检查是否有地址不对齐异常，对相应控制信号赋值
					if (aluoutResM[`forwardBus] == 2'b00) selM = 4'b1111;
					else begin
						adesM = 1'b1;
						selM = 4'b0000;
					end
				end

				8'b11101001: begin
					// 写半字，将写的数据分割后，取低半字
					writedataactualM = { writedataM[`lhdataBus], writedataM[`lhdataBus] };
					// 根据地址的最后两位判断写的位置
					case (aluoutResM[`forwardBus])
						2'b10: selM = 4'b1100;
						2'b00: selM = 4'b0011;
						default: begin
							adesM = 1'b1;
							selM = 4'b0000;
						end
					endcase
				end

				8'b11101000: begin
					// 写字节，将写的数据分割后，取低字节
					writedataactualM = { writedataM[`exceptBus], writedataM[`exceptBus], writedataM[`exceptBus], writedataM[`exceptBus] };
					// 根据地址的最后两位判断写的位置
					case (aluoutResM[`forwardBus])
						2'b11:selM = 4'b1000;
						2'b10:selM = 4'b0100;
						2'b01:selM = 4'b0010;
						2'b00:selM = 4'b0001;
					endcase
				end
			
			endcase
		end
    
                                    
    assign wrongAdrM = (adelM == 1'b1 || adesM == 1'b1) ? aluoutResM : pcM;
	assign selectionactualM = selM & { 4{except_enM} };

	// 异常处理
	exception exp(.rst(rst),
	              .exception(exceptM),
                  .instrGF(instrGF),
	              .isADEL(adelM),
	              .isADES(adesM),
	              .StatusCP0_i(CP0Status_o),
	              .CaseCP0_i(CP0Cause_o),
	              .exceptionType_o(exceptionTypeM));

	// cp0
	cp0_reg cp0( .clk(clk),
		         .rst(rst),
		         .we_i(CP0WenM),
		         .waddr_i(rdM),
		         .raddr_i(rdM),
		         .data_i(aluoutResM),
		         .int_i(interrupt),
		         .excepttype_i(exceptionTypeM),
		         .current_inst_addr_i(pcM),
		         .is_in_delayslot_i(is_in_delayslotM),
		         .bad_addr_i(wrongAdrM),
		         .data_o(CP0Data_o),
		         .count_o(countRegCP0),
		         .compare_o(compareRegCP0),
		         .status_o(CP0Status_o),
		         .cause_o(CP0Cause_o),
		         .epc_o(epcRegCP0),
		         .config_o(configRegCP0),
		         .prid_o(pridRegCP0),
		         .badadRegCP0(badadRegCP0),
		         .timer_int_o(cp0timer_int));	           
    
	assign resultM = (~memtoregM  && ~CP02GPRM) ? aluoutResM:
									 (memtoregM) ? readdataactualM:
					 							   CP0Data_o;

//[================================================== W ==============================================]
	floprc #(32) r1W ( .clk(clk), .rst(rst), .clear(refreshW),.d(aluoutResM), .q(aluoutW) );
	floprc #(32) r2W ( .clk(clk), .rst(rst), .clear(refreshW),.d(readdataactualM), .q(readdataW) );
	flopenrc #(64) r8M ( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshM),.d(ResDIvE), .q(divresultM) );
	flopenrc #(5) r3M ( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshM),.d(writeRegFinalE), .q(writeregM));
	flopenrc #(32) rpcM( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshM),.d(pcE), .q(pcM));
	flopenrc #(5) r4E ( .clk(clk), .rst(rst), .en(~stopE), .clear(refreshE),.d(rsD), .q(rsE) );
	flopenrc #(5) r5E ( .clk(clk), .rst(rst), .en(~stopE), .clear(refreshE),.d(rtD), .q(rtE) );
	flopenrc #(5) r6E ( .clk(clk), .rst(rst), .en(~stopE), .clear(refreshE),.d(rdD), .q(rdE) );
	flopenrc #(5) r7E ( .clk(clk), .rst(rst), .en(~stopE), .clear(refreshE),.d(saD), .q(saE));
	flopenrc #(8) r4M ( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshM),.d({execeptionE[7:3],overflow,execeptionE[`forwardBus]}),.q(exceptM));
	flopenrc #(5) r5M ( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshM),.d(rdE), .q(rdM));
	flopenrc #(1) r9M ( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshM),.d(hilo_writeE), .q(hilo_writeM) );
	floprc #(32) rpcW( .clk(clk), .rst(rst), .clear(refreshW),.d(pcM), .q(pcW) );
	floprc #(32) r4W( .clk(clk), .rst(rst), .clear(refreshW),.d(resultM), .q(resultW) );
	floprc #(2) r5W(.clk(clk),.rst(rst),.clear(refreshW),
					  .d({memtoregM,regwriteM}),
					  .q({memtoregW,regwriteW})
					  );		
	flopenrc #(64) r7M ( .clk(clk), .rst(rst), .en(~stopM), .clear(refreshM),.d(ResMultE), .q(multresultM) );
	flopenrc #(13) r10M(.clk(clk),.rst(rst),.en(~stopM),.clear(refreshM),
						.d({memtoregE,regwriteE,memenE,alucontrolE,wenCP0E,cp0toregE}),
						.q({memtoregM,regwriteM,memenM,alucontrolM,CP0WenM,CP02GPRM})
						);
	//冒险处理
	hazard h(
		//----------------------IF---------------------------
		.stallF(stopFetch),
		.flushF(refreshF),
		//----------------------ID---------------------------
		.rsD(rsD),
		.rtD(rtD),
		.branchD(branchD),
		.jalD(jalD), 
		.jrD(jrD),
		.refreshD(refreshD),
		.forwardbD(forwardbD),
		.stopD(stopD),
		//----------------------EXE---------------------------
		.rsE(rsE), 
		.rtE(rtE),
		.writeRegFinalE(writeRegFinalE),
		.regwriteE(regwriteE),
		.memtoregE(memtoregE),
		.pcsrcD(pcsrcD),
		.forwardaD(forwardaD), 
		.stopE(stopE),
		.refreshE(refreshE),
		//----------------------MEM---------------------------
		.writeregM(writeregM),
		.regwriteM(regwriteM),
		.exceptionTypeM(exceptionTypeM),
		.stopM(stopM),
		.refreshM(refreshM),
		.cp0_epcM(epcRegCP0),
		.newpcM(newpcM),
		//----------------------WB---------------------------
		.writeregW(writeregW),
		.regwriteW(regwriteW),
		.forwardbE(forwardbE),
		.instructionStall(instructionStall),
		.dataStall(dataStall),
		.refreshW(refreshW),
		.cp0toregE(cp0toregE),
		.div_stallE(StallDivE),
		.forwardaE(forwardaE)
		);

endmodule
