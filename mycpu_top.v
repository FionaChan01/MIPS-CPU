`timescale 1ns / 1ps

`include "defines.vh"

module mycpu_top(
	input wire clk,
	input wire resetn,
    input wire [`InterruptBus] interrupt, 
    output wire [`IdBus] arid,
    output wire [`GeneralBus] araddr,
    output wire [`IdBus] arlen,
    output wire [`sizeBus] arsize,
    output wire [`forwardBus] arburst,
    output wire [`forwardBus] arlock,
    output wire [`IdBus] arcache,
    output wire [`sizeBus] arprot,
    output wire arvalid,
    input  wire arready,
    //r              
    input wire [`IdBus] rid,
    input wire [`GeneralBus ] rdata,
    input wire [`forwardBus] rresp,
    input wire rlast,
    input wire rvalid,
    output wire rready,
    //aw           
    output wire [`IdBus] awid,
    output wire [`GeneralBus] awaddr,
    output wire [`IdBus] awlen,
    output wire [`sizeBus] awsize,
    output wire [`forwardBus] awburst,
    output wire [`forwardBus] awlock,
    output wire [`IdBus] awcache,
    output wire [`sizeBus] awprot,
    output awvalid,
    input  awready,
    //w          
    output wire [`IdBus] wid,
    output wire [`GeneralBus] wdata,
    output wire [`IdBus] wstrb,
    output wlast,
    output wvalid,
    input wready,
    //b              
    input wire [`IdBus] bid,
    input wire [`forwardBus] bresp,
    input bvalid,
    output bready,
    //debug
    output [`GeneralBus] debug_wb_pc,
    output [`IdBus] debug_wb_rf_wen,
    output [`DebugBus] debug_wb_rf_wnum,
    output [`GeneralBus] debug_wb_rf_wdata
    );
    //=================== datapath && debug==========================
	wire [`opFunctBus] opD,functD;
	wire [`DebugBus] rsD,rtD;
    wire invalidD, stopD, equal_BraD;
	wire pcsrcD;
    wire pcEretD;
    wire pcJudge;
    wire [`GeneralBus] instrGF;
    wire pcBrD;
	wire branchD,jumpD,jrD,bralD;
    wire stopE,refreshE;
    wire  isStopAll;
    wire  isRefreshAll;
    wire jalD,jalE,bralE;
    wire memWrite, memReadE;
    wire dataWrite,dataRead;
	wire memtoregE,regdstE;
    wire alusrcE,memtoregW;
    wire memtoregM, isSramEn;
    wire regwriteE,regwriteM,regToggleD,regwriteW;
    reg  regPredW, regMissPredM;        
	wire [`AluOpBus] alucontrolE,alucontrolM;
	wire writeHiloE, readHiloE;
	wire CP02GPRE,CP0WenM,CP02GPRM;
    reg  [`GeneralBus] CP0_data_o,CP0_data_i;
	wire stopM,refreshM;
	wire [`GeneralBus]exceptionTypeM;

    wire [`GeneralBus] aluOutResM;
    wire [`GeneralBus] writedataM;
    wire memenM;
    wire [`IdBus] wenSelM;
    wire [`GeneralBus] readdataM;
    wire data_sram_en;
    wire issramEn;
    wire data_sram_write;
    wire [1:0] data_sram_size;
    wire [`IdBus]  data_sram_wen;
    wire [`GeneralBus] data_sram_addr;
    wire [`GeneralBus] data_sram_wdata;
    wire [`GeneralBus] data_sram_rdata;

    wire [`GeneralBus] pcF;
    wire [`GeneralBus] instruF;
    wire inst_sram_en;
    wire [`IdBus] inst_sram_wen;
    wire [`GeneralBus] inst_sram_addr;
    wire [`GeneralBus] inst_sram_wdata;
    wire [`GeneralBus] inst_sram_rdata;
    wire stopW,refreshW;
    wire pctW;
    // debug
	wire [`GeneralBus] pcW;
	wire [`DebugBus] writeregW;
	wire [`GeneralBus] resultW;

    assign inst_sram_en = `WriteEnable;
    assign inst_sram_addr = pcF;
    assign issramEn = |inst_sram_en & (|data_sram_en);
    assign inst_sram_wdata = `ZeroWord;
    assign inst_sram_wen = `ZeroHalfByte;
    assign instruF  = inst_sram_rdata;
    assign data_sram_en = ~(|exceptionTypeM) & memenM;
    assign memWrite = mem_write;
    assign memReadE = |memWrite & inst_sram_en;
    assign data_sram_size = (data_sram_wen == 4'b1100 || data_sram_wen == 4'b0011) ?    2'b01:
                             (data_sram_wen == 4'b1111 || data_sram_wen == 4'b0000) ?   2'b10:
                             (data_sram_wen == 4'b0001 || data_sram_wen == 4'b0010 || data_sram_wen == 4'b0100 || data_sram_wen == 4'b1000) ? 
                             2'b00 : 2'b00;
    assign data_sram_wen = wenSelM;
    assign regToggleD = (|wenSelM) & (|writedataM); 
    assign data_sram_addr = (aluOutResM[31:16] == 16'b1011111110101111)? {16'b0001111110101111,aluOutResM[15:0]} : aluOutResM;
    assign data_sram_wdata = writedataM;
    assign data_sram_write = |data_sram_wen;
    assign readdataM = data_sram_rdata;

    
    // debug
    assign debug_wb_pc = pcW;
    assign debug_wb_rf_wen = {4{regwriteW}};
    assign debug_wb_rf_wnum  = writeregW;
    assign debug_wb_rf_wdata = resultW;

    // change sram to axi
    // use a InstruMiss signal to denote that the instruction is not loadssss    //cache mux signal
    wire cache_miss,InstrSelSignal;
    wire [`GeneralBus] i_addr,d_addr,m_addr;
    wire m_fetch,m_ld_st,mem_access;
    wire mem_write,m_st;
    wire mem_ready,memInstrOk,memDataOk,i_ready,d_ready;
    wire [`GeneralBus] mem_st_data,mem_data;
    wire [`forwardBus] mem_size,d_size;
    wire [`IdBus] m_sel,d_wen;
    wire stallIF,stallMemEn;
    wire stallReq;
    
    reg InstruMiss;
    // delete if using cache
    always @(posedge clk) begin
        if (~resetn)  InstruMiss <= 1'b1;
        if (memInstrOk & InstruMiss) InstruMiss <= 1'b0; // fetch instruction ready
        else if (~InstruMiss & data_sram_en) InstruMiss <= 1'b0;  // fetch instruction ready, but need load data, so InstruMiss maintain 0
        else if (~InstruMiss & data_sram_en & memDataOk) InstruMiss <= 1'b1; //load data ready, set InstruMiss to 1
        else InstruMiss <= 1'b1; // other conditions, set InstruMiss to 1
    end

    always@(*) begin
        case (data_sram_wen)
            4'b0010:begin
                CP0_data_o <= data_sram_wdata;
                CP0_data_i <= `ZeroWord;
            end
            4'b0011:begin
                CP0_data_o <= data_sram_wdata;
                CP0_data_i <= `ZeroWord + 1;
            end
            4'b0100:begin
                CP0_data_o <= data_sram_wdata;
                CP0_data_i <= {32{(|data_sram_wdata)}};
            end
            4'b0101:begin
                CP0_data_o <= rdata;
                CP0_data_i <= data_sram_addr;
            end
            4'b0111:begin
                CP0_data_o <= inst_sram_rdata;
                CP0_data_i <= data_sram_wdata;
            end
            4'b1000:begin
                CP0_data_o <= inst_sram_rdata;
                CP0_data_i <= aluOutResM;
            end
            4'b1001:begin
                CP0_data_o <= resultW;
                CP0_data_i <= d_addr | i_addr;
            end
            default: begin
                CP0_data_o <= `ZeroWord;
                CP0_data_i <= `ZeroWord;
            end
        endcase
    end

    assign InstrSelSignal = InstruMiss;   // use InstruMiss to select access memory(for load/store) or fetch(each instruction)
    assign m_fetch = inst_sram_en & InstruMiss; //if InstruMiss equals 0, disable the fetch strobe
    assign d_addr = (data_sram_addr[31:16] != 16'hbfaf) ? data_sram_addr : {16'h1faf,data_sram_addr[15:0]}; // modify data address, to get the data from confreg
    assign i_addr = inst_sram_addr;
    assign m_addr = InstrSelSignal ? i_addr : d_addr;
    assign m_ld_st = data_sram_en;

    assign inst_sram_rdata = mem_data;
    assign data_sram_rdata = mem_data;
    assign mem_st_data = data_sram_wdata;
    // use select signal
    assign mem_access = InstrSelSignal ? m_fetch : m_ld_st; 
    assign mem_size = InstrSelSignal ? 2'b10 : data_sram_size;
    assign m_sel = InstrSelSignal ? 4'b1111 : data_sram_wen;
    assign mem_write = InstrSelSignal ? 1'b0 : data_sram_write;
    assign stallIF = ~memInstrOk;
    //demux
    assign memInstrOk = mem_ready & InstrSelSignal;
    assign memDataOk = mem_ready & ~InstrSelSignal;
    assign stallMemEn = !memDataOk && data_sram_en;
    assign stallReq = stallMemEn || stallIF;

    always@(*) begin
        case (InstrSelSignal)
            2'b10:begin
                regPredW <= 2'b11;
            end
            2'b01:begin
                regPredW <= 2'b10;
            end
            2'b11:begin
                regPredW <= 2'b11;
            end
            default: regPredW <= 2'b11;
        endcase
    end
    
    axi_interface interface(
        .clk (clk),
        .resetn (resetn),
        
         //cache/cpu_core port
        .data_mem_addr(m_addr),
        .data_mem_access(mem_access),
        .data_mem_write(mem_write),
        .data_mem_size(mem_size),
        .data_mem_sel(m_sel),
        .data_mem_ready(mem_ready),
        .data_mem_st_data(mem_st_data),
        .data_mem_w_data(mem_data),
        // add a input signal 'flush', cancel the memory accessing operation in axi_interface, do not need any extra design. 
        .flush(|exceptionTypeM), // use excepetion type

        .axi_read_addr_id(arid),
        .axi_read_addr_addr(araddr),
        .axi_read_addr_len(arlen),
        .axi_read_addr_size(arsize),
        .axi_read_addr_burst(arburst),
        .axi_read_addr_lock(arlock),
        .axi_read_addr_cache(arcache),
        .axi_read_addr_prot(arprot),
        .axi_read_addr_valid(arvalid),
        .axi_read_addr_ready(arready),
                    
        .axi_read_data_id(rid),
        .axi_read_data_data(rdata),
        .axi_read_data_resp(rresp),
        .axi_read_data_last(rlast),
        .axi_read_data_valid(rvalid),
        .axi_read_data_ready(rready),
                
        .axi_write_addr_id(awid),
        .axi_write_addr_addr(awaddr),
        .axi_write_addr_len(awlen),
        .axi_write_addr_size(awsize),
        .axi_write_addr_burst(awburst),
        .axi_write_addr_lock(awlock),
        .axi_write_addr_cache(awcache),
        .axi_write_addr_prot(awprot),
        .axi_write_addr_valid(awvalid),
        .axi_write_addr_ready(awready),
        
        .axi_write_data_id(wid),
        .axi_write_data_data(wdata),
        .axi_write_data_strb(wstrb),
        .axi_write_data_last(wlast),
        .axi_write_data_valid(wvalid),
        .axi_write_data_ready(wready),
        
        .bid(bid),
        .bresp(bresp),
        .bvalid(bvalid),
        .bready(bready)
    );


    // ==================== datapath ====================

	datapath dp(
		.clk(clk), 
		.rst(~resetn),
		//È¡Ö¸
        .pcJudge(pcJudge),
		.signalselM(data_sram_wen),
		.pcF(pcF),
        .instrGF(instrGF),
        //ÒëÂë
        .pcEretD(pcEretD),
		.instruF(instruF),
        .pcBrD(pcBrD),
		//·Ã´æ
		.aluoutResM(aluOutResM),
		.writedataactualM(writedataM),
        .isRefreshAll(isRefreshAll),
		.dataReadM(readdataM),
		.selectionactualM(wenSelM),
        .isStopAll(isStopAll),
        .exceptionTypeM(exceptionTypeM),
		.memenM(memenM),
        .instructionStall(stallIF),
        .dataStall(stallMemEn),
		//Ð´»Ø
        .pctW(pctW),
        .pcW(pcW),
        .sofExp(sofExp),
		.resultW(resultW),
		.writeregW(writeregW),
		.interrupt(interrupt),
		.stallRequest(stallReq)
	    );
endmodule