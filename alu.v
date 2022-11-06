`timescale 1ns / 1ps

`include "defines.vh"

module alu(
    input wire [31:0] srcAE,
    input wire [31:0] srcBE,
    input wire [`AluOpBus] op,
    input wire [4:0] sa,
    input wire [31:0] pcplus8E,
    output reg [31:0] aluoutE,
    output reg overflow,
    output reg [63:0] multresultE
);

// 处理乘法所用的原操作数
wire [31:0] mult_a, mult_b;
assign mult_a = ( (op == `EXE_MULT_OP) && (srcAE[31] == 1'b1) ) ? (~srcAE + 1) : srcAE;
assign mult_b = ( (op == `EXE_MULT_OP) && (srcBE[31] == 1'b1) ) ? (~srcBE + 1) : srcBE;

// 得到各个指令的运算结果
always @(*) begin
    case(op)
        // 算数运算指令
        `EXE_ADD_OP, `EXE_ADDU_OP: aluoutE = srcAE + srcBE;
        `EXE_SUB_OP, `EXE_SUBU_OP: aluoutE = srcAE - srcBE;
        `EXE_SLT_OP: aluoutE = $signed(srcAE) < $signed(srcBE)? 32'b1: 32'b0;
        `EXE_SLTU_OP:aluoutE = (srcAE < srcBE) ? 32'b1: 32'b0;
        `EXE_DIV_OP, `EXE_DIVU_OP: aluoutE = 32'b0;
        `EXE_MULTU_OP: multresultE = srcAE * srcBE;
        `EXE_MULT_OP:  multresultE = (srcAE[31] ^ srcBE[31] == 1'b1)? ~(mult_a * mult_b) + 1: mult_a * mult_b;
        // 逻辑运算指令
        `EXE_AND_OP: aluoutE = srcAE & srcBE;
        `EXE_OR_OP:  aluoutE = srcAE | srcBE;
        `EXE_XOR_OP: aluoutE = srcAE^srcBE;
        `EXE_NOR_OP: aluoutE = ~(srcAE | srcBE);
        `EXE_LUI_OP: aluoutE = { srcBE[15:0], 16'b0 };
        // 移位指令
        `EXE_SLL_OP: aluoutE = (srcBE << sa);
        `EXE_SRL_OP: aluoutE = (srcBE >> sa);
        `EXE_SRA_OP: aluoutE = $signed(srcBE) >>> sa ;
        `EXE_SLLV_OP:aluoutE = (srcBE << srcAE[4:0]);
        `EXE_SRLV_OP:aluoutE = (srcBE >> srcAE[4:0]);
        `EXE_SRAV_OP:aluoutE = $signed(srcBE) >>> srcAE[4:0];
        // 访存指令
        `EXE_MTHI_OP:aluoutE = srcAE;
        `EXE_MTLO_OP:aluoutE = srcAE;
        // 分支跳转指令（部分）
        `EXE_JAL_OP, `EXE_JALR_OP, `EXE_BLTZAL_OP, `EXE_BGEZAL_OP: aluoutE = pcplus8E;
        `EXE_LB_OP, `EXE_LBU_OP, `EXE_LH_OP, `EXE_LHU_OP,
        `EXE_LW_OP, `EXE_SB_OP, `EXE_SH_OP, `EXE_SW_OP: aluoutE = srcAE + srcBE;
        // 异常指令
        `EXE_MTC0_OP: aluoutE = srcBE;
        default: aluoutE = 32'b0;
    endcase
end

// 处理有可能溢出异常的运算                
always @(*) begin
    case(op)
        `EXE_ADD_OP: overflow = srcAE[31] & srcBE[31] & ~aluoutE[31] | ~srcAE[31] & ~srcBE[31] & aluoutE[31];
        `EXE_ADDU_OP:overflow = 1'b0;
        `EXE_SUB_OP: overflow = ( (srcAE[31]&&!srcBE[31])&&!aluoutE[31] ) || ( (!srcAE[31]&&srcBE[31])&&aluoutE[31] );
        `EXE_SUBU_OP:overflow = 1'b0;
        default:  overflow = 1'b0;
    endcase
end
                
endmodule
