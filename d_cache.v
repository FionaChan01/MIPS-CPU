// 2路组相联的dCache
module d_cache (
    input wire clk, rst,
    //mips core
    input         cpu_data_req     ,//cpu发出读写请求
    input         cpu_data_wr      ,//当前请求是否是写请求
    input  [1 :0] cpu_data_size    ,//结合地址的最低两位，确定数据的有效字节
    input  [31:0] cpu_data_addr    ,//cpu请求数据的地址
    input  [31:0] cpu_data_wdata   ,//cpu写入的数据
    output [31:0] cpu_data_rdata   ,//cpu读出的数据
    output        cpu_data_addr_ok ,//Cache 返回给 Mipscore 的地址握手成功
    output        cpu_data_data_ok ,//Cache 返回给 Mipscore 的数据成功

    //axi interface
    output         cache_data_req     ,//cache发出的读写请求
    output         cache_data_wr      ,//当前请求是否是写请求
    output  [1 :0] cache_data_size    ,//结合地址的最低两位，确定数据的有效字节
    output  [31:0] cache_data_addr    ,//cache读写地址->内存
    output  [31:0] cache_data_wdata   ,//cache写入内存的数据
    input   [31:0] cache_data_rdata   ,//cache读入的数据
    input          cache_data_addr_ok ,//内存给cache的握手
    input          cache_data_data_ok  //内存返回cache数据成功
);
    //Cache配置，4 way组相联，使用伪LRU算法来选择最近未使用的路
    parameter  INDEX_WIDTH  = 8, OFFSET_WIDTH = 2;
    localparam TAG_WIDTH    = 32 - INDEX_WIDTH - OFFSET_WIDTH;
    localparam CACHE_DEEPTH = 1 << INDEX_WIDTH;

    //Cache存储单元
    reg cache_valid                [CACHE_DEEPTH - 1 : 0][3 : 0];          // 有效位    2d  cache_valid[选行][选块]
    reg[TAG_WIDTH-1:0]  cache_tag  [CACHE_DEEPTH - 1 : 0][3 : 0];          // 标记位    2d  cache_tag[选行][选块] 
    reg cache_dirty [CACHE_DEEPTH - 1 : 0][3 : 0] ;                        // 脏位      2d  cache_dirty[选行][选块] 
    reg [31:0] cache_block         [CACHE_DEEPTH - 1 : 0][3 : 0];          // block数据 2d  cache_block[选行][选块]
    reg [2:0] cache_lru             [CACHE_DEEPTH - 1 : 0];                // 伪LRU记录表，四路组相联，生成的树有三个节点
    
                         
    //访问地址分解
    wire [OFFSET_WIDTH-1:0] offset;
    wire [INDEX_WIDTH-1:0] index;
    wire [TAG_WIDTH-1:0] tag;
    
    assign offset = cpu_data_addr[OFFSET_WIDTH - 1 : 0];
    assign index = cpu_data_addr[INDEX_WIDTH + OFFSET_WIDTH - 1 : OFFSET_WIDTH];
    assign tag = cpu_data_addr[31 : INDEX_WIDTH + OFFSET_WIDTH];

    //访问Cache line
    wire                 c_valid[3:0]; // 有效位
    wire                 c_dirty[3:0]; // 脏位
    wire [2:0]                c_lru;   // 伪LRU记录表（树节点）
    wire [TAG_WIDTH-1:0] c_tag  [3:0]; // 标签位
    wire [31:0]          c_block[3:0]; // block数据
    
    genvar i;
    generate
        for(i=0; i<4; i=i+1)begin:gen_data
            assign c_valid[i] = cache_valid[index][i];
            assign c_dirty[i] = cache_dirty[index][i];
            assign c_tag  [i] = cache_tag  [index][i];
            assign c_block[i] = cache_block[index][i];
        end
    endgenerate

    assign c_lru       = cache_lru   [index];      


    // 判断是否命中
    wire hit, miss;
    assign hit = c_valid[0] & (c_tag[0] == tag) | c_valid[1] & (c_tag[1] == tag)
     | c_valid[2] & (c_tag[2] == tag) |c_valid[3] & (c_tag[3] == tag);  // valid==1 & c_tag==tag
    assign miss = ~hit;
    
    // 判断当前访问的是cache的哪一个block，无论命中还是缺失，都要选择一个block处理，选择逻辑写在这里
    wire[1:0] c_way;
    //* 1. hit，选hit的那一路
    //* 2. miss，选不是最近使用的那一路
    // 虽然hit=1的时候理论上这四个条件肯定能中一个，但是为了严谨还是都判断一遍
    assign c_way = hit ? (c_valid[0] & (c_tag[0] == tag) ? 2'b00 :
                          c_valid[1] & (c_tag[1] == tag) ? 2'b01 :
                          c_valid[2] & (c_tag[2] == tag) ? 2'b10 : 
                          c_valid[3] & (c_tag[3] == tag) ? 2'b11 : 2'b00) : 
                   c_lru[0] ? (c_lru[2] ? 2'b11: 2'b10):
                          (c_lru[1] ? 2'b01: 2'b00) ;  // 用伪LRU策略选择替换路(c_lru[i]==0往左走，c_lru[i]==1往右走) 

    // 读或写
    wire read, write;
    assign write = cpu_data_wr;
    assign read = ~write;

    // 当前访问的block是否脏
    wire dirty, clean;
    assign dirty = c_dirty[c_way];
    assign clean = ~dirty;

    // FSM
    parameter IDLE = 2'b00, RM = 2'b01, WM = 2'b11;
    reg [1:0] state;
    reg in_RM; 

    always @(posedge clk) begin
        if(rst) begin
            state <= IDLE;
            in_RM <= 1'b0;
        end
        else begin
            case(state)
                 IDLE: begin
                    state <= IDLE;
                    if (cpu_data_req) begin
                        if (hit) 
                            state <= IDLE;
                        else if (miss & dirty)
                            state <= WM;
                        else if (miss & clean)
                            state <= RM;
                    end
                    in_RM <= 1'b0;
                end

                WM: begin
                    state <= WM;
                    if (cache_data_data_ok)
                        state <= RM;
                end

                RM: begin
                    state <= RM;
                    if (cache_data_data_ok)
                        state <= IDLE;
                    in_RM <= 1'b1;
                end   
            endcase
        end
    end

    // 读内存
    // 变量read_req, addr_rcv, read_finish用于构造类sram信号
    wire read_state;      // 一次完整的读事务，从发出读请求到结束
    reg addr_rcv;         // 地址接收成功(addr_ok)后到结束
    wire read_finish;     // 数据接收成功(data_ok)，即读请求结束
    always @(posedge clk) begin
        addr_rcv <= rst ? 1'b0 :
                    cache_data_req & read_state & cache_data_addr_ok ? 1'b1 :
                    read_finish ? 1'b0 : addr_rcv;
    end
    assign read_state = state==RM;
    assign read_finish = read_state & cache_data_data_ok;

    //写内存
    wire write_state;     
    reg waddr_rcv;      
    wire write_finish;   
    always @(posedge clk) begin
        waddr_rcv <= rst ? 1'b0 :
                     write_state & cache_data_req & cache_data_addr_ok ? 1'b1 :
                     write_finish ? 1'b0 : waddr_rcv;
    end

    assign write_state = state==WM;
    assign write_finish = write_state & cache_data_data_ok;

    //output to mips core
    assign cpu_data_rdata   = hit ? c_block[c_way] : cache_data_rdata;
    assign cpu_data_addr_ok = cpu_data_req & hit | cache_data_req & read_state & cache_data_addr_ok;
    assign cpu_data_data_ok = cpu_data_req & hit | read_state & cache_data_data_ok;

    //output to axi interface
    assign cache_data_req   = read_state & ~addr_rcv | write_state & ~waddr_rcv;
    assign cache_data_wr    = write_state;
    assign cache_data_size  = cpu_data_size;
    assign cache_data_addr  = cache_data_wr ? {c_tag[c_way], index, offset} : cpu_data_addr;
    assign cache_data_wdata = c_block[c_way];


    //写入Cache
    //保存地址中的tag, index，防止addr发生改变
    reg [TAG_WIDTH-1:0] tag_save;
    reg [INDEX_WIDTH-1:0] index_save;
    always @(posedge clk) begin
        tag_save   <= rst ? 0 :
                      cpu_data_req ? tag : tag_save;
        index_save <= rst ? 0 :
                      cpu_data_req ? index : index_save;
    end

    wire [31:0] write_cache_data;
    wire [3:0] write_mask;

    //根据地址低两位和size，生成写掩码（针对sb，sh等不是写完整一个字的指令），4位对应1个字（4字节）中每个字的写使能
    assign write_mask = cpu_data_size==2'b00 ?
                            (cpu_data_addr[1] ? (cpu_data_addr[0] ? 4'b1000 : 4'b0100):
                                                (cpu_data_addr[0] ? 4'b0010 : 4'b0001)) :
                            (cpu_data_size==2'b01 ? (cpu_data_addr[1] ? 4'b1100 : 4'b0011) : 4'b1111);

    //掩码的使用：位为1的代表需要更新的。
    //位拓展：{8{1'b1}} -> 8'b11111111
    //new_data = old_data & ~mask | write_data & mask
    assign write_cache_data = cache_block[index][c_way] & ~{{8{write_mask[3]}}, {8{write_mask[2]}}, {8{write_mask[1]}}, {8{write_mask[0]}}} | 
                              cpu_data_wdata & {{8{write_mask[3]}}, {8{write_mask[2]}}, {8{write_mask[1]}}, {8{write_mask[0]}}};
 
    wire IDLE_state = state==IDLE;

    wire [2:0] update_c_lru;
    assign update_c_lru = (c_way == 2'b00)? 3'b001 & c_lru | 3'b110: // 原来是00x，要变为11x # TODO没准有错
                          (c_way == 2'b01)? 3'b001 & c_lru | 3'b100: // 原来是01x，要变为10x
                          (c_way == 2'b10)? 3'b010 & c_lru | 3'b001: // 原来是1x0，要变为0x1
                          (c_way == 2'b11)? 3'b010 & c_lru | 3'b000: 3'b000;// 原来是1x1，要变为0x0
    integer t;
    always @(posedge clk) begin
        if(rst) begin
            for(t=0; t<CACHE_DEEPTH; t=t+1) begin   //刚开始将Cache置为无效
                cache_valid[t][0] <= 0;   
                cache_valid[t][1] <= 0;
                cache_valid[t][2] <= 0;
                cache_valid[t][3] <= 0;
                cache_dirty[t][0] <= 0;
                cache_dirty[t][1] <= 0;
                cache_dirty[t][2] <= 0;
                cache_dirty[t][3] <= 0;
                cache_lru[t] <= 0;
            end
        end
        else begin
            if(read_finish) begin //读缺失，访存结束时
                cache_valid  [index_save][c_way] <= 1'b1;             //将Cache line置为有效
                cache_dirty[index_save][c_way] <= 1'b0;           // 读取内存的数据后设为clean
                cache_tag    [index_save][c_way] <= tag_save;
                cache_block  [index_save][c_way] <= cache_data_rdata;
            end
            else if(write & IDLE_state & (hit | in_RM)) begin   //写缺失 写->读 结束之后在回到IDLE的上跳沿还要把新数据写到cache
                cache_dirty[index][c_way] <= 1'b1; // 改了数据，变dirty
                cache_block[index][c_way] <= write_cache_data;      //写入Cache line，使用index而不是index_save
            end

            if ((read | write) & IDLE_state & (hit | in_RM)) begin
                //* load 或 store指令，hit进入IDLE状态 或 从读内存回到IDLE后，将最近使用情况更新
                cache_lru[index] <= update_c_lru; //
            end   
        end
    end
endmodule