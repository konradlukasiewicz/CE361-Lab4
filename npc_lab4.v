// Northwestern - CompEng 361 - Lab4
// Groupname: npc
// NetIDs: gmv2910, lpi1150

// Some useful defines...please add your own
`define WORD_WIDTH 32
`define NUM_REGS 32
`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_STORE      7'b0100011 
`define OPCODE_JUMP       7'b1101111
`define OPCODE_JALR       7'b1100111
`define OPCODE_IMM        7'b0010011
`define OPCODE_UPPER      7'b0110111
`define OPCODE_AUIPC      7'b0010111
`define FUNC_ADD      3'b000
`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000
`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10

`define OPCODE_BUBBLE 7'b0

module PipelinedCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   reg[`WORD_WIDTH-1:0] InstWord;

   wire [`WORD_WIDTH-1:0] PC, nxtInstWord;
   wire [`WORD_WIDTH-1:0] StoreData, DataWord;
   reg [1:0]  MemSize;
   reg        MemWrEn;
   
   wire [`WORD_WIDTH-1:0] Rdata1, Rdata2;
   wire [`WORD_WIDTH-1:0] RegRdata1, RegRdata2;
   wire [`WORD_WIDTH-1:0] fastRdata1, fastRdata2;

   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4;

   reg [4:0] Rsrc1, Rsrc2; 
   reg [6:0] opcode;
   reg [4:0] EXRdst;
   reg [2:0] funct3;
   reg [6:0] funct7;
   reg [`WORD_WIDTH-1:0] EXPC;

   reg [11:0] I_imm;
   reg [11:0] S_imm;
   reg [11:0] B_imm12_1;
   reg [`WORD_WIDTH-1:0] B_offset;
   reg [19:0] J_imm20_1;
   reg [`WORD_WIDTH-1:0] J_offset;
   reg [`WORD_WIDTH-1:0] st_offset;
   reg [19:0] U_imm;

   wire [31:0] r_out, ir_out, load_out, lui_out, auipc_out;
   wire [31:0] link_jal, link_jalr, npc_jal, npc_jalr, npc_branch;

   wire [`WORD_WIDTH-1:0] ld_addr;
   wire [1:0] ld_size;
   wire ld_wren;  
   wire [`WORD_WIDTH-1:0] st_addr, st_datain;
   wire [1:0] st_size;
   wire st_wren;  

   reg [`WORD_WIDTH-1:0] MEMRWrdata, MEMDataAddr, MEMStoreData;
   reg MEMRWrEn;
   reg [6:0] MEMopcode;
   reg [4:0] MEMRdst;
   reg [2:0] MEMfunct3;

   reg [`WORD_WIDTH-1:0] RWrdata;
   reg RWrEn;
   reg [4:0] Rdst;

   wire known_opcode =
      (MEMopcode == `OPCODE_COMPUTE) ||
      (MEMopcode == `OPCODE_IMM) ||
      (MEMopcode == `OPCODE_LOAD) ||
      (MEMopcode == `OPCODE_STORE) ||
      (MEMopcode == `OPCODE_BRANCH) ||
      (MEMopcode == `OPCODE_JUMP) ||
      (MEMopcode == `OPCODE_JALR) ||
      (MEMopcode == `OPCODE_UPPER) ||
      (MEMopcode == `OPCODE_AUIPC);

   wire pc_misalign = |PC[1:0];
   wire ld_misalign = (MEMopcode == `OPCODE_LOAD) &&
                        ((MemSize == `SIZE_WORD && | MEMDataAddr[1:0]) ||
                        (MemSize == `SIZE_HWORD && MEMDataAddr[0]));
   wire st_misalign = (MEMopcode == `OPCODE_STORE) &&
                        ((MemSize == `SIZE_WORD && |MEMDataAddr[1:0]) ||
                        (MemSize == `SIZE_HWORD && MEMDataAddr[0]));
   wire BranchTaken;
   wire stall;
   wire dependancy;
   wire fastforward;
   wire [`WORD_WIDTH-1:0] fastforwardval;

   assign PC_Plus_4 = PC + 32'd4;
   wire MEMHalt;
   assign MEMHalt = (rst ? (~known_opcode | pc_misalign | ld_misalign | st_misalign) : 1'b0);

   //Handle hault
   
   reg finalHalt;
   always @(posedge clk ) begin
      finalHalt <= (MEMHalt | finalHalt);
   end

   assign halt = finalHalt;
   

   reg [4:0] stallCounter;
   wire continueComplexStall;

   // System State 
   Mem   MEM(.InstAddr(PC), .InstOut(nxtInstWord), 
            .DataAddr(MEMDataAddr), .DataSize(MemSize), .DataIn(MEMStoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(RegRdata1), 
	      .AddrB(Rsrc2), .DataOutB(RegRdata2), 
	      .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   wire [`WORD_WIDTH-1:0] PC_next = (stall || continueComplexStall) ? PC : NPC;

   Reg PC_REG(.Din(PC_next), .Qout(PC), .WE(1'b1), .CLK(clk), .RST(rst));


   initial begin
      MEMRWrdata  = 32'd0;
      MEMRWrEn    = 1'b0;
      MEMDataAddr = 32'd0;
      MemSize     = 2'b0;
      MemWrEn     = 1'b0;
      MEMStoreData= 32'd0;
      MEMopcode   = `OPCODE_IMM;
      MEMRdst     = 5'd0;
      MEMfunct3   = 3'd0;

      RWrdata = 32'd0;
      RWrEn   = 1'b0;
      Rdst    = 5'd0;

      stallCounter = 5'd0;
      finalHalt    = 1'b0;
      InstWord     = 32'h00000013;
      DECPC        = 32'd0;

      opcode = `OPCODE_IMM;
      EXRdst = 5'd0;
      funct3 = 3'd0;
      funct7 = 7'd0;
      Rsrc1  = 5'd0;
      Rsrc2  = 5'd0;
      EXPC   = 32'd0;

      I_imm      = 12'd0;
      S_imm      = 12'd0;
      B_imm12_1  = 12'd0;
      B_offset   = {`WORD_WIDTH{1'b0}};
      J_imm20_1  = 20'd0;
      J_offset   = {`WORD_WIDTH{1'b0}};
      st_offset  = {`WORD_WIDTH{1'b0}};
      U_imm      = 20'd0;

      RWrdata = {`WORD_WIDTH{1'b0}};
      RWrEn   = 1'b0;
      Rdst    = 5'd0;

   end

   //Instruction fetch
   reg [`WORD_WIDTH-1:0] DECPC;
   always @(posedge clk ) begin
      if (stall || continueComplexStall) begin
         InstWord <= InstWord;
         DECPC <= DECPC;
      end
      else if (BranchTaken) begin 
         InstWord <= 32'h00000013;
         DECPC <= 32'd0;
      end
      else begin
         InstWord <= nxtInstWord;
         DECPC <= PC;
      end
   end

   // Main Instruction Decode
  
  
   always @(posedge clk ) begin
      if (stall || continueComplexStall) begin
         // hold current EX-stage instruction
         opcode <= opcode;
         EXRdst <= EXRdst;
         funct3 <= funct3;
         funct7 <= funct7;
         Rsrc1  <= Rsrc1;
         Rsrc2  <= Rsrc2;
         EXPC   <= EXPC;
      end else if (BranchTaken) begin
         opcode <= `OPCODE_IMM;
         EXRdst <= 5'd0;
         funct3 <= 3'd0;
         funct7 <= 7'd0;
         Rsrc1  <= 5'd0;
         Rsrc2  <= 5'd0;
         EXPC   <= 32'd0;
    end else begin
        opcode <=  InstWord[6:0];     // ALL
        EXRdst   <= InstWord[11:7];    // R, I, U, J
        funct3 <= InstWord[14:12];   // R, I, S
        funct7 <= InstWord[31:25];    // R
        Rsrc1 <= InstWord[19:15];  
        Rsrc2 <= InstWord[24:20]; 
        EXPC <= DECPC;
    end
end

   //Handle Decoding Immediates

   wire [11:0] nxt_I_imm = InstWord[31:20];
   wire [11:0] nxt_S_imm = {InstWord[31:25], InstWord[11:7]};
   wire [11:0] nxt_B_imm12_1 = {InstWord[31], InstWord[7], InstWord[30:25], InstWord[11:8]};
   wire [`WORD_WIDTH-1:0] nxt_B_offset = {{19{nxt_B_imm12_1[11]}}, nxt_B_imm12_1, 1'b0};
   wire [19:0] nxt_J_imm20_1  = {InstWord[31], InstWord[19:12], InstWord[20], InstWord[30:21]};
   wire [`WORD_WIDTH-1:0] nxt_J_offset = {{11{nxt_J_imm20_1[19]}}, nxt_J_imm20_1, 1'b0};
   wire [`WORD_WIDTH-1:0] nxt_st_offset = {{20{nxt_S_imm[11]}}, nxt_S_imm};
   wire [19:0] nxt_U_imm = InstWord[31:12];


   always @(posedge clk ) begin
   if (stall || continueComplexStall) begin
        I_imm      <= I_imm;
        S_imm      <= S_imm;
        B_imm12_1  <= B_imm12_1;
        B_offset   <= B_offset;
        J_imm20_1  <= J_imm20_1;
        J_offset   <= J_offset;
        st_offset  <= st_offset;
        U_imm      <= U_imm;
    end else if (BranchTaken) begin
        I_imm      <= 12'd0;
        S_imm      <= 12'd0;
        B_imm12_1  <= 12'd0;
        B_offset   <= {`WORD_WIDTH{1'b0}};
        J_imm20_1  <= 20'd0;
        J_offset   <= {`WORD_WIDTH{1'b0}};
        st_offset  <= {`WORD_WIDTH{1'b0}};
        U_imm      <= 20'd0;
    end else begin
        I_imm <= nxt_I_imm;
        S_imm <= nxt_S_imm;
        B_imm12_1 <= nxt_B_imm12_1;
        B_offset <= nxt_B_offset;
        J_imm20_1 <= nxt_J_imm20_1;
        J_offset <= nxt_J_offset;
        st_offset <= nxt_st_offset;
        U_imm <= nxt_U_imm;
    end
end


 

   ALU_R EU_R(.out(r_out), .opA(Rdata1), .opB(Rdata2), .func(funct3), .auxFunc(funct7));
   ALU_IR EU_IA(.out(ir_out), .opA(Rdata1), .imm(I_imm), .func(funct3));
   MEM_I EU_IL(.opA(Rdata1), .imm(I_imm), .func(funct3), .dataaddr(ld_addr), .memsize(ld_size), .wren(ld_wren));
   LUI EU_LUI (.out(lui_out), .imm(U_imm));
   AUIPC EU_AUI(.out(auipc_out), .imm(U_imm), .pc(EXPC));
   JUMP EU_J(.out(link_jal), .npc(npc_jal), .pc(EXPC), .imm(J_offset));
   JALR EU_JR(.out(link_jalr), .npc(npc_jalr), .pc(EXPC), .opA(Rdata1), .imm(I_imm));
   BRANCH EU_B(.opA(Rdata1), .opB(Rdata2), .offset(B_offset), .func(funct3), .pc(EXPC), .npc(npc_branch), .currpc(PC));
   MEM_S EU_S(.datain(StoreData), .opA(Rdata1), .opB(Rdata2), .imm(S_imm), .func(funct3), .dataaddr(st_addr), .memsize(st_size), .wren(st_wren));

   assign NPC = 
      (opcode == `OPCODE_BRANCH) ? npc_branch:
      (opcode == `OPCODE_JUMP) ? npc_jal:
      (opcode == `OPCODE_JALR) ? npc_jalr :
      PC_Plus_4;

   assign StoreData = (Rsrc2 == MEMRdst && MEMRWrEn) ? 
                     ((MEMopcode == `OPCODE_LOAD) ? DataWord : MEMRWrdata) :
                  (Rsrc2 == Rdst && RWrEn) ? RWrdata :
                  RegRdata2;

   //Handle Branch Hazards
   assign BranchTaken = (opcode == `OPCODE_BRANCH && npc_branch != EXPC+4) || (opcode == `OPCODE_JUMP && npc_jal != EXPC+4) || 
                        (opcode == `OPCODE_JALR && npc_jalr != EXPC+4);

   //Handle Data Hazards (load use dependencies and RAW)
   assign dependancy = (Rsrc1 == MEMRdst && MEMRWrEn) || (Rsrc2 == MEMRdst && MEMRWrEn) ||
                  (Rsrc1 == Rdst && RWrEn) || (Rsrc2 == Rdst && RWrEn);

   //Handle fast forwarding
   assign fastforward = (Rsrc1 == MEMRdst  && (MEMopcode != `OPCODE_LOAD) && MEMRWrEn) || 
                        (Rsrc2 == MEMRdst && (MEMopcode != `OPCODE_LOAD) && MEMRWrEn) ||
                        (Rsrc1 == Rdst && RWrEn) || (Rsrc2 == Rdst && RWrEn);
   assign stall = (dependancy && !fastforward) || 
               ((opcode == `OPCODE_STORE) && (Rsrc2 == MEMRdst) && (MEMopcode == `OPCODE_LOAD));

   assign fastRdata1 = (Rsrc1 == MEMRdst  && (MEMopcode != `OPCODE_LOAD) && MEMRWrEn) ? MEMRWrdata :
                        (Rsrc1 == Rdst && RWrEn) ? RWrdata : 32'b0; 
   assign fastRdata2 =  (Rsrc2 == MEMRdst  && (MEMopcode != `OPCODE_LOAD) && MEMRWrEn) ? MEMRWrdata :
                        (Rsrc2 == Rdst && RWrEn) ? RWrdata : 32'b0;
   assign Rdata1 = ((Rsrc1 == MEMRdst  && (MEMopcode != `OPCODE_LOAD) && MEMRWrEn) || (Rsrc1 == Rdst && RWrEn)) ? fastRdata1 :
                     RegRdata1;
   assign Rdata2 =  ((Rsrc2 == MEMRdst  && (MEMopcode != `OPCODE_LOAD) && MEMRWrEn) || (Rsrc2 == Rdst && RWrEn)) ? fastRdata2 :
                     RegRdata2;

   //Handle Stalling for Multiply/Divide instructions
 
   /*
   (auxFunc == 7'b0000001) ? (
      (func == 3'b000) ? mul :
      (func == 3'b001) ? mulh :
      (func == 3'b010) ? mulhsu :
      (func == 3'b011) ? mulhu :
      (func == 3'b100) ? div :
      (func == 3'b101) ? divu :
      (func == 3'b110) ? rem :
      (func == 3'b111) ? remu :
      32'b0):
      32'b0;
   */
   assign continueComplexStall =  (opcode == `OPCODE_COMPUTE) &&
      (funct7 == 7'b0000001) && (
         ((funct3 == 3'b000 || funct3 == 3'b001 || funct3 == 3'b010 || funct3 == 3'b011) && stallCounter < 4) || 
         ((funct3 == 3'b100 || funct3 == 3'b101 || funct3 == 3'b110 || funct3 == 3'b111) && stallCounter < 20)
      );

   always @(posedge clk ) begin 
      if (stall) begin
         stallCounter <= stallCounter;
      end
      else if (BranchTaken) begin
         stallCounter <= 5'd0;
      end
      else if (continueComplexStall) begin
         stallCounter <= stallCounter + 1;
      end
      else begin 
         stallCounter <= 5'd0;
      end
   end

   //Define MEM Stage Registers
   
   always @(posedge clk) begin
      MEMRWrdata <=  (opcode == `OPCODE_COMPUTE) ? r_out :
                     (opcode == `OPCODE_IMM) ? ir_out :
                     (opcode == `OPCODE_JUMP) ? link_jal :
                     (opcode == `OPCODE_JALR) ? link_jalr :
                     (opcode == `OPCODE_UPPER) ? lui_out :
                     (opcode == `OPCODE_AUIPC) ? auipc_out :
                     32'b0;

      MEMRWrEn <= (opcode == `OPCODE_COMPUTE) ||
                  (opcode == `OPCODE_IMM) ||
                  (opcode == `OPCODE_LOAD) ||
                  (opcode == `OPCODE_JUMP) ||
                  (opcode == `OPCODE_JALR) ||
                  (opcode == `OPCODE_UPPER) ||
                  (opcode == `OPCODE_AUIPC);

      MEMDataAddr <= (opcode == `OPCODE_LOAD) ? ld_addr : 
                           (opcode == `OPCODE_STORE) ? st_addr :
                           32'b0;
      MemSize <= (opcode == `OPCODE_LOAD) ? ld_size : 
                        (opcode == `OPCODE_STORE) ? st_size :
                        2'b0;
      MemWrEn <= (opcode == `OPCODE_LOAD) ? ld_wren : 
                        (opcode == `OPCODE_STORE) ? st_wren :
                        1'b0;
      MEMStoreData <= StoreData;
      MEMopcode <= opcode;
      MEMRdst <= EXRdst;
      MEMfunct3 <= funct3;
   end

   //Handle Final Register Write Back Stage

   wire [31:0] load_ext;
   assign load_ext =
   (MEMfunct3 == 3'b000) ? {{24{DataWord[7]}},  DataWord[7:0]}  :
   (MEMfunct3 == 3'b001) ? {{16{DataWord[15]}}, DataWord[15:0]} :
   (MEMfunct3 == 3'b010) ? DataWord :
   (MEMfunct3 == 3'b100) ? {24'b0, DataWord[7:0]} :
   (MEMfunct3 == 3'b101) ? {16'b0, DataWord[15:0]} :
                              DataWord; 


   always @(posedge clk) begin
      RWrdata <= (MEMopcode == `OPCODE_LOAD) ? load_ext : MEMRWrdata; 
      RWrEn <= MEMRWrEn;
      Rdst <= MEMRdst;
   end
  
    
endmodule // PipelinedCPU


// R-Type 
module ALU_R(out, opA, opB, func, auxFunc);
   output [`WORD_WIDTH-1:0] out;
   input [`WORD_WIDTH-1:0]  opA, opB;
   input [2:0] 	 func;
   input [6:0] 	 auxFunc;

   wire signed [31:0] s_opA = opA, s_opB = opB;

   wire [`WORD_WIDTH-1:0] add = opA + opB;
   wire [`WORD_WIDTH-1:0] sub = opA - opB;
   wire [`WORD_WIDTH-1:0] sll = opA << opB[4:0];
   wire [`WORD_WIDTH-1:0] slt = (s_opA < s_opB) ? 32'd1 : 32'd0;
   wire [`WORD_WIDTH-1:0] sltu = (opA < opB) ? 32'd1 : 32'd0;
   wire [`WORD_WIDTH-1:0] xor1 = opA ^ opB;
   wire [`WORD_WIDTH-1:0] srl = opA >> opB[4:0];
   wire [`WORD_WIDTH-1:0] sra = s_opA >>> opB[4:0];
   wire [`WORD_WIDTH-1:0] or1 = opA | opB;
   wire [`WORD_WIDTH-1:0] and1 = opA & opB;

   //Multiply extension
   wire signed [2*`WORD_WIDTH-1:0] mulh_full   = $signed(s_opA) * $signed(s_opB);
   wire signed [2*`WORD_WIDTH-1:0] mulhsu_full = $signed(s_opA) * $signed({1'b0, opB});
   wire [2*`WORD_WIDTH-1:0] mulhu_full  = $unsigned(opA) * $unsigned(opB);

   wire [`WORD_WIDTH-1:0] mul = s_opA * s_opB;
   wire [`WORD_WIDTH-1:0] mulh   = mulh_full[2*`WORD_WIDTH-1:`WORD_WIDTH];
   wire [`WORD_WIDTH-1:0] mulhsu = mulhsu_full[2*`WORD_WIDTH-1:`WORD_WIDTH];
   wire [`WORD_WIDTH-1:0] mulhu  = mulhu_full[2*`WORD_WIDTH-1:`WORD_WIDTH];

   wire div_zero = (opB == 32'b0); // dividing by 0
   wire overflow = (s_opA == 32'h80000000) && (s_opB == 32'hFFFFFFFF); //-2^31 / -1
   wire [`WORD_WIDTH-1:0] signed_div = $signed(s_opA) / $signed(s_opB);
   wire [`WORD_WIDTH-1:0] signed_rem = $signed(s_opA) % $signed(s_opB);

   wire [`WORD_WIDTH-1:0] div =  div_zero ? 32'hFFFFFFFF : 
                                 overflow ? s_opA :
                                 signed_div;
   wire [`WORD_WIDTH-1:0] divu = div_zero ? 32'hFFFFFFFF : ($unsigned(opA) / $unsigned(opB));
   wire [`WORD_WIDTH-1:0] rem =  div_zero ? s_opA : 
                                 overflow ? 32'b0 :
                                 signed_rem;
   wire [`WORD_WIDTH-1:0] remu = div_zero ? opA : ($unsigned(opA) % $unsigned(opB));


   assign out = 
      (auxFunc == 7'd0) ? (
      (func == 3'b000) ? add :
      (func == 3'b001) ? sll :
      (func == 3'b010) ? slt :
      (func == 3'b011) ? sltu :
      (func == 3'b100) ? xor1 :
      (func == 3'b101) ? srl :
      (func == 3'b110) ? or1 :
      (func == 3'b111) ? and1 :
      32'b0
      ) : (auxFunc == 7'b0100000) ? (
      (func == 3'b000) ? sub :
      (func == 3'b101) ? sra :
      32'b0
      ) : (auxFunc == 7'b0000001) ? (
      (func == 3'b000) ? mul :
      (func == 3'b001) ? mulh :
      (func == 3'b010) ? mulhsu :
      (func == 3'b011) ? mulhu :
      (func == 3'b100) ? div :
      (func == 3'b101) ? divu :
      (func == 3'b110) ? rem :
      (func == 3'b111) ? remu :
      32'b0):
      32'b0;
   
endmodule // ExecutionUnit

// I-Type - arithmetic
module ALU_IR(out, opA, imm, func);
   output [`WORD_WIDTH-1:0] out;
   input [`WORD_WIDTH-1:0]  opA;
   input [11:0]     imm;
   input [2:0] 	 func;

   wire [31:0] opB = {{20{imm[11]}}, imm};
   wire signed [31:0] s_opA = opA, s_opB = opB;

   wire [`WORD_WIDTH-1:0] addi = opA + opB;
   wire [`WORD_WIDTH-1:0] slli = opA << opB[4:0];
   wire [`WORD_WIDTH-1:0] slti = (s_opA < s_opB) ? 32'd1 : 32'd0;
   wire [`WORD_WIDTH-1:0] sltiu = (opA < opB) ? 32'd1 : 32'd0;
   wire [`WORD_WIDTH-1:0] xori = opA ^ opB;
   wire [`WORD_WIDTH-1:0] srli = opA >> opB[4:0];
   wire [`WORD_WIDTH-1:0] srai = s_opA >>> s_opB[4:0];
   wire [`WORD_WIDTH-1:0] ori = opA | opB;
   wire [`WORD_WIDTH-1:0] andi = opA & opB;

   assign out = 
      (func == 3'b000) ? addi :
      (func == 3'b001) ? slli :
      (func == 3'b010) ? slti :
      (func == 3'b011) ? sltiu :
      (func == 3'b100) ? xori :
      (func == 3'b101) ? (imm[11:5] == 7'd0 ?  srli : srai):
      (func == 3'b110) ? ori :
      (func == 3'b111) ? andi :
      32'b0;
   
endmodule // ExecutionUnit



// I-Type - memory loads
module MEM_I(opA, imm, func, dataaddr, memsize, wren);
   input [`WORD_WIDTH-1:0]  opA;
   input [11:0]     imm;
   input [2:0] 	 func;
   output [`WORD_WIDTH-1:0] dataaddr;
   output [1:0] memsize;
   output wren;

   wire [`WORD_WIDTH-1:0] byte_sext, halfword_sext;
   wire [`WORD_WIDTH-1:0] byte_zext, halfword_zext;
   wire [`WORD_WIDTH-1:0] word;


   wire [31:0] opB = {{20{imm[11]}}, imm};
   assign dataaddr = opA + opB;
   assign wren = 0;
   assign memsize =  (func == 3'b000 || func == 3'b100)  ? 2'b00:
      (func == 3'b001 || func == 3'b101) ? 2'b01:
      2'b10;
 
endmodule

// Store
module MEM_S(datain, opA, opB, imm, func, dataaddr, memsize, wren);
   input [`WORD_WIDTH-1:0]  opA, opB;
   output [`WORD_WIDTH-1:0]  datain, dataaddr;
   input [11:0] imm;
   input [2:0] 	 func;  
   output [1:0] memsize;
   output wren;

   wire [31:0] imm_sext = {{20{imm[11]}}, imm};
   assign datain = opB;
   assign dataaddr = opA + imm_sext;
   assign memsize =  
   (func == 3'b000) ? 2'b00 :
   (func == 3'b001) ? 2'b01 :
   (func == 3'b010) ? 2'b10 : 2'b11; 
   assign wren = 1;
endmodule




module BRANCH(opA, opB, offset, func, pc, npc, currpc);
   input [`WORD_WIDTH-1:0]  opA, opB, pc, currpc;
   input signed [`WORD_WIDTH-1:0] offset;
   output [`WORD_WIDTH-1:0] npc;
   wire signed  [`WORD_WIDTH-1:0] s_opA = opA, s_opB = opB;
   wire [`WORD_WIDTH-1:0] beq_pc, bne_pc, blt_pc, bge_pc, bltu_pc, bgeu_pc; 
   input [2:0] func;

   assign beq_pc = (s_opA == s_opB) ? pc + offset : currpc + 4;
   assign bne_pc = (s_opA != s_opB) ? pc + offset : currpc + 4;
   assign blt_pc = (s_opA < s_opB) ? pc + offset : currpc + 4;
   assign bge_pc = (s_opA >= s_opB) ? pc + offset : currpc + 4;
   assign bltu_pc = (opA < opB) ? pc + offset : currpc + 4;
   assign bgeu_pc = (opA >= opB) ? pc + offset : currpc + 4;

   assign npc = 
      (func == 3'b000) ? beq_pc :
      (func == 3'b001) ? bne_pc :
      (func == 3'b100) ? blt_pc :
      (func == 3'b101) ? bge_pc :
      (func == 3'b110) ? bltu_pc :
      (func == 3'b111) ? bgeu_pc:
      32'b0;
endmodule

// J-Type (jal)
module JUMP(out, npc, pc, imm);
   output [`WORD_WIDTH-1:0] out, npc;
   input [`WORD_WIDTH-1:0] pc, imm;
   assign out = pc + 32'd4;
   assign npc = pc + imm;
endmodule

// J-Type (jalr)
module JALR(out, npc, pc, opA, imm);
   output [`WORD_WIDTH-1:0] out, npc;
   input [`WORD_WIDTH-1:0] pc, opA;
   input [11:0] imm;
   wire [31:0] imm_ex = {{20{imm[11]}}, imm};
   assign out = pc + 32'd4;
   assign npc = (opA + imm_ex) & ~32'd1;
endmodule

// U-type - lui
module LUI(out, imm);
   output [`WORD_WIDTH-1:0] out;
   input [19:0] imm;
   assign out = {imm, 12'b0};
  endmodule

// U-type - auipc
module AUIPC(out, imm, pc);
   output [`WORD_WIDTH-1:0] out;
   input [19:0] imm;
   input [`WORD_WIDTH-1:0] pc;
   assign out = pc + {imm, 12'b0};
endmodule