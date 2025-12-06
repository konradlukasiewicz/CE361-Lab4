// Testbench for Northwestern - CompEng 361 - Lab4
`define CHAR_WIDTH 8
`define MAX_CHAR 80

module tb;
    reg clk, rst;
    reg exit;
    wire halt;
    reg [`CHAR_WIDTH*`MAX_CHAR-1:0] mem_in_fname, mem_out_fname, regs_in_fname, regs_out_fname;
    reg [`CHAR_WIDTH*`MAX_CHAR-1:0] signal_dump_fname;

    // Pipelined CPU instantiation
    PipelinedCPU CPU (halt, clk, rst);

    // Clock Period = 10 time units
    always
        #5 clk = ~clk;

    always @(negedge clk)
        if (halt)
            exit = 1;

    initial begin

        // Read commandline options
        if (!$value$plusargs("MEM_IN=%s", mem_in_fname))
            mem_in_fname = "mem_in.hex";
        if (!$value$plusargs("REGS_IN=%s", regs_in_fname))
            regs_in_fname = "regs_in.hex";
        if (!$value$plusargs("REGS_OUT=%s", regs_out_fname))
            regs_out_fname = "regs_out.hex";
        if (!$value$plusargs("MEM_OUT=%s", mem_out_fname))

            mem_out_fname = "mem_out.hex";
         if (!$value$plusargs("DUMP=%s", signal_dump_fname))
            signal_dump_fname = "single.vcd";

        // Clock and reset steup
        #0 rst = 0; exit = 0; clk = 0;
        #0 rst = 1; 
        
        

        // Load program memory and regs
        #0 $readmemh(mem_in_fname, CPU.MEM.Mem);
        #0 $readmemh(regs_in_fname, CPU.RF.Mem);

        // Dumpfile
        $dumpfile(signal_dump_fname);
        $dumpvars();

        // Feel free to modify to inspect whatever you want
        #0 $monitor($time,, "PC=%08x IR=%08x halt=%x exit=%x", CPU.PC, CPU.InstWord, halt, exit);

        // Exit???
        wait(exit);
      
        // Dump memory and regs 
        #0 $writememh(regs_out_fname, CPU.RF.Mem);
        #0 $writememh(mem_out_fname, CPU.MEM.Mem);
        
        $finish;      
   end

endmodule // tb


module Mem(InstAddr, InstOut,
         DataAddr, DataSize, DataIn, DataOut, WE, CLK);
    input [31:0] InstAddr, DataAddr;
    input [1:0] 	DataSize;   
    input [31:0] DataIn;   
    output [31:0] InstOut, DataOut;  
    input      WE, CLK;
    reg [7:0] 	Mem[0:1024];

    wire [31:0] 	DataAddrH, DataAddrW, InstAddrW;

    // Instruction Addresses are word aligned
    assign InstAddrW = InstAddr & 32'hfffffffc;
   
    assign DataAddrH = DataAddr & 32'hfffffffe;
    assign DataAddrW = DataAddr & 32'hfffffffc;

    // Little endian
    assign InstOut = {Mem[InstAddrW+3], Mem[InstAddrW+2], 
		Mem[InstAddrW+1], Mem[InstAddrW]};

    // Little endian
    assign DataOut = (DataSize == 2'b00) ? {4{Mem[DataAddr]}} :
	       ((DataSize == 2'b01) ? {2{Mem[DataAddrH+1],Mem[DataAddrH]}} :
		{Mem[DataAddrW+3], Mem[DataAddrW+2], Mem[DataAddrW+1], Mem[DataAddrW]});
   
     always @ (posedge CLK)
        if (WE) begin
	        case (DataSize)
            2'b00: begin // Write byte
                Mem[DataAddr] <= DataIn[7:0];
            end
            2'b01: begin  // Write halfword
                Mem[DataAddrH] <= DataIn[7:0];
                Mem[DataAddrH+1] <= DataIn[15:8];
            end
            2'b10, 2'b11: begin // Write word
                Mem[DataAddrW] <= DataIn[7:0];
                Mem[DataAddrW+1] <= DataIn[15:8];
                Mem[DataAddrW+2] <= DataIn[23:16];
                Mem[DataAddrW+3] <= DataIn[31:24];
            end
        endcase // case (Size)
     end
endmodule // Mem

module RegFile(AddrA, DataOutA,
	       AddrB, DataOutB,
	       AddrW, DataInW, WenW, CLK);
   input [4:0] AddrA, AddrB, AddrW;
   output [31:0] DataOutA, DataOutB;  
   input [31:0]  DataInW;
   input 	 WenW, CLK;
   reg [31:0] 	 Mem[0:31];
   
   assign DataOutA = (AddrA == 0) ? 32'h00000000 : Mem[AddrA];
   assign DataOutB = (AddrB == 0) ? 32'h00000000 : Mem[AddrB]; 

   always @ (posedge CLK) begin
     if (WenW) begin
       Mem[AddrW] <= DataInW;
     end
      Mem[0] <= 0; // Enforce the invariant that x0 = 0
   end
   
endmodule // RegFile


module Reg(Din, Qout, WE, CLK, RST);
   parameter width = 32;
   parameter init = 0;
   input [width-1:0] Din;
   output [width-1:0] Qout;
   input 	      WE, CLK, RST;

   reg [width-1:0]    Qout;
   
   always @ (posedge CLK or negedge RST)
     if (!RST)
       Qout <= init;
     else
       if (WE)
	     Qout <= Din;
  
endmodule // Reg
