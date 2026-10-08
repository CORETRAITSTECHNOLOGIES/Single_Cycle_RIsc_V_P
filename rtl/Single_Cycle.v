`timescale 1ns/1ps
`default_nettype none
// Harvard, single-cycle RV32I subset. Byte addresses, word-aligned accesses.
module Single_Cycle #(parameter IMEM_WORDS=256, DMEM_WORDS=64,
                      parameter PROGRAM_FILE="sim/programs/demo.hex")
(input wire clk, reset, output reg [31:0] PC,
 output wire [31:0] instruction, ALU_Out, writeback,
 output wire RegWrite, MemWrite, BranchTaken, output wire fault);
 wire [31:0] a,b,imm,alu_b,load_data;
 wire [3:0] alu_op;
 wire use_imm,mem_read,mem_to_reg,branch,jump,legal,zero;
 wire [31:0] next_pc;
 wire if_ok,mem_ok;
 wire [31:0] target=PC+imm;
 assign next_pc=(BranchTaken || jump) ? target : PC+32'd4;
 assign fault=!if_ok || !legal || ((mem_read || MemWrite) && !mem_ok)
              || ((BranchTaken || jump) && target[1:0]!=0);
 assign BranchTaken=branch && zero;
 assign alu_b=use_imm ? imm : b;
 assign writeback=jump ? PC+32'd4 : (mem_to_reg ? load_data : ALU_Out);
 always @(posedge clk or posedge reset)
   if(reset) PC<=0;
   else if(!fault) PC<=next_pc;
 Instruction_Memory #(IMEM_WORDS,PROGRAM_FILE) imem(PC,instruction,if_ok);
 Register_File rf(clk,reset,RegWrite && !fault,instruction[19:15],
                 instruction[24:20],instruction[11:7],writeback,a,b);
 Immediate_Generator ig(instruction,imm);
 Control_Unit cu(instruction,RegWrite,use_imm,mem_read,MemWrite,
                 mem_to_reg,branch,jump,alu_op,legal);
 ALU alu(a,alu_b,alu_op,ALU_Out,zero);
 Data_Memory #(DMEM_WORDS) dmem(clk,reset,MemWrite && !fault,
                               mem_read,ALU_Out,b,load_data,mem_ok);
endmodule

module Instruction_Memory #(parameter WORDS=256, parameter PROGRAM_FILE="sim/programs/demo.hex")
(input wire [31:0] address,output wire [31:0] instruction,output wire valid);
 reg [31:0] mem[0:WORDS-1];
 initial $readmemh(PROGRAM_FILE,mem);
 assign valid=(address[1:0]==0 && address[31:2]<WORDS);
 assign instruction=valid ? mem[address[31:2]] : 32'h00000000;
endmodule

module Register_File(input wire clk,reset,we,input wire [4:0] rs1,rs2,rd,
 input wire [31:0] wd,output wire [31:0] a,b);
 reg [31:0] regs[0:31]; integer i;
 always @(posedge clk or posedge reset)
   if(reset) begin
     for(i=0;i<32;i=i+1) regs[i]<=0;
   end else if(we && rd!=0) regs[rd]<=wd;
 assign a=(rs1==0) ? 32'b0 : regs[rs1];
 assign b=(rs2==0) ? 32'b0 : regs[rs2];
endmodule

module Immediate_Generator(input wire [31:0] ins,output reg [31:0] imm);
 always @* begin
   imm=0;
   case(ins[6:0])
    7'h03,7'h13: imm={{20{ins[31]}},ins[31:20]};
    7'h23: imm={{20{ins[31]}},ins[31:25],ins[11:7]};
    7'h63: imm={{19{ins[31]}},ins[31],ins[7],ins[30:25],ins[11:8],1'b0};
    7'h6f: imm={{11{ins[31]}},ins[31],ins[19:12],ins[20],ins[30:21],1'b0};
    default: imm=0;
   endcase
 end
endmodule

// ALU encoding: ADD,SUB,AND,OR,XOR,SLL,SRL,SRA,SLT,SLTU.
module Control_Unit(input wire [31:0] ins,
 output reg rw,src,mr,mw,mtr,branch,jump,output reg [3:0] op,output reg legal);
 always @* begin
  rw=0;src=0;mr=0;mw=0;mtr=0;branch=0;jump=0;op=0;legal=0;
  case(ins[6:0])
   7'h33,7'h13: begin
    rw=1;src=(ins[6:0]==7'h13);legal=1;
    case(ins[14:12])
     0: begin
      op=0;
      if(!src) begin
       if(ins[31:25]==7'h20) op=1;
       else if(ins[31:25]!=0) legal=0;
      end
     end
     7: op=2;
     6: op=3;
     4: op=4;
     1: begin op=5; if(ins[31:25]!=0) legal=0; end
     5: begin
      if(ins[31:25]==0) op=6;
      else if(ins[31:25]==7'h20) op=7;
      else legal=0;
     end
     2: op=8;
     3: op=9;
     default: legal=0;
    endcase
    if(!src && ins[14:12]!=0 && ins[14:12]!=5 && ins[31:25]!=0) legal=0;
   end
   7'h03: begin src=1;rw=1;mr=1;mtr=1;legal=(ins[14:12]==2);end
   7'h23: begin src=1;mw=1;legal=(ins[14:12]==2);end
   7'h63: begin branch=1;op=1;legal=(ins[14:12]==0);end
   7'h6f: begin jump=1;rw=1;legal=1;end
   default: legal=0;
  endcase
 end
endmodule

module ALU(input wire [31:0] a,b,input wire [3:0] op,
 output reg [31:0] result,output wire zero);
 always @* begin
  case(op)
   0:result=a+b; 1:result=a-b; 2:result=a&b; 3:result=a|b;
   4:result=a^b; 5:result=a<<b[4:0]; 6:result=a>>b[4:0];
   7:result=$signed(a)>>>b[4:0];
   8:result={31'b0,($signed(a)<$signed(b))};
   9:result={31'b0,(a<b)};
   default:result=0;
  endcase
 end
 assign zero=(result==0);
endmodule

module Data_Memory #(parameter WORDS=64)
(input wire clk,reset,we,re,input wire [31:0] address,wd,
 output wire [31:0] data,output wire valid);
 reg [31:0] mem[0:WORDS-1]; integer i;
 assign valid=(address[1:0]==0 && address[31:2]<WORDS);
 always @(posedge clk or posedge reset)
  if(reset) begin for(i=0;i<WORDS;i=i+1) mem[i]<=0;end
  else if(we && valid) mem[address[31:2]]<=wd;
 assign data=(re && valid) ? mem[address[31:2]] : 32'b0;
endmodule
`default_nettype wire
