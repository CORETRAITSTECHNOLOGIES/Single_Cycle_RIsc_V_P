// program counter
module Program_Counter(clk,reset,Pc_in, Pc_out);
input clk,reset;
input [31:0] Pc_in;
output reg[31:0] Pc_out;

always @(posedge clk or posedge reset)
begin
  if(reset)
  Pc_out <= 32'b00;
  else
  Pc_out <= Pc_in;
  end
endmodule


// pc+4bitadder

module Adder_4bit(fromPC,nextoPC);
input [31:0] fromPC;
output [31:0] nextoPC;
assign  nextoPC = fromPC + 4;

endmodule

// Instruction Memory

module Instruction_Memory(clk, reset,read_address,instruction);
input clk,reset;
input [31:0] read_address;
output reg [31:0] instruction;
integer k;
reg [31:0] Mem[63:0];
assign instruction = Mem[read_address];
always @(posedge clk or posedge reset)
begin
    if(reset) 
            begin
                   for(k =0 ;k<64;k=k+1) begin
                   Mem[k]  = 32'b00;
                   end
            end
    else
         instruction = Mem[read_address];   
end

      

endmodule

module register_file(clk, reset,RegWirte,rs1,rs2,rd,write_data,read_data1,read_data2);
input clk,reset,RegWirte;
output [4:0] rs1,rs2,rd;
input [31:0] write_data;
output [31:0] read_data1 ,read_data2;
integer i;
reg[31:0] Registers[31:0];
always @(posedge clk or posedge reset)
begin 
  if(reset)
  begin
     for(i=0;i<32;i=i+1)
     begin
        Registers[i] = 32'b0;
     end 
  end
  else if (RegWirte)
  begin
     Registers[rd] = write_data;
  end
end
assign read_data1 = Registers[rs1];
assign read_data2 = Registers[rs2];
endmodule

// Immediate generator

module Immediate_Generator(Opcode, instruction, ImmExt);
 input [6:0] Opcode;
 input [31:0] instruction;
 output reg[31:0] ImmExt;

 always@(*)
 begin
    case(Opcode)
    7'b0000011 : ImmExt <= {{20{instruction[31]}},instruction[31:20]};
    7'b0100011 : ImmExt <= {{20{instruction[31]}},instruction[31:25],instruction[11:7]};
    7'b1100011 : ImmExt <= {{20{instruction[31]}},instruction[31],instruction[31:25],instruction[11:8],1'b0};
    endcase
 end

 
endmodule

module Control_unit(instruction,branch,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite);


input [6:0] instruction;
output reg branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;
output reg [1:0] ALUOp;
always@(*)
begin 
     case(instruction)
     7'b0110011 :{ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,branch,ALUOp} <= 8'b001000_01;
     7'b0000011 :{ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,branch,ALUOp} <= 8'b111100_00;
     7'b0100011 :{ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,branch,ALUOp} <= 8'b100010_00;
     7'b1100011 :{ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,branch,ALUOp} <= 8'b000001_01;
     endcase
end

endmodule

module AlU_unit(A,B, Control_in,ALU_Result,zero);
input [31:0]A,B;
input [3:0]Control_in;
output reg zero;
output reg[31:0] ALU_Result;
always @(Control_in or A or B)
begin
       case(Control_in)
        4'b0000: begin zero <=0; ALU_Result <= A & B;end
        4'b0001: begin zero <=0; ALU_Result <= A | B;end
        4'b0010: begin zero <=0; ALU_Result <= A + B;end
        4'b0110: begin if(A == B) zero <= 1; else zero <= 0; ALU_Result <= A - B; end
        endcase
end
endmodule
// ALU Control
module ALU_Control(ALUOP,fun7,fun3,Control_out);

input fun7;
input [2:0] fun3;
input [1:0] ALUOP;
output reg [3:0] Control_out;
always@(*)
begin
     case({ALUOP, fun7, fun3})
     6'b00_0_000:Control_out <= 4'b0010;
     6'b00_0_000:Control_out <= 4'b0110;
     6'b10_0_000:Control_out <= 4'b0010;
     6'b10_1_000:Control_out <= 4'b0110;
     6'b10_0_111:Control_out <= 4'b0000;
     6'b10_0_110:Control_out <= 4'b0001;
     endcase
end
endmodule

//data memory

module Data_Memory(clk,reset,MemWrite,MemRead,read_address,Write_data,MemData_out);
input clk,reset,MemWrite, MemRead;
input [31:0] read_address ,Write_data;
output [31:0] MemData_out;
integer K;
reg[31:0] D_Memory[63:0];
always @(posedge clk or posedge reset)
begin
if(reset)
        begin
               for(K=0;K<64;K=K+1)begin
               D_Memory[K] <= 32'b00;
               end
        end
else if(MemWrite) begin
       D_Memory[read_address] <= Write_data;
      end
end
assign MemData_out = (MemRead) ? D_Memory[read_address]: 32'b00;

endmodule

//Multuiplexers Mux1(Sel,A1,B1,Mux1_out);
module Mux1(Sel,A1,B1,Mux1_out);
input Sel;
input [31:0] A1,B1;
output [31:0]Mux1_out;
assign Mux1_out = (Sel == 1'b0)? A1:B1;



endmodule

module Mux2(Sel2,A2,B2,Mux2_out);
input Sel2;
input [31:0] A2,B2;
output [31:0]Mux2_out;
assign Mux2_out = (Sel2 == 1'b0)? A2:B2;



endmodule

module Mux3(Sel3,A3,B3,Mux3_out);
input Sel3;
input [31:0] A3,B3;
output [31:0]Mux3_out;
assign Mux3_out = (Sel3 == 1'b0)? A3:B3;



endmodule

//And Logic

module And_logic(branch,zero,and_out);
input branch,zero;
output and_out;

assign and_out = branch & zero;

endmodule


module Adder(in_1,in_2,Sum_out);
input [31:0] in_1,in_2;
output [31:0] Sum_out;
assign Sum_out = in_1 + in_2;


endmodule 

//all modules are instantiated here
module top (clk ,reset);
input clk,reset;
wire [31:0] PC_top , Instruction_Top ,Rd1_Top,Rd2_Top , ImmExt_Top , mux1_top , Sumout_Top,NextoPC_TOP ,PCIn_Top,address_top,Memdata_top,Writeback_top;
wire RegWire_Top , AlUsrc_Top,Zero_Top,branch_top,Sel2_top ,Memtoreg_top , MemWrite_top,MemRead_top;
wire [1:0] ALU_TOP;
wire [3:0]Control_Top;

//program counter

Program_Counter  PC(.clk(clk),.reset(reset),.Pc_in(PCIn_Top), .Pc_out(PC_top));

Adder_4bit PC_Adder(.fromPC(PC_top),.nextoPC(NextoPC_TOP));

Instruction_Memory Instace_Mem(.clk(clk), .reset(reset),.read_address(PC_top),.instruction(Instruction_Top));

register_file  Reg_File(.clk(clk), .reset(reset),.RegWirte(RegWire_Top),.rs1(Instruction_Top[19:15]),.rs2(Instruction_Top[24:20]),.rd(Instruction_Top[11:7]),.write_data(Writeback_top),.read_data1(Rd1_Top),.read_data2(Rd2_Top));

Immediate_Generator IMM_GEN(.Opcode(Instruction_Top[6:0]), .instruction(Instruction_Top), .ImmExt(ImmExt_Top));

Control_unit Ctrl_unit(.instruction(Instruction_Top[6:0]),.branch(bracnh_top),.MemRead(MemRead_top),.MemtoReg(Memtoreg_top),.ALUOp(ALU_TOP),.MemWrite(MemWrite_top),.ALUSrc(AlUsrc_Top),.RegWrite(RegWire_Top));

ALU_Control  Alu_ctr(.ALUOP(ALU_TOP),.fun7(Instruction_Top[30]),.fun3(Instruction_Top[14:12]),.Control_out(Control_Top));

AlU_unit Alu_unit(.A(Rd1_Top),.B(mux1_top), .Control_in(Control_Top),.ALU_Result(address_top),.zero(Zero_Top));

Mux1 Mux1_one(.Sel(AlUsrc_Top),.A1(Rd2_Top),.B1(ImmExt_Top),.Mux1_out(mux1_top));

//Adder
Adder(.in_1(PC_top),.in_2(ImmExt_Top),.Sum_out(Sumout_Top));
//And

 And_logic And_gate(.branch(bracnh_top),.zero(Zero_Top),.and_out(Sel2_top));

Mux2 Mux2_Two(.Sel2(Sel2_top),.A2(NextoPC_TOP),.B2(Sumout_Top),.Mux1_out(PCIn_Top));
//data_memory

Data_Memory Data_Mem(.clk(clk),.reset(reset),.MemWrite(MemWrite_top),.MemRead(MemRead_top),.read_address(address_top),.Write_data(Rd2_Top),.MemData_out());

//multiplexer

 Mux3 Mux3_three(.Sel3(Memtoreg_top),.A3(address_top),.B3(Memdata_top),.Mux3_out(Writeback_top));

endmodule


// testbench

module tb_top;

reg clk ,reset;

top uut(.clk(clk) ,.reset(reset));
initial begin
clk =0;
reset = 1;
#5;
reset =0;
#400;
end

always begin
#5 clk = ~clk;
end



endmodule