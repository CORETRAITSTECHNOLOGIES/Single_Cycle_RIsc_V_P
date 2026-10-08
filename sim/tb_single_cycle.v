`timescale 1ns/1ps
`default_nettype none
`include "sim/programs/count.vh"
module tb_single_cycle;
 reg clk=0,reset=1;
 wire [31:0] pc,ins,alu,wb;
 wire rw,mw,bt,fault;
 reg [255:0] expected[0:`TRACE_COUNT-1];
 reg [31:0] expected_regs[0:31],expected_mem[0:63];
 reg [255:0] row; integer i;
 reg [63:0] imm_vectors[0:`IMM_COUNT-1];
 reg [31:0] probe_ins=0; wire [31:0] probe_imm;
 Immediate_Generator probe(probe_ins,probe_imm);
 Single_Cycle dut(clk,reset,pc,ins,alu,wb,rw,mw,bt,fault);
 always #5 clk=~clk;
 initial begin
  $readmemh("sim/programs/expected.hex",expected);
  $readmemh("sim/programs/immediates.hex",imm_vectors);
  $readmemh("sim/programs/registers.hex",expected_regs);
  $readmemh("sim/programs/memory.hex",expected_mem);
  $dumpfile("results/core.vcd");$dumpvars(0,tb_single_cycle);
  #12;reset=0;
  for(i=0;i<`TRACE_COUNT;i=i+1) begin
   row=expected[i];#1;
   if(fault || pc!==row[255:224]) $fatal(1,"PC/fault mismatch at step %0d PC=%h",i,pc);
   if(rw!==row[128] || mw!==row[64]) $fatal(1,"Control mismatch at step %0d",i);
   if(rw && (ins[11:7]!==row[164:160] || wb!==row[127:96])) $fatal(1,"Writeback mismatch step %0d got %h expected %h",i,wb,row[127:96]);
   if(mw && (alu!==row[63:32] || dut.b!==row[31:0])) $fatal(1,"Store mismatch step %0d",i);
   @(posedge clk);#1;
   if(pc!==row[223:192]) $fatal(1,"Next PC mismatch step %0d",i);
   @(negedge clk);
  end
  if(pc!==`END_PC) $fatal(1,"End PC mismatch");
  for(i=0;i<32;i=i+1) if(dut.rf.regs[i]!==expected_regs[i]) $fatal(1,"Register x%0d mismatch",i);
  for(i=0;i<64;i=i+1) if(dut.dmem.mem[i]!==expected_mem[i]) $fatal(1,"Memory word %0d mismatch",i);
  repeat(3) begin @(posedge clk);#1;if(pc!==`END_PC) $fatal(1,"JAL loop failed");end
  // Inject one instruction while reset is asserted. Check faults have no side effects.
  check_fault(32'hffffffff); // illegal opcode
  check_fault(32'h020000b3); // unsupported MUL encoding
  check_fault(32'h00202083); // misaligned LW
  check_fault(32'h10002083); // out-of-range LW
  check_fault(32'h00002123); // misaligned SW
  check_fault(32'h10002023); // out-of-range SW
  check_fault(32'h0020006f); // misaligned JAL target
  check_fault(32'h00200163); // taken BEQ +2
  reset=1;#1;dut.imem.mem[0]=32'h00000013;#1;reset=0;
  @(negedge clk);dut.PC=32'd1024;#1;
  if(!fault) $fatal(1,"Instruction range guard failed");
  @(posedge clk);#1;if(pc!==1024) $fatal(1,"Fetch fault did not halt");
  for(i=0;i<`IMM_COUNT;i=i+1) begin
   probe_ins=imm_vectors[i][63:32];#1;
   if(probe_imm!==imm_vectors[i][31:0]) $fatal(1,"Immediate mismatch vector %0d",i);
  end
  $display("PASS: %0d immediate extraction checks",`IMM_COUNT);
  $display("PASS: %0d reference-trace checks, 32 registers, 64 memory words, JAL loop and 9 fault cases",`TRACE_COUNT);
  $finish;
 end
 task check_fault(input [31:0] bad);
  begin
   reset=1;#1;dut.imem.mem[0]=bad;#1;reset=0;#1;
   if(!fault) $fatal(1,"Expected fault for %h",bad);
   @(posedge clk);#1;
   if(pc!==0 || dut.rf.regs[1]!==0 || dut.dmem.mem[0]!==0) $fatal(1,"Fault caused side effects");
   @(negedge clk);
  end
 endtask
 initial begin #20000;$fatal(1,"Simulation timeout");end
endmodule
`default_nettype wire
