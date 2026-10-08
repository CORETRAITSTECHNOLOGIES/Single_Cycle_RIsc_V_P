#!/usr/bin/env python3
"""Deterministic test-program encoder and independent architectural reference."""
from pathlib import Path
import random
ROOT=Path(__file__).resolve().parents[1]
MASK=0xffffffff
R={'add':0,'sub':0,'and':7,'or':6,'xor':4,'sll':1,'srl':5,'sra':5,'slt':2,'sltu':3}
I={'addi':0,'andi':7,'ori':6,'xori':4,'slli':1,'srli':5,'srai':5,'slti':2,'sltiu':3}
p=[]
def emit(op,rd=0,a=0,b=0): p.append((op,rd,a,b))
def encode(t):
 op,rd,a,b=t
 if op in R: return ((0x20 if op in ('sub','sra') else 0)<<25)|(b<<20)|(a<<15)|(R[op]<<12)|(rd<<7)|0x33
 if op in I:
  imm=(b&0xfff)|(0x400 if op=='srai' else 0)
  return (imm<<20)|(a<<15)|(I[op]<<12)|(rd<<7)|0x13
 if op=='lw': return ((b&0xfff)<<20)|(a<<15)|(2<<12)|(rd<<7)|3
 if op=='sw':
  v=b&0xfff
  return ((v>>5)<<25)|(rd<<20)|(a<<15)|(2<<12)|((v&31)<<7)|0x23
 if op=='beq':
  v=b&0x1fff
  return ((v>>12)<<31)|(((v>>5)&63)<<25)|(rd<<20)|(a<<15)|(((v>>1)&15)<<8)|(((v>>11)&1)<<7)|0x63
 if op=='jal':
  v=b&0x1fffff
  return ((v>>20)<<31)|(((v>>1)&1023)<<21)|(((v>>11)&1)<<20)|(((v>>12)&255)<<12)|(rd<<7)|0x6f
 raise ValueError(op)
def signed(v): return v if v<0x80000000 else v-(1<<32)
emit('addi',1,0,12);emit('addi',2,0,5);emit('addi',3,0,-8)
for op in R: emit(op,4,1,2)
for op in I: emit(op,5,3,3 if op in ('slli','srli','srai') else -1)
emit('sra',6,3,2);emit('srl',7,3,2);emit('slt',8,3,1);emit('sltu',9,3,1)
emit('sw',1,0,0);emit('lw',10,0,0)
emit('sw',3,0,252);emit('lw',11,0,252)
emit('addi',12,0,256);emit('sw',2,12,-8);emit('lw',13,12,-8)
emit('addi',0,0,99)
emit('beq',2,1,8) # unequal: fall through
emit('addi',14,0,1)
emit('beq',1,1,8);emit('addi',14,0,99) # equal: skip
emit('addi',15,0,3)
loop=len(p);emit('addi',15,15,-1);emit('beq',0,15,8)
emit('jal',0,0,(loop-len(p))*4)
emit('jal',16,0,8);emit('addi',14,0,99)
rng=random.Random(2026)
for _ in range(120):
 op=rng.choice(list(R)+list(I));rd=rng.randrange(0,32);a=rng.randrange(32)
 b=rng.randrange(32) if op in R or op in ('slli','srli','srai') else rng.randrange(-2048,2048)
 emit(op,rd,a,b)
end=len(p)*4;emit('jal',0,0,0)
regs=[0]*32;mem=[0]*64;pc=0;trace=[];coverage=set()
for step in range(1000):
 if pc==end: break
 op,rd,a,b=p[pc//4];coverage.add(op);x=regs[a];y=regs[b] if op in R else b&MASK
 npc=pc+4;val=0;rw=int(op in R or op in I or op in ('lw','jal'));mw=int(op=='sw');addr=0;sd=0
 base=op[:-1] if op in ('addi','andi','ori','xori','slti') else op
 if base=='add':val=x+y
 elif op=='sub':val=x-y
 elif base=='and':val=x&y
 elif base=='or':val=x|y
 elif base=='xor':val=x^y
 elif op in ('sll','slli'):val=x<<(y&31)
 elif op in ('srl','srli'):val=x>>(y&31)
 elif op in ('sra','srai'):val=signed(x)>>(y&31)
 elif op in ('slt','slti'):val=int(signed(x)<signed(y))
 elif op in ('sltu','sltiu'):val=int(x<y)
 elif op=='lw':val=mem[((x+b)&MASK)//4]
 elif op=='sw':addr=(x+b)&MASK;sd=regs[rd];mem[addr//4]=sd
 elif op=='beq':npc=pc+b if regs[rd]==x else npc
 elif op=='jal':val=pc+4;npc=pc+b
 val&=MASK
 trace.append((pc,npc,rd,rw,val,mw,addr,sd))
 if rw and rd:regs[rd]=val
 pc=npc
else:raise RuntimeError('Reference timeout')
(ROOT/'sim/programs/demo.hex').write_text('\n'.join(f'{encode(t):08x}' for t in p)+ '\n'+'00000013\n'*(256-len(p)))
(ROOT/'sim/programs/demo.asm').write_text('# Test listing: tuples are op, rd (store/branch rs2), rs1, rs2/immediate.\n'+'\n'.join(f'{i*4:04x}: {t}' for i,t in enumerate(p))+'\n')
(ROOT/'sim/programs/expected.hex').write_text('\n'.join(''.join(f'{v:08x}' for v in t) for t in trace)+'\n')
(ROOT/'sim/programs/registers.hex').write_text('\n'.join(f'{v:08x}' for v in regs)+'\n')
(ROOT/'sim/programs/memory.hex').write_text('\n'.join(f'{v:08x}' for v in mem)+'\n')
(ROOT/'sim/programs/count.vh').write_text(f'`define TRACE_COUNT {len(trace)}\n`define END_PC 32\'d{end}\n')
print(f'Generated {len(p)} instructions, {len(trace)} executed checks, {len(coverage)} instruction types.')
imm_tests=[]
for off in range(-4096,4096,2):imm_tests.append((encode(('beq',1,2,off)),off&MASK))
for off in (-1048576,-524288,-2048,-2,0,2,2048,524288,1048574):imm_tests.append((encode(('jal',1,0,off)),off&MASK))
for off in (-2048,-1,0,1,2047):
 for op in ('addi','lw','sw'):imm_tests.append((encode((op,1,2,off)),off&MASK))
(ROOT/'sim/programs/immediates.hex').write_text('\n'.join(f'{ins:08x}{imm:08x}' for ins,imm in imm_tests)+'\n')
with (ROOT/'sim/programs/count.vh').open('a') as f:f.write(f'`define IMM_COUNT {len(imm_tests)}\n')
print(f'Generated {len(imm_tests)} immediate extraction checks.')
