#!/usr/bin/env python3
"""Render actual VCD signal samples. Requires matplotlib (optional)."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
signals={name:None for name in ('PC','ALU_Out','BranchTaken','MemWrite','RegWrite')}
scopes=[]
for line in (root/'results/core.vcd').read_text().splitlines():
 parts=line.split()
 if line.startswith('$scope'):scopes.append(parts[2])
 elif line.startswith('$upscope'):scopes.pop()
 elif line.startswith('$var') and scopes==['tb_single_cycle','dut'] and parts[4] in signals:signals[parts[4]]=parts[3]
 elif line.startswith('$enddefinitions'):break
points={k:[] for k in signals};t=0
for line in (root/'results/core.vcd').read_text().splitlines():
 if line.startswith('#'):t=int(line[1:])/1000
 if t>500:break
 if line.startswith('b'):
  value,code=line[1:].split()
 elif line and line[0] in '01xz':value,code=line[0],line[1:]
 else:continue
 if any(c in value for c in 'xz'):continue
 for name,key in signals.items():
  if code==key:points[name].append((t,int(value,2)))
fig,axes=plt.subplots(5,1,figsize=(12,8),sharex=True)
for ax,(name,data) in zip(axes,points.items()):
 x,y=zip(*data);ax.step(x,y,where='post',linewidth=1.5);ax.set_ylabel(name);ax.grid(alpha=.2)
 if name in ('PC','ALU_Out'):ax.ticklabel_format(axis='y',style='plain',useOffset=False)
axes[-1].set_xlabel('Simulation time (ns)')
fig.suptitle('Single-cycle RV32I subset — samples from executed Icarus VCD\nFirst 500 ns: arithmetic, loads/stores, branches and jumps')
fig.tight_layout();fig.savefig(root/'results/waveform.png',dpi=160)
