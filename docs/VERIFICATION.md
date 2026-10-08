# Executed verification — 8 October 2026

## Icarus Verilog 12.0

Compiled RTL and testbench with `-g2012 -Wall`. No compiler warnings.
Actual simulation output:

```
PASS: 4120 immediate extraction checks
PASS: 167 reference-trace checks, 32 registers, 64 memory words, JAL loop and 9 fault cases
```

The 165-instruction ROM executes 167 instructions before the terminal JAL loop.
The seeded test covers all 23 implemented instruction types, compares every
executed write-back and store plus PC progression against a Python architectural
reference, and checks the complete final register and data-memory state.
Additional extraction tests cover every even B-immediate (-4096 through +4094),
J-immediate boundaries, and I/S sign-extension boundaries.

Nine fault tests cover illegal opcode, unsupported MUL encoding, misaligned and
out-of-range LW/SW, misaligned taken BEQ/JAL target, and out-of-range fetch.
Faulting instructions are checked for a frozen PC and no x1/memory[0] changes.
These tests do not establish full RISC-V compliance or exhaustive core correctness.

`results/core.vcd` is the actual simulator output; `results/waveform.png` plots
samples from that VCD. `results/simulation.log` contains the executed results.

## Yosys 0.69 (YoWASP build)

Executed `tools/synth.ys` with yowasp-yosys, including generic synthesis,
`check -assert`, statistics and JSON netlist output.

- Both post-synthesis structural checks: **Found and reported 0 problems.**
- Generic mapped design: **14,380 cells**, including the initialized ROM image.
- Two unique warnings: register array and resettable data memory become registers.
- No inferred latches; no multiple-driver or undriven-wire check failures.

This is generic synthesis, not a board-specific resource or timing result.
Cell counts depend on ROM content and tool version. `results/synthesis.log`
contains the full report; `results/core.json` contains the generated netlist.

## Not executed

Vivado synthesis, ModelSim/Questa execution, Verilator lint, FPGA implementation,
physical timing closure and hardware testing. Scripts/commands are provided;
compatibility with those tools must be validated in the target environment.
