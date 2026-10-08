# Changes from uploaded RTL

The original monolithic RTL was rebuilt into a consistent synthesizable datapath
and separated from simulation. The original MIT license is preserved.

Corrected register-file port directions, x0 semantics, nonblocking state updates,
PC/memory word addressing, ALU decode, B-immediate instruction[7], missing defaults,
implicit signal typo, unnamed adder/mux port errors and missing load write-back.
Replaced inconsistent ALUOp wiring with explicit validated instruction decode.
Added signed shifts/comparisons, immediate arithmetic and JAL.
Added ROM initialization, guarded memories, fault halt behavior, executable test
programs, architectural reference vectors, self-checking testbench, VCD capture,
Yosys and Vivado synthesis scripts and reproducible run commands.

Top module is now `Single_Cycle` with clk/reset and debug outputs; original `top`
and module names are not retained as a drop-in interface. Reset polarity remains
active-high. See README for memory sizes, supported instructions and limitations.
