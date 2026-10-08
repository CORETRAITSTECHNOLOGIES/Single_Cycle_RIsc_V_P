.PHONY: sim synth lint clean
sim:
	bash tools/sim.sh
synth:
	bash tools/synth.sh
lint:
	verilator --lint-only -Wall -Wno-fatal --top-module Single_Cycle rtl/Single_Cycle.v
clean:
	rm -f results/core.vvp results/core.vcd results/core.json
