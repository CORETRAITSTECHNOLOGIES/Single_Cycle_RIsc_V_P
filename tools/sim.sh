#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p results
python3 tools/make_program.py
iverilog -g2012 -Wall -s tb_single_cycle -o results/core.vvp rtl/Single_Cycle.v sim/tb_single_cycle.v
vvp results/core.vvp | tee results/simulation.log
