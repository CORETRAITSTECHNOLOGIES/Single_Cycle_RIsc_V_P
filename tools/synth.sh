#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p results
yosys -l results/synthesis.log tools/synth.ys
