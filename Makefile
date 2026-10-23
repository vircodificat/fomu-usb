# Fomu UART-over-USB PoC — build / test / synth flow.
#
# HARD RULE (plan §0 rule 5, §6): there is NO upload/flash target in this
# Makefile and there must never be one.  `vir upload` now builds a
# bitstream (src/top.vir carries the platform export) but flashing/
# hardware bring-up is the operator's call — nothing here runs it.

VIR      := $(HOME)/.local/virdant/bin/vir
IVERILOG := iverilog
VVP      := vvp
YOSYS    := yosys
NEXTPNR  := nextpnr-ice40

SHELL := /bin/bash

# Vendored plain-Verilog USB core + PoC top (the entire PoC DUT).
USB_SRVS := ext/usb_uart.sv
TOP_SV   := ext/fomu_top.sv

# Testbench sources.  tb/usb_host_bfm.v is an include file (tasks/macros
# meant to be `include`d inside a TB module) and is never compiled on its own.
TB_SIM_BFM_INC := tb/usb_host_bfm.v

.PHONY: all build test synth tools clean
.DEFAULT_GOAL := all

all: build test synth

## build — Virdant-side green check.  src/top.vir now carries a real
## `export mod Top for ice40::Fomu` (platform wrapper), so vir build
## emits the wrapper `module Top` (instantiating the ext mod
## \top::FomuTop) into build/top.sv plus the manifest.  The check is
## that the wrapper EXISTS — the old "top.sv must stay empty"
## invariant guarded against the ext mod drifting into codegen and
## died with the platform export.
build:
	$(VIR) build
	@test -s build/top.sv || { \
	    echo 'FAIL: build/top.sv is empty — no platform wrapper emitted'; \
	    exit 1; }

## test — the PoC's primary acceptance: all iverilog testbenches pass.
test: build test-crc test-usb-phy test-usb-serial

test-crc: tools
	$(IVERILOG) -g2012 -gspecify -o build/tb_crc.vvp $(USB_SRVS) tb/tb_crc.v
	$(VVP) build/tb_crc.vvp

test-usb-phy: tools
	$(IVERILOG) -g2012 -gspecify -o build/tb_usb_phy.vvp \
	    $(USB_SRVS) tb/usb_host_bfm.v tb/tb_usb_phy.v
	$(VVP) build/tb_usb_phy.vvp

test-usb-serial: tools
	$(IVERILOG) -g2012 -gspecify -o build/tb_usb_serial.vvp \
	    $(USB_SRVS) $(TOP_SV) tb/sb_io_sim.v tb/usb_host_bfm.v \
	    tb/tb_usb_serial.v
	$(VVP) build/tb_usb_serial.vvp

## synth — yosys synth_ice40 + nextpnr UP5K at 48 MHz on the same design
## `vir upload` would flash: the platform wrapper (build/top.sv) + the
## ext core + the descriptor/FIFO BRAMs.  Plain tool flow for timing
## measurement; still NO upload target (see header).
synth: build tools
	mkdir -p build
	set -o pipefail; \
	$(YOSYS) -p "read_verilog $(USB_SRVS) $(TOP_SV) build/top.sv; synth_ice40 -top Top -json build/top.json" \
	    | tee build/yosys.log
	set -o pipefail; \
	$(NEXTPNR) --up5k --package uwg30 --json build/top.json \
	    --pcf fomu.pcf --asc build/top.asc --freq 48 2>&1 | tee build/nextpnr.log

## tools — verify the toolchain is present before using it.
tools:
	@command -v $(IVERILOG) >/dev/null || { echo 'FAIL: iverilog not on PATH'; exit 1; }
	@command -v $(YOSYS)    >/dev/null || { echo 'FAIL: yosys not on PATH'; exit 1; }
	@command -v $(NEXTPNR)  >/dev/null || { echo 'FAIL: nextpnr-ice40 not on PATH'; exit 1; }
	@test -x $(VIR) || { echo 'FAIL: vir not at $(VIR)'; exit 1; }

clean:
	rm -f build/*.vvp build/top.json build/top.asc build/*.log