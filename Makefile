# Fomu UART-over-USB PoC — build / test / synth flow.
#
# HARD RULE (plan §0 rule 5, §6): there is NO upload/flash target in this
# Makefile and there must never be one.  `vir bitstream`/`vir upload` fail by
# construction in this repo (the ext-mod Top has no platform `for` clause),
# and nothing in this file calls any flashing tool.

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

## build — Virdant-side green check only.  The ext-mod Top emits no Verilog
## body, so build/top.sv must come out empty (a non-empty file means the
## port contract in src/top.vir drifted into codegen — fail loudly).
build:
	$(VIR) build
	@test ! -s build/top.sv || { \
	    echo 'FAIL: build/top.sv is not empty — ext mod Top emitted a body'; \
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

## synth — yosys synth_ice40 + nextpnr UP5K at 48 MHz.  Plain tool flow:
## `vir bitstream` cannot run here (ext-mod Top has no platform) and must
## not exist as a target.
synth: tools
	mkdir -p build
	set -o pipefail; \
	$(YOSYS) -p "read_verilog $(USB_SRVS) $(TOP_SV); synth_ice40 -top Top -json build/top.json" \
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