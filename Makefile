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
PANDOC   := pandoc

# Wiki sources and rendered output (see `make html`).
WIKI_SRC := $(wildcard wiki/*.md)
WIKI_OUT := $(patsubst wiki/%.md,build/html/%.html,$(WIKI_SRC))
WIKI_CSS := wiki/style.css

SHELL := /bin/bash

# Vendored plain-Verilog USB core + the ext bodies of the Virdant/Verilog
# split (src/usb.vir): the SB_IO pad pair (the only `inout` in the
# design), the parameterized POR, and the USB core itself.
USB_SRVS := ext/usb_uart.sv
PADS_SV  := ext/usb_pads.sv
POR_SV   := ext/usb_por.sv
ECHO_SV  := ext/echo_fifo.sv
EXT_SRVS := $(USB_SRVS) $(PADS_SV) $(POR_SV) $(ECHO_SV)
USB_BUILD := build/usb.sv build/echo.sv build/uart.sv

# Testbench sources.  tb/usb_host_bfm.v is an include file (tasks/macros
# meant to be `include`d inside a TB module) and is never compiled on its own.
TB_SIM_BFM_INC := tb/usb_host_bfm.v

## html — render wiki/*.md to build/html/*.html for browser reading.
## One page per module plus the overview pages; Home.html is the index.
## The dark stylesheet (wiki/style.css) is INLINED into every page via
## --include-in-header, so each file is self-contained (a separate
## style.css once silently failed to copy and the pages rendered white).
## Mermaid diagrams are emitted as plain code blocks (no JS in the output).
html: $(WIKI_OUT) | build/html

build/html:
	mkdir -p build/html

build/html/style.header.html: $(WIKI_CSS) | build/html
	printf '<style>\n' > $@
	cat $(WIKI_CSS) >> $@
	printf '</style>\n' >> $@

build/html/%.html: wiki/%.md build/html/style.header.html | build/html
	$(PANDOC) --from gfm --to html5 --standalone --toc --toc-depth=2 \
	    --include-in-header=build/html/style.header.html \
	    --metadata title="$*" \
	    --metadata pagetitle="$*" $< \
	    | sed -E 's/href="([^"#?]*)\.md"/href="\1.html"/g' > $@

.PHONY: all build test synth tools clean html
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
	    echo 'FAIL: build/top.sv is empty -- no platform wrapper emitted'; \
	    exit 1; }
	# NOTE: vir build copies the ext sources into build/ext/ itself and
	# its generated script reads them from there -- do NOT copy ext/*.sv
	# into build/ manually (a stale manual copy once made vir upload
	# synthesize old RTL, and a fresh one now collides with vir's own
	# copy: 'Re-definition of module').

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
	    $(EXT_SRVS) $(USB_BUILD) \
	    tb/sb_io_sim.v tb/usb_host_bfm.v \
	    tb/tb_usb_serial.v
	$(VVP) build/tb_usb_serial.vvp

## synth — yosys synth_ice40 + nextpnr UP5K at 48 MHz on the same design
## `vir upload` would flash: the platform wrapper (build/top.sv) + the
## ext core + the descriptor/FIFO BRAMs.  Plain tool flow for timing
## measurement; still NO upload target (see header).
synth: build tools
	mkdir -p build
	set -o pipefail; \
	$(YOSYS) -p "read_verilog $(EXT_SRVS) $(USB_BUILD) build/top.sv; synth_ice40 -top Top -json build/top.json" \
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
	rm -rf build/html