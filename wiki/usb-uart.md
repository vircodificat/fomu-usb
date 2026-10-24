# `usb::UsbUart` — core top

`usb::UsbUart` (`mod UsbUart` in `src/usb.vir`) is the **top level of the
USB core**: it owns the descriptor ROM plumbing, the token router, the
response arbiter, the endpoint FIFOs, the snapshot loader, the rx push,
the byte-stream interface — and instantiates every other module in the
stack. It is wrapped by `usb::UsbController` (also in `src/usb.vir`),
which registers the power-on reset and exposes the pad-facing wires (see
[fomu-top.md](fomu-top.md) for how `src/top.vir` wires it: `UsbPor`
→ `UsbController` → `UsbPads`, with the echo server
([echo-server.md](echo-server.md)) as the byte-stream consumer).

Scope (the `src/usb.vir` file header): FS only, one configuration, EP0
control + EP1 IN/OUT bulk; the EP2 IN interrupt endpoint exists in the
descriptors only (Linux `cdc_acm` refuses a CDC function without one)
and is never armed — every token to it is NAKed. Single 48 MHz clock
domain (4 clks per USB FS bit), no PLL, no CDC.

## Module structure

`usb::UsbUart` instantiates, bottom-up (instance → mod):

| Instance | Mod | Role | See |
|---|---|---|---|
| `rxphy` | `usb::RxPhy` | pads → bit stream, SOP/EOP, bus-reset detect | [usb-rx-phy.md](usb-rx-phy.md) |
| `rxpkt` | `usb::RxPacket` | bit stream → token/data/handshake records | [usb-rx-packet.md](usb-rx-packet.md) |
| `txphy` | `usb::TxPhy` | packet records → pads | [usb-tx-phy.md](usb-tx-phy.md) |
| `rxfifo` | `\usb::RxFifo` (ext body of `usb_fifo16x8`) | 64-deep × 8-bit bulk-OUT FIFO, BRAM storage | [usb-fifo.md](usb-fifo.md) |
| `txfifo` | `\usb::TxFifo` (ext body of `usb_fifo16x8`) | 16-deep × 8-bit bulk-IN FIFO, flop storage | [usb-fifo.md](usb-fifo.md) |
| `ep0` | `usb::Ep0` | control endpoint state machine, CDC requests | [usb-ep0.md](usb-ep0.md) |
| `serial` | `usb::Serial` | EP1 IN/OUT bulk protocol | [usb-serial.md](usb-serial.md) |
| `snapram` | `usb::SnapRam` | 64×8 bulk-IN snapshot RAM (one `SB_RAM40_4K`) | — |
| `descrom` | `\usb::DescRom` | 256×8 initialized descriptor ROM | — |

`usb::Crc5` / `usb::Crc16` live one level down — instantiated inside
`usb::RxPacket` (checker) and `usb::TxPhy` (generator). The only ext
bodies left in `ext/usb_uart.sv` are the two FIFOs and the descriptor
ROM (indexed memories Virdant cannot express inside a plain `mod`);
the pads are `usb::UsbPads` (`ext/usb_pads.sv`).

Data flows. **RX chain**: `rxphy` produces decoded bits, `rxpkt` turns
them into `tk_*` (token) / `dt_*` (data) / `hs_*` (handshake) strobes
plus a payload RAM read port — `rxpkt.dt_rd_addr` is wired to the rx
push index `push_i[6..0]`, so accepted payload bytes stream out through
the payload RAM's registered read. **TX chain**: the response arbiter's
`tx_start` / `tx_pid_w` / `tx_len_w` drive `txphy`, which pulls bytes
from a registered producer mux (`tx_byte`: `snapram.rd` for serial data,
zero bytes for GET_STATUS, `desc_q` for everything else).

## Descriptor ROM — what the device says it is

A 256×8 initialized BRAM (`\usb::DescRom`, contents in the ext body's
`initial` block) holds every descriptor as contiguous pages; `usb::UsbUart`'s
`rom_page` map selects the page and `rom_base_r` latches page start +
transfer offset at arm time (on `ep0_resp_req`):

| `resp_sel` | Page start | Size | Descriptor |
|---|---|---|---|
| 0 | 0 | 18 B | **Device**: bcdUSB 2.00, class `EF` (misc), subclass 02, protocol 01 (IAD — needed to bind CDC data + comm), bMaxPacketSize0 = 64, idVendor 0x1209, idProduct 0x7A10, 1 configuration |
| 1 | 18 | 75 B | **Configuration** (total 75 B): IAD (interface 0–1, class EF) + **CDC Communication interface 0** (class 02, subclass 00, protocol 01): header (24h), call management (01), ACM (02 — with `SendBreak`), union (interfaces 0+1), **EP2 IN interrupt** (03h, 8 B maxpacket), + **CDC Data interface 1** (class 0A): **EP1 IN bulk** (64 B) + **EP1 OUT bulk** (64 B) |
| 2 | 93 | 4 B | **String 0** (LANGIDs: en-US 0x0409) |
| 3 | 97 | 10 B | **String 1** (manufacturer: "Fomu") |
| 4 | 107 | 32 B declared | **String 2** (product: "Fomu UART PoC", bLength 0x20 — the UTF-16 string data is zero-padded up to the declared length) |
| 6 | — | 2 B | **GET_STATUS** — two zero bytes; bypasses the ROM (the `tx_byte` producer mux returns zeros for sel 6) |

There is no sel 5: `usb::Ep0` implements no line-coding request, and an
invalid GET_DESCRIPTOR select STALLs instead of arming.

The descriptors *must* match what the core implements: `cdc_acm` on Linux
refuses a CDC function with no interrupt endpoint — hence EP2 exists in the
descriptors even though the core never arms it.

## Token router — who answers the host

Two register stages in `mod UsbUart`:

1. **Address filter** (`tok_v_r`): a token is ours iff
   `rxpkt.tk_valid && rxpkt.tk_crc_ok && (rxpkt.tk_addr == ep0.dev_addr)`.
   The device answers **only its own address** — `dev_addr` starts 0 (the
   enumeration default) and is latched by `usb::Ep0` when SET_ADDRESS
   completes. SOF tokens decode (the frame number is captured by `rxpkt`)
   but are otherwise ignored.
2. **Endpoint decode** (`ep0_tk_setup_r` / `ep0_tk_in_r` / `ep0_tk_out_r`,
   `ser_tk_in_r` / `ser_tk_out_r`, `ep2_tk_in_r` / `ep2_tk_out_r`):
   PID+endpoint demux into per-endpoint strobes — SETUP/IN/OUT at
   endpoint 0 → `ep0`; IN/OUT at endpoint 1 → `serial`; IN/OUT at
   endpoint 2 → only feed EP2's NAK.

Registering the decode twice costs +1 clk each and is *free*: the
turnaround budget is ~30–50 clks and no new packet can arrive within
8 clks (≥2 bit inter-packet gap). `tk_pid`/`tk_endp` hold their values
between packets, so a delayed read is bit-identical.

**Data packet router**: the module that is *expecting* a data packet owns
it. EP0's strobe is registered one clk after arrival (`ep0_dt_valid_r` —
the `rxpkt` EOP outputs (`dt_len`, `dt_crc_ok`) carry wide cones, and
every field EP0 reads holds until the next packet's EOP, so the delayed
read is bit-identical); the gate is `ep0_data_take` =
`ep0.ep0_busy || (ep0.ep0_stalling && last_out_ep0)` — while EP0 STALLs,
it still owns the stalled transfer's own data stage, tracked by the
`last_out_ep0` register ("last valid token was OUT-to-EP0").
`usb::Serial` keeps the arrival-cycle strobe (`ser_dt_valid` =
`rxpkt.dt_valid && serial.ser_busy && !ep0_data_take`; `ser_busy` is
`armed_out`) — its accept decision is itself registered one clk later
(`dt_acc`/`dt_rej`), so its cone is not critical.

**Handshake router**: the module with a response **in flight** owns the
ACK — `ep0.ep0_wait_ack` first, else `serial.ser_ack_wait` (both gated on
`hs_pid` = ACK).

## Response arbiter — choosing the reply

Only one module wants the wire at a time (protocol order guarantees it:
one response per token/data packet), so a priority mux + an in-flight
`tx_owner` register suffices:

```systemverilog
tx_pid_w = ep0_resp_req ? ep0.resp_pid
          : ser_resp_req ? serial.resp_pid
          : 10w4;                       // 10 = the NAK pid: EP2, nothing armed
```

- `ep0_resp_req` (from `usb::Ep0`) > `ser_resp_req` (from `usb::Serial`)
  > EP2. EP2 is the notification endpoint: the core **never arms it**, so
  every IN/OUT token to EP2 gets a NAK from the **registered**
  `ep2_nak_req` (a combinational NAK off the token decode would drag the
  whole token cone into the tx engine's state logic; one clk later is
  still far inside the turnaround budget) — the host polls it forever and
  always gets NAK.
- `tx_owner` (2 bits: 0 none, 1 ep0, 2 serial, 3 ep2) is set at
  `tx_start` and cleared on `txphy.tx_done`; it holds the bus while
  `usb::TxPhy` transmits.

## Byte-stream interface — the contract

The byte-stream FIFO contract, consumed by the echo server in
`src/top.vir`:

```
rx_data/rx_valid  — device → consumer: one strobe per byte (= rx FIFO pop)
rx_ready          — consumer → device: low = stop popping (the rx FIFO fills
                    and the serial endpoint NAKs bulk-OUT instead)
tx_data/tx_strobe — consumer → device: push one byte per strobe
tx_ready          — device → consumer: tx FIFO has room
```

- **`rx_valid`** is a 1-clk strobe; the popped byte rides `rx_data`
  (`rx_pop_d` → `rx_valid_r` → `rx_valid_rr` 3-stage pipe, with
  `rx_data_d` holding the byte from the FIFO's registered read port).
  Each pulse *is* the rx FIFO pop — the consumer doesn't need a separate
  "consumed" signal; it signals room via `rx_ready` instead.
- **Pop gating** (`rx_pop_d`): pop only while the rx FIFO has a byte AND
  `txfifo.count < 13` (headroom for the bytes already in flight through
  the consumer's pipeline + 1 margin, so a landing push never meets a
  full tx FIFO and a pop is never silently dropped) AND `rx_ready`. A
  popped byte *must* land in the tx FIFO — nothing else can re-read it
  from the rx FIFO. One extra guard: no pop while the in-flight pop is
  consuming the last byte (the 7-bit count must never underflow).
- **`rx_ready` low → NAK**: with the echo server as consumer, `rx_ready`
  throttles the pop rate to the UART's 115200-baud drain rate. The rx
  FIFO fills, `usb::Serial`'s `rx_room` test fails → NAK → the host
  retries. End-to-end flow control with no byte loss.
- **`tx_strobe`** pushes into the tx FIFO through the **registered**
  write gate `txfifo_wr_d` (`tx_strobe && !txfifo.full`): the count
  full-compare stays out of every byte's write-enable cone. The +1 clk
  of gate staleness is safe — the only writer (the echo server) pushes
  at most one byte per UART byte time, and `txfifo_wr_d` re-checks
  `!full` at the registered write edge as belt-and-braces.
- **`tx_ready`** is **registered** too (`tx_ready_r` = `!txfifo.full`,
  one clk stale): the consumer's push decision then reads only
  registered terms. The same staleness argument makes it safe.
- Bulk-IN pulls from the tx FIFO: on a fresh arm (`arm_fresh` =
  `tx_start && ser_resp_req && serial.resp_fresh`), the **snapshot
  loader** pops `min(count,64)` bytes (`tx_len_w` = `serial.resp_len`)
  into the 64-byte snapshot RAM one byte per clk (byte 0 isn't needed
  until ~72 clks after the arm — the load always wins the race).
  Termination is a **countdown** (`rem` decrements per popped byte; the
  run drops when `rem == 1`, i.e. `any(rem[7..1])` is false — a 6-input
  OR into `load_run`'s D). Retransmissions replay the snapshot, never
  re-pop the FIFO (see [usb-serial.md](usb-serial.md)).
- Bulk-OUT accept → **rx push**: accepted payload bytes stream from
  `rxpkt`'s payload RAM into the rx FIFO one byte per clk, index and
  byte pipelined to match the BRAM's registered read
  (`rxpkt.dt_rd_addr = push_i`). Termination is a **countdown** too
  (`push_rem`), and the write strobe is registered one clk into
  `rx_push_run_d` with `!full` pre-gated inside `rx_push_run`'s own
  next-state — the FIFO write port sees two clean registers.

## Reset behavior

`rst` is sync, active high: `usb::UsbController` registers `por_done`
one clk into `por_rst` and feeds the inverted, active-high `core_rst` to
the core (the extra register keeps the pad-adjacent POR flop from
sourcing the core-wide reset net). `bus_reset` (detected by `usb::RxPhy`)
is ORed into the reset of the FIFOs (`rst || bus_reset` on both), the
router strobes, the snapshot loader, and the rx pop/push pipes — and
propagates to `usb::Serial` and `usb::Ep0` to clear toggles and address.
A few registers deliberately reset on `rst` **only**, not `bus_reset`:
`ep2_nak_req`, `tx_owner`, `rom_base_r`, and `configured_d`.
SET_CONFIGURATION completion toggles `cfg_strobe`
(`ep0.configured ^^ configured_d`) to reset the *bulk toggles only*
(USB 2.0 §9.1.1.5).

Separately from the protocol state, `usb::UsbUart` stretches the
byte-stream strobes into LED-visible activity outputs: `rxing` and
`txing` are 23-bit down-counters (`rx_led_cnt` / `tx_led_cnt`) that
reload `0x7f_ffff` on `rx_valid_rr` / `tx_strobe` and count down —
~175 ms past the last byte. They have **no reset** (power-on zero,
count down to rest) and sit outside every USB engine path; `src/top.vir`
drives the red/green LEDs (active low) from them.

## Ports

| Port | Dir | Width | Meaning |
|---|---|---|---|
| `clk` | in | 1 | 48 MHz core clock |
| `rst` | in | 1 | sync reset, active high (`core_rst` from `usb::UsbController`) |
| `dp_rx` / `dn_rx` | in | 1 | pad samples (SB_IO D_IN_0 via `usb::UsbPads`) |
| `dp_tx` / `dn_tx` | out | 1 | pad drive levels (SB_IO D_OUT_0 via `usb::UsbPads`) |
| `output_enable` | out | 1 | pad drive enable (SB_IO OUTPUT_ENABLE via `usb::UsbPads`) |
| `rx_data` | out | 8 | popped rx byte |
| `rx_valid` | out | 1 | 1-clk strobe per popped byte |
| `tx_data` | in | 8 | byte to push |
| `tx_strobe` | in | 1 | tx FIFO push strobe |
| `rx_ready` | in | 1 | consumer room; low → stop popping → NAK |
| `tx_ready` | out | 1 | tx FIFO has room |
| `rxing` | out | 1 | receiving (LED-stretched ~175 ms past the last byte) |
| `txing` | out | 1 | sending (LED-stretched ~175 ms past the last byte) |

## Key takeaways

- `usb::UsbUart` is the core top: descriptor ROM plumbing, token router
  (address filter + endpoint decode, both registered), response arbiter
  (priority mux + `tx_owner`), snapshot loader, rx push, FIFOs, and the
  byte-stream interface. `usb::UsbController` wraps it, registering the
  power-on reset.
- Descriptors live in a 256×8 ROM as contiguous pages (device @0, config
  @18, str0 @93, str1 @97, str2 @107); GET_STATUS (sel 6) is dynamic and
  bypasses the ROM; there is no sel 5 (no line-coding request — invalid
  selects STALL). The descriptors must match the core: EP2 exists because
  Linux `cdc_acm` requires a CDC interrupt endpoint — the core NAKs it
  forever.
- The device answers only its own `dev_addr` (latched at SET_ADDRESS);
  SOF decodes but is ignored.
- Token strobes are registered twice and EP0's data strobe once — timing
  closure tricks that are free because the turnaround budget (~30–50
  clks) dwarfs the 1-clk costs.
- Byte-stream contract: `rx_valid` strobes *are* rx FIFO pops; `rx_ready`
  low stops pops, fills the rx FIFO, and makes the serial endpoint NAK —
  USB's own retry mechanism becomes end-to-end flow control. The tx-side
  gates (`txfifo_wr_d`, `tx_ready_r`) are registered for the same
  timing reason, made safe by the slow, single-writer consumer.
- A fresh bulk-IN arm snapshots `min(count,64)` bytes from the tx FIFO
  into the snapshot RAM, so retransmissions are byte-identical until
  ACKed. Both the loader and the rx push terminate by countdown.
- `bus_reset` clears the FIFOs and all per-packet protocol state;
  SET_CONFIGURATION additionally resets the bulk data toggles
  (§9.1.1.5). The LED counters (`rxing`/`txing`, ~175 ms) deliberately
  have no reset.

Further reading: [system-overview.md](system-overview.md),
[fomu-top.md](fomu-top.md), [echo-server.md](echo-server.md),
[usb-primer.md](usb-primer.md).
