# `usb_uart` — core top

`usb_uart` (`ext/usb_uart.sv:1595`) is the **top level of the USB core**: it
owns the descriptor ROM, the token router, the response arbiter, the
endpoint FIFOs, the byte-stream interface — and instantiates every other
module in the stack. See [system-overview.md](system-overview.md) and
[fomu-top.md](fomu-top.md) for how the PoC top wires it (loopback:
`tx_data = rx_data`, `tx_strobe = rx_valid`).

Scope (file header, `ext/usb_uart.sv:8`–`11`): FS only, one configuration,
EP0 control + EP1 IN/OUT bulk, EP2 IN interrupt (CDC notification,
descriptors only — always NAKed). Single 48 MHz clock domain (4 clks per
USB FS bit), no PLL, no CDC.

## Module structure

`usb_uart` instantiates, bottom-up:

| Submodule | Role | See |
|---|---|---|
| `usb_rx_phy` | pads → bit stream, SOP/EOP, bus-reset detect | [usb-rx-phy.md](usb-rx-phy.md) |
| `usb_rx_packet` | bit stream → token/data/handshake records | [usb-rx-packet.md](usb-rx-packet.md) |
| `usb_tx_phy` | packet records → pads | [usb-tx-phy.md](usb-tx-phy.md) |
| `usb_fifo16x8` ×2 | rx FIFO (64-deep) + tx FIFO (16-deep) | [usb-fifo.md](usb-fifo.md) |
| `usb_ep0` | control endpoint state machine, CDC requests | [usb-ep0.md](usb-ep0.md) |
| `usb_serial` | EP1 IN/OUT bulk protocol | [usb-serial.md](usb-serial.md) |
| `usb_crc5` / `usb_crc16` | CRC LFSRs (used inside packet/phy) | [usb-crc.md](usb-crc.md) |

Data flows: **RX chain** (`ext/usb_uart.sv:1618`–`1649`) — `usb_rx_phy`
produces bits, `usb_rx_packet` produces `tk_*` (token) / `dt_*` (data) /
`hs_*` (handshake) strobes plus a payload RAM read port. **TX chain**
(`ext/usb_uart.sv:2233`–`2288`) — the response arbiter's `tx_start` /
`tx_pid_w` / `tx_len_w` drive `usb_tx_phy`, which pulls bytes from a
registered source mux (`snap_q`, `ep0_small_q`, or `desc_q`,
`ext/usb_uart.sv:2278`).

## Descriptor ROM — what the device says it is

A 256×8 initialized BRAM (`ext/usb_uart.sv:1973`) holds every descriptor as
contiguous pages (`rom_page`, `ext/usb_uart.sv:2244`); EP0 selects a page +
offset at arm time (`rom_base_r`, `ext/usb_uart.sv:2257`):

| `resp_sel` | Page start | Size | Descriptor |
|---|---|---|---|
| 0 | 0 | 18 B | **Device**: bcdUSB 2.00, class `EF` (misc), subclass 02, protocol 01 (IAD — needed to bind CDC data + comm), bMaxPacketSize0 = 64, idVendor 0x1209, idProduct 0x7A10, 1 configuration |
| 1 | 18 | 75 B | **Configuration** (total 75 B): IAD (interface 0–1, class EF) + **CDC Communication interface 0** (class 02, subclass 00, protocol 01): header (24h), call management (01), ACM (02 — with `SendBreak`), union (interfaces 0+1), **EP2 IN interrupt** (03h, 8 B maxpacket), + **CDC Data interface 1** (class 0A): **EP1 IN bulk** (64 B) + **EP1 OUT bulk** (64 B) |
| 2 | 93 | 4 B | **String 0** (LANGIDs: en-US 0x0409) |
| 3 | 97 | 10 B | **String 1** (manufacturer: "Fomu") |
| 4 | 107 | 28 B | **String 2** (product: "Fomu UART PoC") |
| 5 | — | 7 B | **line coding** — dynamic, from `usb_ep0`'s `line_coding` register, bypasses ROM (`ext/usb_uart.sv:2270`) |
| 6 | — | 2 B | **GET_STATUS** — two zero bytes (`ext/usb_uart.sv:2269`) |

The descriptors *must* match what the core implements: cdc_acm on Linux
refuses a CDC function with no interrupt endpoint — hence EP2 exists in the
descriptors even though the core never arms it (`ext/usb_uart.sv:8`–`11`).

## Token router — who answers the host

`ext/usb_uart.sv:1770`–`1850`. Two register stages:

1. **Address filter** (`tok_v_r`, `ext/usb_uart.sv:1786`): a token is ours
   iff `tk_valid && tk_crc_ok && (tk_addr == dev_addr)`. The device answers
   **only its own address** — `dev_addr` starts 0 (the enumeration default)
   and is latched by `usb_ep0` when SET_ADDRESS completes. SOF tokens
   decode (frame number is captured) but are otherwise ignored.
2. **Endpoint decode** (`ext/usb_uart.sv:1796`–`1824`): PID+endpoint
   demux into per-endpoint strobes: `ep0_tk_setup/in/out`,
   `ser_tk_in/out` (EP1), `ep2_tk_in/out` (EP2).

Registering the decode twice costs +1 clk each and is *free*: the
turnaround budget is ~30–50 clks and no new packet can arrive within
8 clks (≥2 bit inter-packet gap). `tk_pid`/`tk_endp` hold their values
between packets, so a delayed read is bit-identical.

**Data packet router** (`ext/usb_uart.sv:1826`–`1844`): the module that is
*expecting* a data packet owns it — `ep0_busy` gates EP0's (registered,
`ext/usb_uart.sv:1839`–`1843`); `ser_busy` (= `armed_out`, see
[usb-serial.md](usb-serial.md)) gates EP1's (`ser_dt_valid`,
`ext/usb_uart.sv:1844`).

**Handshake router** (`ext/usb_uart.sv:1847`–`1849`): the module with a
response **in flight** owns the ACK — `ep0_wait_ack` first, else
`ser_ack_wait`.

## Response arbiter — choosing the reply

`ext/usb_uart.sv:1851`–`1882`. Only one module wants the wire at a time
(protocol order guarantees it: one response per token/data packet), so a
priority mux + an in-flight `tx_owner` register suffices:

```systemverilog
tx_pid_w = ep0_resp_req ? ep0_resp_pid
          : ser_resp_req ? ser_resp_pid
          : PID_NAK;                    // EP2: nothing armed
```

- `ep0_resp_req` (from `usb_ep0`) > `ser_resp_req` (from `usb_serial`) >
  EP2. EP2 is the notification endpoint: the core **never arms it**, so
  every IN/OUT token to EP2 gets a registered NAK (`ep2_nak_req`,
  `ext/usb_uart.sv:1861`) — the host polls it forever and always gets NAK.
- `tx_owner` (`ext/usb_uart.sv:1867`–`1875`) holds the bus while `usb_tx_phy`
  transmits (`tx_busy`) and clears on `tx_done`.

## Byte-stream interface — the contract

`ext/usb_uart.sv:1603`–`1613`. The byte-stream FIFO contract (plan §2;
loopback wired in the PoC Top):

```
rx_data/rx_valid  — device → consumer: one strobe per byte (= rx FIFO pop)
rx_ready          — consumer → device: low = stop popping (the rx FIFO fills
                    and the serial endpoint NAKs bulk-OUT instead)
tx_data/tx_strobe — consumer → device: push one byte per strobe
tx_ready          — device → consumer: tx FIFO has room
```

- **`rx_valid`** is a 1-clk strobe; the popped byte rides `rx_data`
  (`rx_pop_d` → `rx_valid_r` → `rx_valid_rr` 3-stage pipe,
  `ext/usb_uart.sv:1670`–`1698`). Each pulse *is* the rx FIFO pop — the
  consumer doesn't need a separate "consumed" signal; it signals room via
  `rx_ready` instead.
- **Pop gating** (`ext/usb_uart.sv:1691`): pop only while the rx FIFO has a
  byte AND `txfifo_cnt < 13` (headroom for up to 3 in-flight pushes in the
  3-stage loopback pipe + 1 margin) AND `rx_ready`. A popped byte *must*
  land in the tx FIFO — nothing else can re-read it from the rxfifo.
- **`rx_ready` low → NAK**: with the Virdant echo server as consumer,
  `rx_ready` throttles the pop rate to the UART's 115200-baud drain rate.
  The rx FIFO fills, `usb_serial`'s `rx_room` test fails → NAK → the host
  retries. End-to-end flow control with no byte loss
  (`ext/usb_uart.sv:1690`).
- **`tx_strobe`** pushes into the tx FIFO, gated combinationally with
  `!txfifo_full` **at the write edge** (`ext/usb_uart.sv:1723`) — a
  registered gate would be stale by the time the push lands.
- **`tx_ready`** = `!txfifo_full` (`ext/usb_uart.sv:1727`).
- Bulk-IN pulls from the tx FIFO: on a fresh arm, the **snapshot loader**
  (`ext/usb_uart.sv:1889`–`1920`) pops `min(count,64)` bytes into a 64-byte
  BRAM snapshot one byte per clk (byte 0 isn't needed until ~72 clks after
  the arm — the load always wins the race). Retransmissions replay the
  snapshot, never re-pop the FIFO (see [usb-serial.md](usb-serial.md)).
- Bulk-OUT accept → **rx push** (`ext/usb_uart.sv:1922`–`1963`): accepted
  payload bytes stream from `usb_rx_packet`'s payload RAM into the rx FIFO
  one byte per clk, index and byte pipelined to match the BRAM's
  registered read (`dt_rd_addr = push_i`, `ext/usb_uart.sv:1648`).

## Reset behavior

`rst` is sync, active high (POR in the top). `bus_reset` (detected by
`usb_rx_phy`) clears the FIFOs (`rst || bus_reset`,
`ext/usb_uart.sv:1701`/`1714`), the router strobes, the snapshot loader —
and propagates to `usb_serial` and `usb_ep0` to clear toggles and address.
SET_CONFIGURATION completion toggles `cfg_strobe` (`configured ^
configured_d`, `ext/usb_uart.sv:2290`–`2295`) to reset the *bulk toggles
only* (USB 2.0 §9.1.1.5). `connected_o` = `configured`.

## Ports

| Port | Dir | Width | Meaning |
|---|---|---|---|
| `clk` | in | 1 | 48 MHz core clock |
| `rst` | in | 1 | sync reset, active high (POR in Top) |
| `dp_rx` / `dn_rx` | in | 1 | pad readback (SB_IO D_IN_0) |
| `dp_tx` / `dn_tx` | out | 1 | pad drive (SB_IO D_OUT_0) |
| `oe` | out | 1 | pad drive enable (SB_IO OUTPUT_ENABLE) |
| `rx_data` | out | 8 | popped rx byte |
| `rx_valid` | out | 1 | 1-clk strobe per popped byte |
| `tx_data` | in | 8 | byte to push |
| `tx_strobe` | in | 1 | tx FIFO push strobe |
| `rx_ready` | in | 1 | consumer room; low → stop popping → NAK |
| `tx_ready` | out | 1 | tx FIFO has room |
| `bus_reset_o` | out | 1 | bus reset detected (TB visibility) |
| `connected_o` | out | 1 | SET_CONFIGURATION completed |

## Key takeaways

- `usb_uart` is the core top: descriptor ROM (256×8 BRAM), token router
  (address filter + endpoint decode), response arbiter (priority mux +
  `tx_owner`), snapshot loader, rx push, FIFOs, and the byte-stream
  interface.
- Descriptors live in a ROM as contiguous pages (device @0, config @18,
  str0 @93, str1 @97, str2 @107); line coding and GET_STATUS are dynamic and
  bypass the ROM. The descriptors must match the core: EP2 exists because
  Linux cdc_acm requires a CDC interrupt endpoint — the core NAKs it
  forever.
- The device answers only its own `dev_addr` (latched at SET_ADDRESS);
  SOF decodes but is ignored.
- Token strobes are registered twice and data strobes once — timing closure
  tricks that are free because the turnaround budget (~30–50 clks) dwarfs
  the 1-clk costs.
- Byte-stream contract: `rx_valid` strobes *are* rx FIFO pops; `rx_ready`
  low stops pops, fills the rx FIFO, and makes the serial endpoint NAK —
  USB's own retry mechanism becomes end-to-end flow control.
- A fresh bulk-IN arm snapshots `min(count,64)` bytes from the tx FIFO into
  `snap[]`, so retransmissions are byte-identical until ACKed.
- `bus_reset` clears everything; SET_CONFIGURATION additionally resets the
  bulk data toggles (§9.1.1.5).

Further reading: [system-overview.md](system-overview.md),
[fomu-top.md](fomu-top.md), [echo-server.md](echo-server.md),
[usb-primer.md](usb-primer.md).