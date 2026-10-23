# `usb_ep0` — control transfers and enumeration

`usb_ep0` (`ext/usb_uart.sv:987`) is the control-transfer state machine for
**EP0**, the endpoint every USB device must implement. It is the module the
host talks to first: it answers **GET_DESCRIPTOR**, applies **SET_ADDRESS**
and **SET_CONFIGURATION**, and serves the CDC-specific requests
(**SET_LINE_CODING**, **GET_LINE_CODING**, **SET_CONTROL_LINE_STATE**). It
never touches the pads — it sees the world as router-gated packet strobes
from [`usb_uart`](usb-uart.md) and answers with small `resp_*` "please send
this" records that the tx PHY ([usb-tx-phy.md](usb-tx-phy.md)) turns into
wire packets.

Prerequisites: [usb-primer.md](usb-primer.md) (tokens, PIDs, packets),
[usb-rx-packet.md](usb-rx-packet.md) (where `dt_valid`/`dt_head` come from),
[system-overview.md](system-overview.md) (how the modules stack up).

## What a control transfer is

Recall from [usb-primer.md](usb-primer.md): a USB *transaction* is a token
packet, optionally a data packet, optionally a handshake. A control
*transfer* is a fixed three-act play built from such transactions
(USB 2.0 §8.5.3):

1. **SETUP stage** — the host sends a SETUP token followed by a **DATA0**
   data packet carrying exactly **8 request bytes**. The device must accept
   it and ACK.
2. **DATA stage** (optional) — either:
   - a *control read*: the host polls with IN tokens and the device sends
     `wLength` (or fewer) bytes, or
   - a *control write*: the host sends data with OUT tokens.
   No data stage at all is also legal (e.g. SET_ADDRESS).
3. **STATUS stage** — a zero-length transaction in the *opposite* direction
   of the data stage. For a control write (or no-data request) the *device*
   sends a zero-length **DATA1** packet ("status ZLP") in reply to an IN
   token; for a control read the *host* sends a zero-length DATA1 after an
   OUT token and the device ACKs it.

The 8 request bytes are a little fixed-layout structure. `usb_ep0` latches
them into `rq0..rq3, rq6, rq7` (`ext/usb_uart.sv:1035`, `ext/usb_uart.sv:1254`;
`wIndex` at bytes 4–5 is never needed and simply not kept):

| Byte | Register | Field | Meaning |
|------|----------|-------|---------|
| 0 | `rq0` | `bmRequestType` | direction (bit 7), type (standard/class), recipient |
| 1 | `rq1` | `bRequest` | which request (e.g. `0x06` = GET_DESCRIPTOR) |
| 2–3 | `rq2`/`rq3` | `wValue` | request-specific (descriptor type/index, address…) |
| 6–7 | `rq6`/`rq7` | `wLength` | bytes the data stage will carry |

The toggles follow strict rules (header comment, `ext/usb_uart.sv:966`):
SETUP is always DATA0 and **resets the toggle**; the *data* stage starts at
DATA1 for both directions; the status packet is always a zero-length DATA1.
See [Data toggles on EP0](#data-toggles-on-ep0) below.

## What enumeration is

Enumeration is the host's boot-time interrogation of a freshly attached
device, conducted entirely over EP0. A typical pass against this core:

1. **Bus reset** — the host holds SE0 for ≥ 2.5 µs; `usb_rx_phy` pulses
   `bus_reset` (`ext/usb_uart.sv:243`), which drops `dev_addr` to 0,
   `configured` to 0 and the state machine to `EP_IDLE`
   (`ext/usb_uart.sv:1212`).
2. **GET_DESCRIPTOR(device)** — the host learns vendor/product IDs and
   `bMaxPacketSize0 = 64` (ROM byte `desc_rom[7]`, `ext/usb_uart.sv:1982`).
   It asks for 64 bytes; the device sends `min(64, 18) = 18` — a *short
   packet*, which is how the data stage ends early.
3. **SET_ADDRESS(n)** — the host picks an address; the device applies it
   only when the transfer completes (see [the address latch](#the-address-latch)).
4. **GET_DESCRIPTOR(config)** at the new address — 75 bytes, delivered as a
   64-byte DATA1 packet plus an 11-byte DATA0 packet (two IN transactions).
   The host reads the string descriptors the same way.
5. **SET_CONFIGURATION(1)** — `configured` goes 1; the token router starts
   forwarding EP1 traffic to `usb_serial`, and a `cfg_strobe` pulse
   (`ext/usb_uart.sv:2292`) resets the bulk data toggles
   (USB 2.0 §9.1.1.5) — see [usb-serial.md](usb-serial.md).
6. **CDC requests** — `SET_CONTROL_LINE_STATE` (host asserts DTR when a
   program opens `/dev/ttyACM0`), `SET_LINE_CODING`/`GET_LINE_CODING`
   (baud rate). The device accepts them but the loopback demo ignores the
   settings beyond storing the 7 line-coding bytes.

After step 5 the host has a working CDC-ACM port; all bulk data moves on
EP1 ([usb-serial.md](usb-serial.md)). EP0's job is done until the host
re-enumerates.

## Ports

| Port | Dir | Width | Purpose |
|------|-----|-------|---------|
| `clk`, `rst`, `bus_reset` | in | 1 | 48 MHz clock, sync reset, bus-reset pulse |
| `tk_setup` / `tk_in` / `tk_out` | in | 1 | router-gated EP0 token strobes (address + endpoint matched) |
| `dt_valid` | in | 1 | router-gated data packet for EP0 (registered +1 clk) |
| `dt_pid`, `dt_len` | in | 4, 7 | data packet PID and byte length |
| `dt_head` | in | 64 | flopped copy of payload bytes 0–7 ([usb-rx-packet.md](usb-rx-packet.md)) |
| `dt_crc_ok` | in | 1 | CRC16 residual check passed ([usb-crc.md](usb-crc.md)) |
| `hs_ack` | in | 1 | EP0's in-flight response was ACKed by the host |
| `resp_req` | out | 1 | 1-clk pulse: start transmitting a response |
| `resp_pid`, `resp_len` | out | 4, 7 | response PID / payload length (0 = handshake or ZLP) |
| `resp_sel`, `resp_base` | out | 3, 10 | descriptor select + byte offset for the ROM read |
| `dev_addr` | out | 7 | device address (latched at status completion) |
| `configured` | out | 1 | SET_CONFIGURATION(1) has completed |
| `line_coding` | out | 56 | 7 CDC line-coding bytes |
| `ep0_busy` | out | 1 | EP0 expects a data packet (router hint, `ext/usb_uart.sv:1131`) |
| `ep0_wait_ack` | out | 1 | EP0 has a response in flight (handshake-router hint) |

Note what is *absent*: no FIFOs, no pads, no descriptor storage. The module
is pure protocol. The two hint outputs are how the router in
[usb_uart.md](usb-uart.md) decides who owns each incoming data packet
(`ep0_dt_gate = dt_valid && ep0_busy`, `ext/usb_uart.sv:1839`) and each
incoming ACK (`ep0_hs_ack`, `ext/usb_uart.sv:1847`).

## The state machine

`usb_ep0` is a classic registered FSM (`st`, 11 states,
`ext/usb_uart.sv:1023`–`1033`) plus a small pipeline of helper registers.
The state encodings are deliberately non-sequential: `EP_PARSE = 8`,
`EP_SETUP = 9`, `EP_ARM = 10` are the three decode stages that follow
`EP_RCV_REQ = 1`, written in the order the request flows through them.

```mermaid
stateDiagram-v2
    [*] --> EP_IDLE
    EP_IDLE --> EP_RCV_REQ : SETUP token
    EP_RCV_REQ --> EP_PARSE : DATA0 8 bytes ok - ACK
    EP_PARSE --> EP_SETUP : decode request flags
    EP_SETUP --> EP_ARM : route one-hot action
    EP_ARM --> EP_SEND : read request with data stage
    EP_ARM --> EP_RCV_DATA : SET_LINE_CODING
    EP_ARM --> EP_STATUS_I : no-data write or wLength 0
    EP_ARM --> EP_STATUS_O : read with wLength 0
    EP_ARM --> EP_STALL : unsupported request
    EP_SEND --> EP_WAIT_ACK : IN token - arm data packet
    EP_WAIT_ACK --> EP_SEND : ACK and more bytes remain
    EP_WAIT_ACK --> EP_STATUS_O : ACK and data stage done
    EP_WAIT_ACK --> EP_IDLE : ACK of status ZLP
    EP_WAIT_ACK --> EP_SEND : IN retry - rearm same packet
    EP_WAIT_ACK --> EP_STATUS_I : IN retry of status ZLP
    EP_RCV_DATA --> EP_RCV_DATA : OUT data packet accepted - ACK
    EP_RCV_DATA --> EP_STATUS_I : wLength bytes received
    EP_STATUS_I --> EP_WAIT_ACK : IN token - arm DATA1 ZLP
    EP_STATUS_O --> EP_IDLE : host ZLP accepted - ACK
    EP_STALL --> EP_STALL : IN or OUT token - STALL
    note right of EP_IDLE
        A SETUP token from ANY state jumps to EP_RCV_REQ
        and aborts the transfer in progress
        (USB 2.0 8.5.2.2)
    end note
```

### Why four states just to read a request

Decoding "which request is this?" means comparing byte values at three
levels (request type → descriptor type → string index). That compare tree
is too deep for one 48 MHz clock, so it is *pipelined one compare per
clock* (header comment, `ext/usb_uart.sv:973`):

- **`EP_RCV_REQ`** — wait for the SETUP DATA0 (8 bytes, CRC good:
  `req_acc_r`, `ext/usb_uart.sv:1192`), ACK it, latch the request bytes
  into `rq*` (`ext/usb_uart.sv:1254`).
- **`EP_PARSE`** — turn `rq*` into one-hot *flags*: `p_is_desc`,
  `p_is_set_addr`, `p_is_set_lc`, … (`ext/usb_uart.sv:1276`–`1283`), plus
  `p_wlen`, the descriptor select `p_sel`, and the side-effect values
  `addr_pending`/`cfg_pending` with their ownership tags.
- **`EP_SETUP`** — route the flags into a single one-hot **action**
  register `p_action` (`ext/usb_uart.sv:1313`) and precompute the
  data-stage length `p_dremain_r = min(wLength, desc_len)` for descriptor
  reads (`ext/usb_uart.sv:1310`).
- **`EP_ARM`** — apply the action to the datapath (`sel`, `remain`, `tog`,
  next state) with shallow decodes only (`ext/usb_uart.sv:1323`).

The same idea recurs everywhere: wide decisions land on their *own*
register one clock early, and the FSM reads the register. The three
data-stage accepts (`req_acc_r`, `rcd_acc_r`, `sto_acc_r`,
`ext/usb_uart.sv:1192`–`1198`) and the STALL trigger (`stall_resp_r`,
`ext/usb_uart.sv:1207`) are registered copies of accept conditions; the
FSM reacts one clock late. That is bit-identical and free: USB gives a
device ~30–50 clocks to answer a token, and every field involved
(`dt_pid`, `dt_len`, `dt_head`, `dt_crc_ok`) is itself a register that
holds until the next packet.

### Three worked examples

**Control read — GET_DESCRIPTOR(device)** (`80 06 01 00 00 00 40 00`):
EP_RCV_REQ latches and ACKs → EP_PARSE sets `p_is_desc`, `p_sel = 0`,
`p_wlen = 64` → EP_SETUP computes `p_action = 5` (SEND_DESC) and
`p_dremain_r = min(64, 18) = 18` → EP_ARM arms: `sel = 0`, `tog = 1`,
`remain = 18`, state EP_SEND. The host's IN token makes EP_SEND arm an
18-byte DATA1 response (`resp_sel = 0`, `resp_base = 0`,
`ext/usb_uart.sv:1367`); `usb_uart` computes the ROM address and the tx
PHY streams the bytes out (see [descriptor ROM reads](#descriptor-rom-reads)).
The host ACKs → `last_r` was set (18 ≤ 64) → EP_STATUS_O. The host sends
OUT + a zero-length DATA1, the device ACKs (`sto_acc_r`,
`ext/usb_uart.sv:1436`) and returns to EP_IDLE.

**Control write, no data — SET_ADDRESS(9)** (`00 05 09 00 00 00 00 00`):
EP_PARSE sets `p_is_set_addr`, `addr_pending = 9`, `pend_addr = 1`
(`ext/usb_uart.sv:1294`–`1296`) → EP_SETUP routes to action 1 → EP_ARM
goes straight to EP_STATUS_I (no data stage). The host's IN token gets the
zero-length DATA1 status packet (`ext/usb_uart.sv:1426`); when the host
ACKs it, EP_WAIT_ACK applies `dev_addr <= 9` and returns to EP_IDLE
(`ext/usb_uart.sv:1389`).

**Control write with data — SET_LINE_CODING** (`21 20 00 00 00 00 07 00`):
EP_ARM sets `tog = 1` and goes to EP_RCV_DATA — the *host's* data packet
must be DATA1, and `rcd_acc_r` checks exactly that (`ext/usb_uart.sv:1195`).
The 7 bytes all live in `dt_head` (bytes 0–7 of the payload are flopped by
`usb_rx_packet` for exactly this consumer), so a registered capture pulse
`lc_cap_r` copies them into `line_coding` one clock after the accept
(`ext/usb_uart.sv:1161`). Having received `min(wLength, 7)` bytes
(`expect_len`, set in EP_PARSE), the FSM ACKs and
finishes with the device-sent status ZLP (EP_STATUS_I). A bad CRC or wrong
toggle is met with silence — the host retries the packet.

## Supported requests

Everything is decoded in EP_PARSE from `rq0`/`rq1` (and `rq2`/`rq3` where
noted), `ext/usb_uart.sv:1274`–`1299`:

| Request | Match | Data stage | Result |
|---------|-------|------------|--------|
| GET_DESCRIPTOR(device) | `rq1=06`, `rq3=01` | IN, `min(wLength,18)` B | ROM page 0 |
| GET_DESCRIPTOR(config) | `rq1=06`, `rq3=02` | IN, `min(wLength,75)` B | ROM page 1 (2 packets) |
| GET_DESCRIPTOR(string 0/1/2) | `rq1=06`, `rq3=03`, `rq2=0/1/2` | IN, `min(wLength, 4/10/32)` B | ROM pages 2–4 |
| GET_STATUS(device) | `rq1=00`, `rq0=80` | IN, `min(wLength,2)` B | two `0x00` bytes (`sel 6`) |
| SET_ADDRESS | `rq1=05` | none | `dev_addr <= rq2[6:0]` at status |
| SET_CONFIGURATION(0/1) | `rq1=09`, `rq2=00/01` | none | `configured` at status |
| SET_LINE_CODING | `rq1=20`, `rq0=21` | OUT, ≤ 7 B | 7 bytes → `line_coding` |
| GET_LINE_CODING | `rq1=21`, `rq0=A1` | IN, `min(wLength,7)` B | `sel 5`, from registers |
| SET_CONTROL_LINE_STATE | `rq1=22`, `rq0=21` | none | ACK only; DTR/RTS bits ignored |

Anything else — an unknown `bRequest`, a string index > 2, GET_CONFIGURATION
(not implemented here) — decodes to `p_valid = 0`, routes to `EP_STALL`
(`ext/usb_uart.sv:1313`), and the device answers every IN/OUT token with a
STALL handshake (`ext/usb_uart.sv:1457`) until the next SETUP clears it.
That is the protocol-legal way to say "I don't do that": the host logs it
and moves on.

`line_coding` packs the 7 CDC bytes as `dwDTERate[31:0]` (little-endian
baud), `bCharFormat`, `bParityType`, `bDataBits` — byte *i* at
`line_coding[8*i +: 8]`, defaulting to 115200 8N1 (`lc_default`,
`ext/usb_uart.sv:1097`).

## The address latch

The single most timing-sensitive rule in enumeration: **SET_ADDRESS takes
effect only after its status stage completes**. The host sends the
SET_ADDRESS SETUP to address 0 and immediately follows with the status IN
token — still to address 0. If `dev_addr` changed at request-decode time,
that status token would no longer match and the transfer would hang.

So EP_PARSE parks the value in `addr_pending` with a one-bit ownership tag
`pend_addr` ("*this* transfer is a SET_ADDRESS", `ext/usb_uart.sv:1050`),
and the FSM applies it only at its own transfer's completion — in
EP_WAIT_ACK when the status ZLP is ACKed (`ext/usb_uart.sv:1389`) or in
EP_STATUS_O when the host's ZLP is accepted (`ext/usb_uart.sv:1447`).
`cfg_pending`/`pend_cfg` work identically for SET_CONFIGURATION.

The tags are not decoration. Every request latches `addr_pending <=
rq2[6:0]`, including GET_DESCRIPTOR — whose `wValue` low byte is 0. At a
GET_DESCRIPTOR's status stage, `pend_addr` is 0, so the 0 parked value is
*not* applied; without the tag, every control read would wipe `dev_addr`
back to 0 mid-enumeration (the comment at `ext/usb_uart.sv:1046` records
exactly this bug).

The token router in `usb_uart` compares every token's address field against
`dev_addr` (`ext/usb_uart.sv:1786`), so the instant the latch updates, EP0
answers only to its new name.

## Descriptor ROM reads

`usb_ep0` does not store descriptor bytes. When EP_SEND arms a data packet
it outputs `resp_sel` (which descriptor) and `resp_base` (byte offset
within the transfer, 0 for EP0); the descriptor *contents* are pulled
byte-by-byte by the tx PHY from a 256×8 block-RAM ROM inside `usb_uart`
(`desc_rom`, `ext/usb_uart.sv:1973`):

- a `rom_page()` function maps `sel` to the ROM page start — device@0,
  config@18, string0@93, string1@97, string2@107 (`ext/usb_uart.sv:2244`);
- at `ep0_resp_req` time `usb_uart` latches
  `rom_base_r = rom_page(resp_sel) + resp_base` (`ext/usb_uart.sv:2261`);
- as `usb_tx_phy` transmits, it presents each byte index on
  `tx_addr_next`; the top adds it to `rom_base_r` and sync-reads the BRAM
  into `desc_q` (`ext/usb_uart.sv:2266`) — one register of latency, no
  LUT mux tree in the bit engine's path (see [usb-tx-phy.md](usb-tx-phy.md)).

Two "descriptors" are dynamic and bypass the ROM: `sel 5` (line coding)
reads the `line_coding` registers and `sel 6` (GET_STATUS) returns zeroes —
both through `ep0_small_q` (`ext/usb_uart.sv:2270`). The final producer mux
is three-way: serial snapshot / EP0 small regs / ROM (`tx_byte`,
`ext/usb_uart.sv:2278`).

The `desc_len()` function inside `usb_ep0` (`ext/usb_uart.sv:1137`) mirrors
the ROM's page sizes so the FSM can size the data stage:
`min(wLength, desc_len(sel))`. It must be kept in sync with the ROM — a
desync means over- or under-length transfers. Multi-packet data stages
fall out naturally: the 75-byte config descriptor exceeds EP0's 64-byte
max packet, so EP_SEND arms 64 bytes (`send_len`,
`ext/usb_uart.sv:1377`); on ACK, EP_WAIT_ACK advances `base += send_len`,
`remain -= send_len`, flips the toggle and re-enters EP_SEND for the last
11 bytes (`ext/usb_uart.sv:1397`). `last_r` (`remain <= 64`) tells
EP_WAIT_ACK that an ACK finishes the stage and it is time for EP_STATUS_O.

## Data toggles on EP0

EP0 keeps a single toggle bit `tog` (`ext/usb_uart.sv:1086`) with rules that
differ subtly from bulk ([usb-serial.md](usb-serial.md)):

- **SETUP resets it**: the `tk_setup` branch clears `tog` to 0 — the SETUP
  data packet is always DATA0 (`ext/usb_uart.sv:1240`).
- **The data stage starts at DATA1**: every action that arms a data stage
  sets `tog = 1` (`ext/usb_uart.sv:1331`). An IN data packet goes out as DATA1
  (`resp_pid = tog ? PID_DATA1 : PID_DATA0`); an OUT data packet *from the
  host* must arrive as DATA1 or it is ignored (`rcd_acc_r`).
- **Flip on ACK, never on retry**: EP_WAIT_ACK flips `tog` only when the
  host ACKs; if the host re-sends the IN token instead (it never saw its
  ACK), the FSM re-arms the *same* packet — same bytes, same PID
  (`ext/usb_uart.sv:1403`).
- **Status is always a zero-length DATA1**: device-sent in EP_STATUS_I
  (`ext/usb_uart.sv:1426`); the host-sent status ZLP in EP_STATUS_O is
  checked for length 0 and CRC only — deliberately lenient on PID
  (`sto_acc_r`, `ext/usb_uart.sv:1197`).

There is no persistent toggle *across* transfers, unlike bulk: each control
transfer starts fresh at DATA0, because the SETUP token says so.

## EP2 IN: the notification endpoint that never speaks

The configuration descriptor advertises a third endpoint besides EP0 and
EP1: **EP2 IN, interrupt type, 8-byte max, 16 ms interval** — the CDC
*notification* endpoint (ROM bytes `desc_rom[63..69]`, `ext/usb_uart.sv:2038`).
CDC uses it for SERIAL_STATE notifications ("carrier detected", etc.).

Linux's `cdc_acm` driver refuses to bind a CDC function that does not
declare a notification endpoint, so the descriptor must exist. But nothing
in CDC says the device must ever *send* a notification — so the core never
arms EP2. The router still decodes tokens to endpoint 2, and
`usb_uart` answers every one with a registered **NAK**
(`ep2_nak_req`, `ext/usb_uart.sv:1861`; the arbiter's PID default is
`PID_NAK`, `ext/usb_uart.sv:1879`). NAK means "no data right now, ask
again" — the host polls, gets NAK, moves on, and no data is ever lost. The
endpoint is a billboard, not a worker (file header,
`ext/usb_uart.sv:9`–`11`).

## Key takeaways

- A **control transfer** is SETUP (8-byte request in DATA0) → optional
  DATA stage → STATUS (a zero-length DATA1 in the opposite direction);
  USB 2.0 §8.5.5 is the whole plot of `usb_ep0`.
- **Enumeration** is just a sequence of these transfers: read descriptors,
  SET_ADDRESS, SET_CONFIGURATION, then CDC requests — and this core walks
  the whole list with one 11-state FSM.
- Deep request decoding is **pipelined one compare per clock**
  (EP_RCV_REQ → EP_PARSE → EP_SETUP → EP_ARM); wide decisions land on their
  own registers so no datapath enable sees a deep cone — the recurring
  48 MHz trick in this codebase.
- **SET_ADDRESS applies at status completion**, parked in `addr_pending`
  with a `pend_addr` ownership tag so unrelated transfers can't clobber it
  (`ext/usb_uart.sv:1389`).
- Descriptor bytes live in `usb_uart`'s **block-RAM ROM**; EP0 only emits
  `(resp_sel, resp_base)` and the tx PHY pulls bytes through
  `rom_page + base + tx_addr` as it transmits
  (`ext/usb_uart.sv:2244`–`2266`).
- EP0 toggles: SETUP forces DATA0, data stage starts DATA1, status is a
  DATA1 ZLP, and retries re-arm byte-identical packets.
- Unsupported requests **STALL until the next SETUP**; EP2 IN exists only
  in the descriptors (Linux `cdc_acm` requires it) and answers every poll
  with NAK, forever — which is perfectly legal.

Next: [`usb_uart`](usb-uart.md) — the token router, response arbiter and
ROM that connect this module to the wire, and to
[`usb_serial`](usb-serial.md) for the bulk data path.
