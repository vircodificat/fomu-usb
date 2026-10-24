# USB Primer — taught through fomu-usb

You are a beginning digital designer: you know registers, FSMs, and
clocks, and you have heard of USB but never seen a wire. This page
teaches just enough USB protocol to read every module page in this wiki
— using *this project's* RTL as the running example. Deeper dives:
[usb-rx-phy.md](usb-rx-phy.md), [usb-rx-packet.md](usb-rx-packet.md),
[usb-ep0.md](usb-ep0.md), [usb-serial.md](usb-serial.md).

The system picture (where the core sits between host and application)
is [system-overview.md](system-overview.md).

## Electrical basics: what's on the connector

USB is a **differential pair**: D+ and D− (pins A1/A2 on Fomu,
`fomu.pcf`). Only the *difference* matters:

| D+ | D− | called | meaning |
|---|---|---|---|
| 0 | 1 | **J** | idle (for full-speed) |
| 1 | 0 | **K** | the other state |
| 0 | 0 | **SE0** | end-of-packet / reset |
| 1 | 1 | **SE1** | illegal (treated as error) |

- Full-Speed runs at **12 Mbit/s**. At the Fomu's 48 MHz clock that's
  exactly 4 cycles per wire bit (`usb::UsbUart` in `src/usb.vir`).
- The host keeps a 15 kΩ pull-down on both lines. The device declares
  itself **full-speed** by pulling **D+ high through 1.5 kΩ**
  (`usb_dp_pu`, pin A4). Pull-up on D+ ⇒ FS; on D− ⇒ low-speed. When
  the host sees its pull-down "lose" on D+, that's an **attach event**
  — this is why the pull-up enable (`usb_dp_pu`) comes straight from
  `usb::UsbPor.por_done` in `src/top.vir`: the POR
  (`usb::UsbPor`, `src/usb.vir`) counts 8_400_000 cycles at 48 MHz
  (~175 ms) before releasing it.
- Packets start with a **sync pattern** (J→K transitions) and end with
  **EOP**: both lines driven low (SE0) for ≥2 bit times, then idle J.
- While transmitting, the device drives the pair through the `SB_IO`
  tristate cells (`usb::UsbPads`, body in `ext/usb_pads.sv`); when
  receiving, the pads are released and the core just samples them.

## Bit-level coding: LSb first, NRZI, bit stuffing

**Bit order.** Every USB field goes on the wire **LSb first** — the
opposite of how you'd write it on paper. The byte `0xA5` =
`1010_0101` goes out as `1,0,1,0,0,1,0,1` (bit 0 first). All the
CRC LFSRs (`usb::Crc5`/`usb::Crc16` in `src/usb.vir`) therefore
advance one serial *wire* bit per clock — from the LSb side.

**NRZI.** The wire doesn't carry raw bits; it carries *transitions*.
A **0 bit = toggle** the line state (J↔K); a **1 bit = keep** it:

```
raw bits:   0  1  1  0  0  1  0
wire (J=--):K  K  K  J  K  K  J     (start from J: 0→toggle, 1→stay)
```

Why: long runs of identical raw bits would leave the line static and
the receiver's bit-phase would drift. NRZI guarantees transitions
*only* if 0s appear often enough — hence:

**Bit stuffing.** After **six consecutive 1 bits**, the transmitter
inserts a spurious 0 (a toggle), even if the next raw bit is also 0.
The receiver deletes any 0 following six 1s. Both directions are
implemented in `usb::RxPhy`/`usb::TxPhy` (`src/usb.vir`; see
[usb-rx-phy.md](usb-rx-phy.md)).

**Sync.** Before every packet the transmitter sends the pattern
`0000_0001` (raw). Through NRZI, starting from idle J, that's
`K J K J K J K K` — a transition on every one of the first seven
bit-times: the receiver's phase recovery locks onto it.
`usb::RxPhy` (`src/usb.vir`) detects it as a J→K edge from idle and
marks SOP.

Concrete full example — a **SETUP token to address 5, endpoint 0**:

- SETUP PID = `0b1101` (0x2D with check bits; see next section)
- raw sync + token = `00000001` + PID byte + `5` as 7 bits
  LSb-first (`1010000`) + endpoint `0000` + CRC5.
On the wire that's sync (NRZI-coded), then the PID's *low nibble first*,
then addr/EP, then CRC — all NRZI-encoded with stuffing — then SE0 for
EOP. `usb::RxPacket` reassembles exactly this stream
([usb-rx-packet.md](usb-rx-packet.md)).

## Packets: PIDs, tokens, data, handshakes

Every packet's first byte is a **PID** (packet ID) with an error-check
twist: the byte on the wire is `{~pid[3:0], pid[3:0]}` — the top four
bits are the complement of the bottom four. The receiver —
`usb::RxPacket` in `src/usb.vir` — checks `pid[3:0] == ~pid[7:4]`;
a mismatch is a corrupted PID and the packet is dropped.

The low 2 bits classify the packet (`usb::RxPacket`):

| pid[1:0] | type | examples |
|---|---|---|
| `01` | **token** (host→device) | `OUT`, `IN`, `SOF`, `SETUP` |
| `11` | **data** | `DATA0`, `DATA1` |
| `10` | **handshake** | `ACK`, `NAK`, `STALL` |
| `00` | special | PRE |

**Token packets** (host only) carry a routing header:

```
sync | PID | addr[6:0] | endp[3:0] | crc5[4:0] | EOP     (all LSb-first)
```

**Data packets** carry a payload:

```
sync | PID | payload bytes (LSb-first) | crc16 (low byte, then high) | EOP
```

**Handshake packets** are just `sync | PID | EOP` — one byte of
payload-free opinion (ACK/NAK/STALL).

## CRC5 and CRC16

Every token/data packet ends in a CRC — the receiver's last line of
defense against wire noise:

- **CRC5** (5 bits, tokens): polynomial x⁵+x²+1, seed `0x1F` —
  `usb::Crc5` in `src/usb.vir` (receive-side only: the device never
  transmits tokens).
- **CRC16** (16 bits, data): polynomial x¹⁶+x¹⁵+x²+1, seed `0xFFFF` —
  `usb::Crc16` checks on RX and generates the TX data CRC in
  `usb::TxPhy`.
- Transmitted bits are complemented **and** bit-reversed:
  `crc_wire[i] = ~crc[W-1-i]` (realized in `usb::TxPhy` by loading
  `crc ^ 0xFFFF` and shifting MSB-first).
- The receiver can't know the payload/CRC boundary before EOP, so it
  checks **by residual**: feed every post-PID bit — *including* the
  received CRC bits — through the same LFSR and compare the final
  state against the magic residual (`5'h0C` for CRC5, `16'h800D` for
  CRC16) — the residual compares in `usb::RxPacket` (`src/usb.vir`).

Full LFSR walkthrough: [usb-crc.md](usb-crc.md).

## Device addresses, endpoints, and EP0

A USB wire is a **shared bus** with one host as the only traffic
director. After enumeration the device has a 7-bit **address** (0-127;
0 is the un-enumerated default) and up to 16 **endpoints** per
direction. The address+endpoint fields in every token tell *which
device* and *which pipe inside it* the transaction targets.

- **EP0** exists in every device, is **control** type, and is where
  all enumeration happens ([usb-ep0.md](usb-ep0.md)).
- This device additionally has **EP1 OUT** and **EP1 IN**, both **bulk**
  (the CDC-ACM data pipe, 64-byte max packet), and a **notification
  EP2 IN** (interrupt) that exists in the descriptors only — Linux's
  `cdc_acm` driver requires it to bind, so `usb::UsbUart` NAKs every
  EP2 token forever rather than omitting it (`src/usb.vir`).

## Enumeration and control transfers

On attach the host resets the bus, assigns an address, reads
descriptors, and configures the device. Every step is a **control
transfer** on EP0: SETUP packet (8-byte request) → optional DATA stage
→ STATUS stage (USB 2.0 §8.5.5, §9). The 8-byte request is:

```
bmRequestType | bRequest | wValue(2) | wIndex(2) | wLength(2)
```

`wValue`/`wIndex` are LSb-first on the wire — a classic porting bug.

The sequence this core serves (`usb::Ep0` in `src/usb.vir`,
[usb-ep0.md](usb-ep0.md)):

1. `GET_DESCRIPTOR(Device)` — host reads the 18-byte device descriptor.
2. `SET_ADDRESS` — the new address is latched **at the status stage**
   (not when the request arrives — a classic gotcha, USB 2.0 §9.4.6).
   After this, the core's token gate (`tok_v_r` in `usb::UsbUart`,
   `src/usb.vir`) only accepts tokens whose address matches
   `ep0.dev_addr`.
3. More `GET_DESCRIPTOR`s (config, strings).
4. `SET_CONFIGURATION(1)` — the device becomes "configured" and EP1
   goes live; data toggles reset here.
5. CDC-ACM class requests (below).

## Configurations, interfaces, alternate settings

Descriptors form a tree: **device** → **configuration** → **interface**
→ **endpoint** descriptors. A configuration is a bundle of power/bandwidth
commitments; an interface is a functional unit (here: a CDC control
interface and a CDC data interface); **alternate settings** are
mutually-exclusive variants of the same interface (e.g. "on" vs "off"
in a network adapter). This device is the simplest legal shape:
**one configuration, two interfaces, no alternate settings** — the
CDC header/ACM descriptors hang off the control interface
([usb-ep0.md](usb-ep0.md) has the full table).

## CDC-ACM: USB's "serial port" class

**CDC** (Communications Device Class) is the spec framework;
**ACM** (Abstract Control Model) is the subclass that a host OS
exposes as a serial port (`/dev/ttyACM*`, "COM port").

- **Descriptors**: CDC header, call management, ACM, union descriptors
  on the control interface; two bulk endpoints on a data interface.
- **EP0 class requests** this core implements in `usb::Ep0`: exactly
  one — `SET_CONTROL_LINE_STATE` (DTR/RTS, bmRequestType 0x21,
  bRequest 0x22) — accepted.  Anything unrecognized → **STALL**
  (error handshake), including `SET_LINE_CODING`/`GET_LINE_CODING`:
  the core implements no line-coding request at all, so baud/parity
  bookkeeping lives entirely on the host side (the raw UART here is
  fixed at 115200 8N1 regardless).
- **EP2 IN** (interrupt/notification) is the *send-break / carrier
  change* channel. As above: in descriptors, never armed, always
  NAKed (`usb::UsbUart`).

The host's serial API (`termios`, 115200 8N1) terminates at the ACM
driver; the bytes after that are just bulk payloads on EP1 — which is
exactly the stream the echo application consumes
([system-overview.md](system-overview.md),
[usb-serial.md](usb-serial.md)).

## Bulk transfers, data toggles, ACK/NAK

**Bulk** = "error-checked, bandwidth-flexible": no reserved bandwidth,
the host polls when it feels like it.

A bulk **OUT** (host→device):

```
host: OUT token(addr,ep)     device: (nothing to do)
host: DATA0/DATA1 payload    device: ACK or NAK
```

A bulk **IN** (device→host):

```
host: IN token(addr,ep)      device: DATA0/DATA1 payload (or NAK)
                             host:   ACK (or nothing → device retries)
```

**Data toggles.** Every payload carries `DATA0` or `DATA1` in its PID,
alternating per packet per direction. Sender flips its toggle only
after an ACK; receiver accepts a packet only if the toggle matches
expectation — a mismatched (duplicate) packet is a retry and is
dropped. Toggles reset to "expect DATA0" on bus reset and
SET_CONFIGURATION (`usb::Serial` in `src/usb.vir`; see
[usb-serial.md](usb-serial.md)).

**ACK/NAK flow control — the important design lesson.** USB has no
"ready" wire; the *protocol* is the flow control:

1. The echo application's 16-byte `EchoFifo` fills up ⇒ its `room`
   output drops (low once the FIFO holds 12 bytes — `usb::EchoServer`
   in `src/echo.vir`, `\echo::EchoFifo` in `ext/echo_fifo.sv`).
2. `room` feeds back as `rx_ready` into the core (`src/top.vir`):
   the router stops popping bytes out of the rx FIFO.
3. The rx FIFO stays non-empty ⇒ on the next OUT token, `usb::Serial`
   answers **NAK** (`src/usb.vir` OUT path).
4. The host sees NAK, waits, retries the *same* OUT token + same
   toggle + same data — nothing is lost, everything is deferred.

The tx direction mirrors this: empty tx FIFO ⇒ NAK on IN tokens. So
NAK here is not an error — it is **backpressure**, and the whole
device never drops or duplicates a byte. Full trace:
[usb-serial.md](usb-serial.md) and [echo-server.md](echo-server.md).

## Key takeaways

- USB FS = 12 Mbit/s on a differential pair; a device attaches by
  pulling D+ high through 1.5 kΩ (Fomu pin A4) after its power-on
  reset (~175 ms = 8_400_000 cycles in `usb::UsbPor`, `src/usb.vir`).
- Wire format: everything **LSb-first**, NRZI-coded (0=toggle,
  1=hold), stuffed (0 after six 1s), wrapped in sync + EOP.
- Three packet classes — token (routing), data (payload+CRC), and
  handshake (one byte of opinion) — selected by the PID's low 2 bits,
  with `{~pid,pid}` self-check bytes.
- CRC5 (tokens) / CRC16 (data), checked **by residual** —
  `5'h0C` / `16'h800D` (the residual compares in `usb::RxPacket`).
- Enumeration = control transfers on EP0: descriptors →
  SET_ADDRESS (latched at status) → SET_CONFIGURATION →
  the one CDC-ACM class request implemented, SET_CONTROL_LINE_STATE
  (anything else STALLs).
- Bulk pipes use DATA0/1 toggles + ACK/NAK as their entire flow
  control; this project rides NAK all the way from the 12-of-16
  threshold on the echo's 16-byte FIFO back to the host's retry
  loop.

*Next: [usb-rx-phy.md](usb-rx-phy.md) to see the wire decoding in RTL,
or [usb-ep0.md](usb-ep0.md) for the enumeration state machine.*