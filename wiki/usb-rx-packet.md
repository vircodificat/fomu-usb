# `usb_rx_packet` — From Serial Bits to Packet Records

Upstream in the receive chain, [usb-rx-phy](usb-rx-phy.md) already did the hard analog-ish work: it recovered the 12 MHz bit stream from the D+/D− pads, dealt with NRZI decoding and bit unstuffing, and handed us a clean packet-boundaried bit stream. This page is about `usb_rx_packet` (`ext/usb_uart.sv:330`), the module that turns that stream into **packet records** — a token here, a data payload there, a handshake — which the router inside [usb-uart](usb-uart.md) then dispatches to the endpoints ([usb-ep0](usb-ep0.md), CDC-ACM, etc.).

If terms like "token" or "PID" feel shaky, skim [usb-primer](usb-primer.md) first; see [system-overview](system-overview.md) for where this module sits.

## Background: what does a USB packet look like on the wire?

A USB packet is, in wire order:

1. **SYNC** — 8 raw bits (`00000001` after unstuffing); only the final `1` marks byte alignment. We can ignore the rest.
2. **PID byte** — identifies the packet type.
3. **Payload** — depends on the type (see below).
4. **CRC** — 5 bits on tokens, 16 bits on data packets.
5. **EOP** — signalled electrically (SE0 for two bit times), not as a bit. The PHY raises `eop`.

Everything on USB is transmitted **LSB first** — the *lowest-numbered* bit of each field hits the wire first, and multi-byte fields come little-endian. This is why the shift registers below shift "up" (`{bit_i, sh[MSB-1:1]}`).

### PIDs: the `{~pid, pid}` encoding

The 8-bit PID byte is really a 4-bit type plus its own bitwise complement:

| PID byte | Meaning |
|---|---|
| `xxxx _xxxx` | low nibble = PID value, high nibble = ~PID |

So `OUT` is `0x1` → byte `E1`, `IN` = `0x9` → `D9`, `SOF` = `0x5` → `A5`, `DATA0` = `0x3` → `C3`, `ACK` = `0x2` → `D2`, etc. The complement makes the PID self-checking: if `pid[3:0] != ~pid[7:4]` the byte is corrupt. That single compare is exactly what `pid_bad_r` does (`ext/usb_uart.sv:438-440`), and it's also re-checked inline in `hs_cap` (`ext/usb_uart.sv:576-580`).

The low two bits of the PID value classify the packet:

| `pid[1:0]` | Class | Wire layout after the PID |
|---|---|---|
| `01` | **Token** | 11 bits token data + 5 bits CRC5 |
| `11` | **Data** | N payload bytes + 16 bits CRC16 |
| `00`, `10` | **Handshake** | nothing — PID only |

### Tokens, LSB first

A token (OUT/IN/SOF) carries:

- bits 0–6: **address** (7 bits),
- bits 7–10: **endpoint** (4 bits),
- bits 11–15: **CRC5** over the 11 data bits.

For SOF, those same 11 bits are an 11-bit **frame number** instead. Because bits arrive LSB-first, a single 11-bit shift register `tk_sh` (`ext/usb_uart.sv:607-608`) collects them; at EOP, `addr = tk_sh[6:0]`, `endp = tk_sh[10:7]` (`ext/usb_uart.sv:695-696`), and for SOF the whole thing is the frame (`ext/usb_uart.sv:687-690`).

## Ports

```systemverilog
module usb_rx_packet(
    input  wire         clk,        // 48 MHz system clock
    input  wire         rst,
    input  wire         bus_reset,  // from usb_rx_phy: long SE0 = bus reset
    input  wire         bit_i,      // decoded NRZI/unstuffed bit
    input  wire         bit_valid,  // 1-clk strobe: bit_i is a real data bit
    input  wire         sop,        // packet start strobe
    input  wire         eop,        // packet end strobe (SE0 detected)
    input  wire         pkt_err,    // PHY-level error (stuff/bit-error)
    // token record
    output reg          tk_valid,   // 1-clk pulse: a token arrived
    output reg  [3:0]   tk_pid,
    output reg  [6:0]   tk_addr,
    output reg  [3:0]   tk_endp,
    output reg  [10:0]  tk_frame,   // SOF only
    output reg          tk_crc_ok,  // CRC5 residual check
    // data record
    output reg          dt_valid,
    output reg  [3:0]   dt_pid,
    output reg  [6:0]   dt_len,     // payload bytes (CRC16 excluded)
    output reg  [63:0]  dt_head,    // payload bytes 0..7, flopped
    output reg  [7:0]   dt_rd_data, // sync read of dt_ram (1-clk latency)
    input  wire [5:0]   dt_rd_addr,
    output reg          dt_crc_ok,  // CRC16 residual check
    // handshake record
    output reg          hs_valid,
    output reg  [3:0]   hs_pid
);
```

The three `*_valid` outputs are **1-clk registered pulses**; the `*_pid`/`addr`/`endp`/etc. fields are held so the router can latch them on the pulse. A packet record is emitted only if the packet was *good* — bad packets are silent (no pulse, nothing downstream ever sees them).

## State machine

Three states (`ext/usb_uart.sv:355`): `ST_IDLE → ST_SYNC → ST_BODY`.

```
IDLE ──sop──▶ SYNC ──(skip 8 sync bits)──▶ BODY ──eop──▶ IDLE
```

### `ST_SYNC`

`sop` arrives, counters clear (`ext/usb_uart.sv:533-547`). The FSM then counts `nb` 0..7 over the sync byte; at `nb==7` it transitions to `ST_BODY` (`ext/usb_uart.sv:589-596`). The sync bits themselves are ignored — the PHY guarantees alignment, so this just re-establishes the bit index.

### `ST_BODY`, PID section (`sec == 0`)

Bit index `nb` restarts at 0 for the PID byte:

- `nb = 0..7` — the 8 PID bits shift into `pid_r` (`ext/usb_uart.sv:605-606`). At `nb==7`, the PID check decides the rest (`ext/usb_uart.sv:609-618`):
  - `pid[1:0] == 2'b11` → **data packet**: switch to the data section (`sec <= 1`), reset `n3`, park `nb` at 24 (done with it).
  - otherwise → **token or handshake**: `nb` continues into 8..23 for the token data and CRC5 bits. A handshake just stops mattering after `nb==7`.
- `nb = 8..18` — the 11 token bits shift up into `tk_sh`.
- `nb = 19..23` — the 5 CRC5 wire bits. These are **not** captured; they only feed the `usb_crc5` LFSR (see [usb-crc](usb-crc.md)).

### `ST_BODY`, data section (`sec == 1`)

Data packets have no fixed length — the end is only known at EOP. So the FSM just counts (`ext/usb_uart.sv:621-640`):

- `n3` counts bits within a byte (0..7, wraps naturally as a 3-bit register).
- `bshift` gathers 8 bits into a byte, LSB first: at `n3==7` the completing byte is `{bit_i, bshift[7:1]}` and `bc` increments.
- `bc` counts **every** byte, including the 2 trailing CRC16 wire bytes. That's why `dt_len = bc - 2` at EOP (`ext/usb_uart.sv:701`).

**Where do the payload bytes go?** Two places (`ext/usb_uart.sv:463-512`):

- `dt_ram` — a 64×8 **block RAM** holding payload bytes 0..63, written whenever a byte completes (`dt_wr`, `ext/usb_uart.sv:496-497`), addressed by `bc`. Downstream consumers read it through the `dt_rd_addr`/`dt_rd_data` port (sync read: present an address, get the byte one clock later).
- `dt_head` — 8 flops holding bytes 0..7 of the packet. Consumers that need several bytes *at once* (the EP0 request latch, SET_LINE_CODING — see [usb-ep0](usb-ep0.md), [usb-serial](usb-serial.md)) can't use a one-at-a-time RAM port, so the first 8 bytes are also kept in registers.

The block-RAM choice is a timing artifact: 512 payload flops would put a 64-way write decode and a 64:1 read mux on the 48 MHz critical path. Block RAM gives both for free (details in the comments at `ext/usb_uart.sv:465-487`).

## How the module decides packet type at EOP

The trick that makes this module simple: **you don't know a packet is valid until EOP, and EOP is ≥8 clocks after the last bit.** So instead of computing "is this packet good?" at EOP (a huge combinational cone over all the counters), the module precomputes **capture flags on the last relevant bit** and holds them:

- `tok_cap` — set on `nb==23` (last token bit) if PID low bits say token (`2'b01`) and nothing's gone wrong (`ext/usb_uart.sv:571-572`),
- `sof_cap` — same, plus PID value is `4'h5` (`ext/usb_uart.sv:573-575`),
- `hs_cap` — set on `nb==7` (last PID bit) for handshake PIDs, with the `{~pid,pid}` check inline (`ext/usb_uart.sv:576-580`),
- `dat_cap` — set when a data byte completes (`n3==7`) with byte count `bc` in 1..65 — at least one payload byte, and the 2 CRC bytes will make ≥2, and one more byte would still fit the 64-byte RAM (`ext/usb_uart.sv:581-583`).

The flags are mutually exclusive by construction (token/handshake live in the PID section, data in the data section), so at `eop` each strobe is just a 2-input AND: `tk_valid <= eop && tok_cap` etc. (`ext/usb_uart.sv:678-680`). The same EOP edge latches the fields — `tk_addr`/`tk_endp` from `tk_sh`, `dt_len` from `bc`, `hs_pid`/`tk_pid`/`dt_pid` from `pid_r[3:0]` (`ext/usb_uart.sv:682-708`).

> The registered helper dance (`q_pid_r`, `q_dat_r`, `dat_bc_ok_r`, `bc_is_66_r`, …) at `ext/usb_uart.sv:366-401` exists to keep those compares off the 48 MHz critical path — each wide comparison ends at its *own* register. Behaviorally you can ignore it: caps qualify with `!bad && !pkt_err && st==ST_BODY && <section>`, one clock early, with `pkt_err` re-checked inline.

## CRC checking by residual

No CRC comparator, no stored CRC bytes — just LFSRs (see [usb-crc](usb-crc.md) for how they work):

- `usb_crc5` advances on token bits `nb = 8..23` — the 11 data bits **plus the 5 CRC5 wire bits** (`ext/usb_uart.sv:409-415`). If you feed a CRC LFSR the data *and* the correct CRC, the LFSR's internal state lands on a fixed magic value — the **residual** (`5'h0C` here, `ext/usb_uart.sv:443`). Wrong data or a corrupted CRC lands anywhere else.
- `usb_crc16` advances on **every** data-section bit — payload plus the 16 CRC16 wire bits (`ext/usb_uart.sv:416-421`); residual `16'h800D` (`ext/usb_uart.sv:444`).

Both LFSRs are reset at `sop` (`rst | bus_reset | sop`), so each packet starts clean. `tk_crc_ok`/`dt_crc_ok` are just the registered residual compares, sampled at EOP when the LFSR has consumed the whole packet. Mid-packet LFSR states are harmless because they're only *read* at EOP.

Why a residual instead of "compute CRC, compare against the wire CRC"? Same answer as everywhere in this module: the 16-bit equality tree would otherwise sit in the timing cone of `dt_crc_ok` at exactly the wrong moment.

## Error handling

The sticky `bad` flag (`ext/usb_uart.sv:453-460`) is the whole error story. Any of these kills the packet:

- **`pkt_err`** — the upstream PHY ([usb-rx-phy](usb-rx-phy.md)) detected a bit-stuff violation or invalid line state mid-packet. Absorbed at `ext/usb_uart.sv:458` and also inline in the caps.
- **PID check failure** — `pid[3:0] != ~pid[7:4]`, registered as `pid_bad_r` then ORed into `bad` (`ext/usb_uart.sv:438-440`).
- **Over-length data packet** — a 64-byte payload + 2 CRC bytes is the legal max (`bc == 66`); a 67th byte means the packet is too big for the payload RAM and for USB. Registered as `bc66_hit_r` (`ext/usb_uart.sv:441-442`). Bytes past 64 aren't stored (`bc[6]` gates the RAM write, `ext/usb_uart.sv:500`), but since the packet is marked bad, `dt_valid` never fires and no consumer sees the truncation.
- **Bad CRC5 / CRC16** — different: these don't set `bad`. The packet *is* delivered (`tk_valid`/`dt_valid` pulse) with `tk_crc_ok`/`dt_crc_ok` low, letting the router do protocol-level retries (a data toggle / re-request in [usb-uart](usb-uart.md) / [usb-ep0](usb-ep0.md)).

`bad` clears at `sop` (`ext/usb_uart.sv:457`) — a new packet gets a fresh start — and the caps all stay 0, so at EOP nothing fires. Silent, exactly as USB wants: a broken packet on the bus is simply ignored.

`bus_reset` (long SE0 — see [usb-primer](usb-primer.md)) flushes everything, including partial packet state.

## FIFO interaction

`usb_rx_packet` itself never buffers — it emits **pulses**, one packet's worth at a time. Payload bytes for the CDC-ACM data path are read out of `dt_ram` by the router and pushed into the receive [usb-fifo](usb-fifo.md) one byte per clock after `dt_valid`.

## Key takeaways

- `usb_rx_packet` is a **packet assembler**: it consumes the PHY's bit stream (`bit_i`/`bit_valid`/`sop`/`eop`/`pkt_err`) and emits 1-clk **record pulses** (`tk_*`/`dt_*`/`hs_*`) for the router in [usb-uart](usb-uart.md).
- USB fields are **LSB first**; multi-bit fields are captured by shift-up registers and sliced afterwards (`addr = tk_sh[6:0]`, `endp = tk_sh[10:7]`).
- The PID byte is self-checking (`{~pid, pid}`); its low 2 bits classify the packet and select the body format (`ext/usb_uart.sv:612-618`).
- You only know a packet is good **at EOP**, so the module precomputes capture flags on the last relevant bit and holds them to EOP — this also keeps the 48 MHz critical paths shallow.
- CRC5/CRC16 are checked by **residual** (`5'h0C` / `16'h800D`): feed data + wire CRC through the LFSR and compare the final state (see [usb-crc](usb-crc.md)).
- Data-packet length isn't in the packet: bytes are counted through EOP, and the 2 trailing CRC16 bytes are subtracted (`dt_len = bc − 2`, `ext/usb_uart.sv:701`).
- Errors: stuff errors, bad PIDs, and over-length packets make the packet **silent** (`bad`, no pulse); bad CRC5/CRC16 deliver the record with `*_crc_ok` low and let the protocol layer retry.
- Payload storage is a 64×8 block RAM (`dt_ram`) plus 8 flops (`dt_head`) for consumers needing whole-word access — see [usb-fifo](usb-fifo.md) and [usb-ep0](usb-ep0.md).