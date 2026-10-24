# `usb::RxPacket` — From Serial Bits to Packet Records

Upstream in the receive chain, [usb-rx-phy](usb-rx-phy.md) already did the hard analog-ish work: it recovered the 12 MHz bit stream from the D+/D− pads, dealt with NRZI decoding and bit unstuffing, and handed us a clean packet-boundaried bit stream. This page is about `usb::RxPacket` (`src/usb.vir`), the mod that turns that stream into **packet records** — a token here, a data payload there, a handshake — which the router inside [usb-uart](usb-uart.md) then dispatches to the endpoints ([usb-ep0](usb-ep0.md), [usb-serial](usb-serial.md), etc.). Its payload RAM is a small helper mod, `usb::RxRam`, in the same file.

If terms like "token" or "PID" feel shaky, skim [usb-primer](usb-primer.md) first; see [system-overview](system-overview.md) for where this mod sits.

## Background: what does a USB packet look like on the wire?

A USB packet is, in wire order:

1. **SYNC** — 8 raw bits (`00000001` after unstuffing); only the final `1` marks byte alignment. We can ignore the rest.
2. **PID byte** — identifies the packet type.
3. **Payload** — depends on the type (see below).
4. **CRC** — 5 bits on tokens, 16 bits on data packets.
5. **EOP** — signalled electrically (SE0 for two bit times), not as a bit. The PHY raises `eop`.

Everything on USB is transmitted **LSB first** — the *lowest-numbered* bit of each field hits the wire first, and multi-byte fields come little-endian. This is why the shift registers below shift toward bit 0: each new bit enters at the MSB (`word(bit_i, sh[8..1])`), so by the end of the field the first-arrived bit has walked down into the LSB position.

### PIDs: the `{~pid, pid}` encoding

The 8-bit PID byte is really a 4-bit type plus its own bitwise complement:

| PID byte | Meaning |
|---|---|
| `xxxx _xxxx` | low nibble = PID value, high nibble = ~PID |

So `OUT` is `0x1` → byte `E1`, `IN` = `0x9` → `D9`, `SOF` = `0x5` → `A5`, `DATA0` = `0x3` → `C3`, `ACK` = `0x2` → `D2`, etc. The complement makes the PID self-checking: if `pid[3:0] != ~pid[7:4]` the byte is corrupt. That single compare is what the `pid_bad_r` register computes (as `(pid_new[4..0] ^ pid_new[8..4]) != 15w4`), and it is also re-checked inline in `hs_cap` (see below — `bad` is not visible yet on that edge).

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

For SOF, those same 11 bits are an 11-bit **frame number** instead. Because bits arrive LSB-first, a single 11-bit shift register `tk_sh` collects them; at EOP, `addr = tk_sh[6:0]`, `endp = tk_sh[10:7]`, and for SOF the whole register is the frame number.

## Ports

```
mod RxPacket {
    incoming clk : Clock        // 48 MHz: 4 clks per USB FS bit
    incoming rst : Bit          // synchronous core reset
    incoming bus_reset : Bit    // from usb::RxPhy: long SE0 = bus reset
    incoming bit_i : Bit        // decoded NRZI/unstuffed bit
    incoming bit_valid : Bit    // 1-clk strobe: bit_i is a real data bit
    incoming sop : Bit          // packet start strobe
    incoming eop : Bit          // packet end strobe (SE0 detected)
    incoming pkt_err : Bit      // PHY-level sticky error (stuff/line state)
    // token record
    outgoing tk_valid  : Bit     // 1-clk pulse: a token arrived
    outgoing tk_pid    : Word[4]
    outgoing tk_addr   : Word[7]
    outgoing tk_endp   : Word[4]
    outgoing tk_frame  : Word[11]  // SOF only
    outgoing tk_crc_ok : Bit       // CRC5 residual check
    // data record
    outgoing dt_valid   : Bit
    outgoing dt_pid     : Word[4]
    outgoing dt_len     : Word[7]  // payload bytes (CRC16 excluded)
    outgoing dt_head    : Word[64] // payload bytes 0..7, flopped
    outgoing dt_rd_data : Word[8]  // sync read of the payload RAM (1-clk latency)
    incoming  dt_rd_addr : Word[6]
    outgoing dt_crc_ok  : Bit      // CRC16 residual check
    // handshake record
    outgoing hs_valid : Bit
    outgoing hs_pid   : Word[4]
}
```

The three `*_valid` outputs are **1-clk registered pulses**; the `*_pid`/`addr`/`endp`/etc. fields hold their values between packets, so the router and the endpoints can read them on the pulse or a clock or two later. A packet record is emitted only if the packet was *good* — bad packets are silent (no pulse, nothing downstream ever sees them). The one partial exception is CRC: a packet that fails CRC5/CRC16 *is* delivered, but with `*_crc_ok` low (see [Error handling](#error-handling)).

## State machine

Three states (the `RxPktState` enum): `Idle → Sync → Body`.

```
IDLE ──sop──▶ SYNC ──(skip 8 sync bits)──▶ BODY ──eop──▶ IDLE
```

### `ST_SYNC`

`sop` arrives and clears the counters (`nb`, `n3`, `bc`, `sec`, `pid_r`, `tk_sh`, `bshift`), the sticky `bad` flag and both CRC LFSRs. The FSM then counts `nb` 0..7 over the sync byte; at `nb==7` (on a clean `bit_valid`) it transitions to `Body`. The sync bits themselves are ignored — the PHY guarantees alignment, so this just re-establishes the bit index.

### `ST_BODY`, PID section (`sec == 0`)

Bit index `nb` restarts at 0 for the PID byte:

- `nb = 0..7` — the 8 PID bits shift into `pid_r`, which then stays **frozen** for the rest of the packet (it classifies the packet at EOP; the token/CRC bits never touch it). At `nb==7`, the PID check decides the rest:
  - `pid[1:0] == 2'b11` → **data packet**: the `sec` flag switches to the data section, `n3` resets, and `nb` parks at 24 (done with it).
  - otherwise → **token or handshake**: `nb` continues into 8..23 for the token data and CRC5 bits. A handshake just stops mattering after `nb==7`.
- `nb = 8..18` — the 11 token bits shift into `tk_sh`.
- `nb = 19..23` — the 5 CRC5 wire bits. These are **not** captured; they only feed the `usb::Crc5` LFSR (see [usb-crc](usb-crc.md)).

### `ST_BODY`, data section (`sec == 1`)

Data packets have no fixed length — the end is only known at EOP. So the FSM just counts:

- `n3` counts bits within a byte (0..7, wrapping as a 3-bit register).
- `bshift` gathers 8 bits into a byte: at `n3==7` the completing byte is `dt_byte_new = {bit_i, bshift[7:1]}` and the byte-complete strobe `dt_wr` fires.
- `bc` counts **every** byte, including the 2 trailing CRC16 wire bytes. That's why `dt_len = bc - 2` at EOP.

**Where do the payload bytes go?** Two places:

- the **payload RAM** (`usb::RxRam`) — a 64×8 iCE40 block RAM (`SbRam40_4k`) holding payload bytes 0..63, written whenever a byte completes (`dt_wr`), addressed by `bc`. Downstream consumers read it through the `dt_rd_addr`/`dt_rd_data` port (registered read: present an address, get the byte one clock later). Writes beyond byte 63 are dropped: the instance's write enable is `dt_wr && !bc[6]` — over-length packets go bad anyway, so no consumer ever sees the truncation (see [Error handling](#error-handling)).
- `dt_head` — payload bytes 0..7 of the packet, flopped. Consumers that need several bytes *at once* (the EP0 request latch for the 8-byte SETUP packet — see [usb-ep0](usb-ep0.md)) can't use the one-at-a-time RAM port, so the head window is also kept in registers. Concretely these are **eight separate byte registers** `head0..head7`: each one's enable is its own shallow decode `dt_wr && (bc == k)` (with k < 8, so the equality subsumes the head-window test) running *in parallel* with the `dt_wr` cone rather than serially behind it. `dt_head` is the concatenation `word(head7, …, head0)`.

The block-RAM choice is a timing artifact: 512 payload flops would put a 64-way write decode and a 64:1 read mux on the 48 MHz critical path. One `SbRam40_4K` cell gives a single write port (no decode tree) and a registered read port for free — and because the cell is instantiated explicitly (`usb::RxRam`), yosys maps it directly, no inference needed. A nicety of the flopped head: a bad packet's payload is still written (RAM and head); `dt_valid` just never fires for it, so no consumer notices.

## How the module decides packet type at EOP

The trick that makes this mod simple: **you don't know a packet is valid until EOP, and EOP is ≥8 clocks after the last bit.** So instead of computing "is this packet good?" at EOP (a huge combinational cone over all the counters), the mod precomputes **capture flags on the last relevant bit** and holds them (no `bit_valid` clks fire during SE0/EOP, so a hold is safe; any further decoded bit re-evaluates them, so over-long packets stay silent):

- `tok_cap` — set on `nb==23` (last token bit) if the PID low bits say token (`2'b01`) and the shared qualify `q_pid_r` holds,
- `sof_cap` — same, plus the PID value is `4'h5` (the frame-number case),
- `hs_cap` — set on `nb==7` (last PID bit) for handshake-class PIDs (low bits `00` or `10`), with the `{~pid, pid}` check inline (at that edge `bad` isn't set yet — it arrives one clk later via `pid_bad_r`),
- `dat_cap` — set when a data byte completes (`n3==7`) with the registered byte-count admission `dat_bc_ok_r` (see below).

Every cap is qualified by the registered shared flags `q_pid_r` / `q_dat_r` (PID section vs data section of `ST_BODY`, with `!bad` and `!pkt_err` folded in, one clock early — free, because the caps are only read at EOP), and a same-cycle `pkt_err` is re-checked inline in every cap.

The flags are mutually exclusive by construction (token/handshake live in the PID section, data in the data section), so each output strobe is just a 2-input AND of registers: `tk_valid_r <= eop && tok_cap`, `dt_valid_r <= eop && dat_cap`, `hs_valid_r <= eop && hs_cap`. Reading the caps *at* the `eop` edge is safe — the same edge clears them, and nonblocking semantics see the pre-edge values. The same EOP edge latches the fields: `tk_addr`/`tk_endp` from `tk_sh` (non-SOF tokens only — SOF leaves them holding their previous value), `tk_frame` from the whole `tk_sh` (SOF only), `dt_len` from `bc` (clamped at 0), and `hs_pid`/`tk_pid`/`dt_pid` from the low nibble of `pid_r`.

> The registered helper dance (`q_pid_r`, `q_dat_r`, `dat_bc_ok_r`, `bc_is_66_r`, `crc5_res_r`, `crc16_res_r`, …) exists to keep those wide compares off the 48 MHz critical path — each comparison ends at its *own* register. Behaviorally you can ignore it: caps qualify with `!bad && !pkt_err && ST_BODY && <section>`, one clock early, with `pkt_err` re-checked inline. `dat_bc_ok_r` admits a data packet when the byte count is non-zero and the over-length hit hasn't fired — sampled one bit early, that means packets ending with 2..66 total bytes: a zero-length payload plus its 2 CRC bytes (a ZLP) at the bottom, the legal-max 64-byte payload at the top.

## CRC checking by residual

No CRC comparator, no stored CRC bytes — just LFSRs (the serial `usb::Crc5` / `usb::Crc16` mods, see [usb-crc](usb-crc.md) for how they work):

- `crc5` advances on token bits `nb = 8..23` — the 11 data bits **plus the 5 CRC5 wire bits** (and deliberately *not* gated by `pkt_err`: a bad packet's residual is never sampled). If you feed a CRC LFSR the data *and* the correct CRC, the LFSR's internal state lands on a fixed magic value — the **residual** (`5'h0C` here). Wrong data or a corrupted CRC lands anywhere else.
- `crc16` advances on **every** data-section bit — payload plus the 16 CRC16 wire bits; residual `16'h800D`.

Both LFSRs are reset at `sop` (`rst | bus_reset | sop`), so each packet starts clean. `tk_crc_ok`/`dt_crc_ok` are the registered residual compares (`crc5_res_r` / `crc16_res_r`), sampled at EOP when the LFSR has consumed the whole packet. Mid-packet LFSR states are harmless because they're only *read* at EOP.

Why a residual instead of "compute CRC, compare against the wire CRC"? Same answer as everywhere in this mod: the 16-bit equality tree would otherwise sit in the timing cone of `dt_crc_ok` at exactly the wrong moment.

## Error handling

The sticky `bad` flag is the whole error story. Any of these kills the packet:

- **`pkt_err`** — the upstream PHY ([usb-rx-phy](usb-rx-phy.md)) saw a bit-stuff violation or illegal line state mid-packet. Absorbed inline into `bad` and re-checked in the caps.
- **PID check failure** — `pid[3:0] != ~pid[7:4]`, registered one clk ahead as `pid_bad_r`, then ORed into `bad`.
- **Over-length data packet** — a 64-byte payload + 2 CRC bytes is the legal max (total byte count 66); a 67th byte means the packet is too big for the payload RAM and for USB. Registered as `bc66_hit_r` (fired when a byte completes with `bc == 66`). Bytes past 63 aren't stored (`!bc[6]` gates the RAM write), but since the packet is marked bad, `dt_valid` never fires and no consumer sees the truncation.
- **Bad CRC5 / CRC16** — different: these don't set `bad`. The packet *is* delivered (`tk_valid`/`dt_valid` pulse) with `tk_crc_ok`/`dt_crc_ok` low, and the consumers do the protocol-level triage: the token router drops bad-CRC tokens outright, [usb-serial](usb-serial.md) rejects a bad-CRC bulk-OUT packet with a NAK (and its data-toggle logic re-requests), and [usb-ep0](usb-ep0.md) only accepts its SETUP/ZLP packets with `dt_crc_ok` high.

`bad` clears at `sop` — a new packet gets a fresh start — and the caps all stay low, so at EOP nothing fires. Silent, exactly as USB wants: a broken packet on the bus is simply ignored.

`bus_reset` (long SE0 — see [usb-primer](usb-primer.md)) flushes everything, including partial packet state.

## FIFO interaction

`usb::RxPacket` itself never buffers beyond one packet — it emits **pulses**, one packet's worth at a time. For the CDC-ACM data path, [usb-serial](usb-serial.md) accepts a bulk-OUT packet only if it has good CRC, the expected data toggle and room (`rxfifo` count + `dt_len` ≤ 64); the rx push in [usb-uart](usb-uart.md) then streams the accepted payload out of the payload RAM into the receive [usb-fifo](usb-fifo.md), one byte per clock: the push index `push_i` drives `dt_rd_addr`, the RAM's registered read hands back the byte one clock later, and a countdown counter (`push_rem`, seeded with `dt_len`) terminates the run — a ZLP (`dt_len == 0`) pushes nothing at all.

## Key takeaways

- `usb::RxPacket` is a **packet assembler**: it consumes the PHY's bit stream (`bit_i`/`bit_valid`/`sop`/`eop`/`pkt_err`) and emits 1-clk **record pulses** (`tk_*`/`dt_*`/`hs_*`) for the router in [usb-uart](usb-uart.md).
- USB fields are **LSB first**; multi-bit fields are captured by shift-toward-bit-0 registers and sliced afterwards (`addr = tk_sh[6:0]`, `endp = tk_sh[10:7]`).
- The PID byte is self-checking (`{~pid, pid}`); its low 2 bits classify the packet and select the body format.
- You only know a packet is good **at EOP**, so the mod precomputes capture flags on the last relevant bit and holds them to EOP — this also keeps the 48 MHz critical paths shallow.
- CRC5/CRC16 are checked by **residual** (`5'h0C` / `16'h800D`): feed data + wire CRC through the LFSR and compare the final state (see [usb-crc](usb-crc.md)).
- Data-packet length isn't in the packet: bytes are counted through EOP, and the 2 trailing CRC16 bytes are subtracted (`dt_len = bc − 2`).
- Errors: stuff errors, bad PIDs, and over-length packets make the packet **silent** (`bad`, no pulse); bad CRC5/CRC16 deliver the record with `*_crc_ok` low and let the protocol layer retry.
- Payload storage is a 64×8 block RAM (`usb::RxRam`) plus eight flopped head bytes (`head0..head7` → `dt_head`) for consumers needing whole-word access — see [usb-fifo](usb-fifo.md) and [usb-ep0](usb-ep0.md).
