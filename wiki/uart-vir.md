# `UartSender` + `UartReceiver` — 115200 8N1 on 48 MHz

**Source:** `src/uart.vir` (`mod UartSender`, `mod UartReceiver`) · **Back:** [Home.md](Home.md) · **Used by:** [echo-server.md](echo-server.md)

Two Virdant modules — one transmitter, one receiver — sharing a single FSM-state enum. Frame: **idle high, start low, 8 data bits LSB first, stop high** (see [usb-primer.md](usb-primer.md) for the format picture, [system-overview.md](system-overview.md) for how they sit in the echo path).

## Baud: 417 cycles per bit

48_000_000 / 115200 = 416.67, so one bit time is **417 cycles** → 115107 baud, **−0.08% error**, far inside the ~2% tolerance UART tolerates.

## A first taste of Virdant

If this is your first `.vir` file:

- `reg baud_counter : Word[10] on clock { ... }` — a 10-bit register clocked on the implicit clock edge; `initial it <= 0w10` gives it a reset value (`0w10` = zero-width-10).
- `wire bit_time_done : Bit { it := ... }` — a named combinational net; `it` is the value being defined (like a self/lambda binding).
- `match state { case #Start => ... }` — a pattern match over enum values; `#Start` is the enum constructor tag.
- `enum type UartFsmState { Idle = 0b00w2 Start = 0b01 ... }` — a four-state enum encoded in 2 bits, shared by both modules; states are referred to as `#Idle`, `#Start`, `#Data`, `#Stop`.
- `case cond => value` — a condition chain (last true arm wins), used inside `it <= when { ... }` for next-state logic.

## `UartSender`

| Port | Dir | Meaning |
|---|---|---|
| `clock` | in | 48 MHz |
| `data` | in | byte to transmit |
| `data_valid` | in | 1-clk strobe; only sampled while `ready` is high — at that edge `data` is latched and the frame begins. Strobes while busy are ignored; there is no input buffering. |
| `ready` | out | high exactly while idle (`state == #Idle`); rises again on the first cycle after the stop bit, so frames can be sent back to back |
| `tx` | out | the serial line |

FSM **Idle → Start → Data → Stop**:

- `tx` is a `match` on `state`: high in Idle (idle marker) and Stop, low in Start (start bit), `data_buf[bit_idx]` during Data.
- `bit_time_done := baud_counter == 416w10`; the counter counts up 0..416 and reloads to 0 on the edge that ends each phase (and is held at 0 in Idle), so every bit — start, each data bit, stop — is exactly 417 cycles. There is no cumulative drift.
- `data_valid && sender_idle` latches `data` into `data_buf` and moves Idle→Start (the start bit is driven the next cycle); Data advances `bit_idx` 0..7, then Stop → Idle. Note `tx` samples `data_buf[bit_idx]` during Data, so the strobe-latched byte survives even if `data` changes mid-frame.

## `UartReceiver`

| Port | Dir | Meaning |
|---|---|---|
| `clock` | in | 48 MHz |
| `uart_rx` | in | the serial line |
| `data` | out | completed byte (just `shift`); stable from the last data-bit sample through the `data_valid` pulse |
| `data_valid` | out | 1-clk pulse in the final cycle of the Stop phase (roughly the stop bit's center) |

Differences from the sender:

- **Midpoint sampling.** On the first clock edge where the line reads low (the falling start-bit edge), the counter loads **208** and the FSM moves to Start. The Start phase runs **209 cycles** (208 counting down to 0) — half of a 417-cycle bit — so the start bit is re-checked at its **middle**, about 209 cycles after the falling edge. Each following bit is sampled 417 cycles after the previous sample, i.e. every bit at its center, well away from transition edges.
- **Glitch rejection**: if the line has bounced back high by that midpoint check, it wasn't a real start bit → back to `#Idle` without touching `shift`.
- The counter counts **down** to 0 within each phase; when Start or Data ends it reloads 416 for the next 417-cycle phase, and Stop also runs 417 cycles before reloading 0 for Idle.
- **MSB-first shift-in of an LSB-first wire**: at the end of each Data phase the captured bit enters at the MSB side, `word(uart_rx, it[8..1])`; after eight captures the byte sits with its **LSB in `shift[0]`** — exactly the order UART sent it. The byte then holds through the Stop phase, which is when `data_valid` pulses.
- **No stop-bit check.** `data_valid` pulses at the end of the Stop phase, but the stop bit's **level is not checked** — Stop is a plain 417-cycle wait.
- **No input synchronizer.** `uart_rx` is sampled directly, with no synchronizer flip-flops. All periods assume `uart_rx` is in the same clock domain as `clock` — true on the top-level loopback wire (see [top-vir.md](top-vir.md)), where sender and receiver share one clock. An externally driven `rxd` would be unsynchronized.

## Key takeaways

- 417 cycles/bit at 48 MHz = 115107 baud, −0.08% error (tolerance is ~2%).
- Both FSMs walk Idle → Start → Data → Stop; the sender counts up with its counter zeroed at every phase change, so bits are exactly 417 cycles — no drift.
- The receiver counts down per phase and samples each bit at its **center** (start bit checked ~209 cycles after the falling edge), rejecting a start bit that bounced high by then (glitch rejection).
- `data_valid` pulses at the end of Stop; the stop bit level is **not** checked, and `uart_rx` has **no input synchronizer** (exact on the same-clock loopback, unsynchronized for an external source).
- Data shifts in MSB-first, so after 8 captures the LSB — sent first — sits in `shift[0]`.
- Virdant syntax in one line: `reg ... on clock` = clocked register, `wire { it := ... }` = named net, `match`/`case` = condition dispatch, `enum type` = tagged states like `#Start`.
