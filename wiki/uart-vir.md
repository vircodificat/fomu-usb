# `UartSender` + `UartReceiver` — 115200 8N1 on 48 MHz

**Source:** `src/uart.vir` (`UartSender` at `src/uart.vir:37`, `UartReceiver` at `src/uart.vir:114`) · **Back:** [Home.md](Home.md) · **Used by:** [echo-server.md](echo-server.md)

Two Virdant modules — one transmitter, one receiver — adapted from the virdant-examples 9600-baud UART; the only arithmetic change is the divisor. Frame: **idle high, start low, 8 data bits LSB first, stop high** (see [usb-primer.md](usb-primer.md) for the format picture, [system-overview.md](system-overview.md) for how they sit in the echo path).

## Baud: 417 cycles per bit

48_000_000 / 115200 = 416.67, so one bit time is **417 cycles** → 115107 baud, **−0.08% error**, far inside the ~2% tolerance UART tolerates.

## A first taste of Virdant

If this is your first `.vir` file:

- `reg baud_counter : Word[10] on clock { ... }` — a 10-bit register clocked on the implicit clock edge; `initial it <= 0w10` gives it a reset value (`0w10` = zero-width-10).
- `wire bit_time_done : Bit { it := ... }` — a named combinational net; `it` is the value being defined (like a self/lambda binding).
- `match state { case #Start => ... }` — a pattern match over enum values; `#Start` is the enum constructor tag.
- `enum type UartFsmState { Idle = 0b00w2 Start = 0b01 ... }` (`src/uart.vir:30-35`) — a 2-state enum shared by both modules; states are referred to as `#Idle`, `#Start`, `#Data`, `#Stop`.
- `case cond => value` — a condition chain (last true arm wins), used inside `it <= when { ... }` for next-state logic.

## `UartSender`

| Port | Dir | Meaning |
|---|---|---|
| `clock` | in | 48 MHz |
| `data` | in | byte to transmit |
| `data_valid` | in | 1-clk strobe; loads `data` when `ready` |
| `ready` | out | high when idle (`state == #Idle`) |
| `tx` | out | the serial line |

FSM **Idle → Start → Data → Stop**:

- `tx` is a `match` on `state` (`src/uart.vir:49-55`): high in Idle (idle marker) and Stop, low in Start (start bit), `data_buf[bit_idx]` during Data.
- `bit_time_done := baud_counter == 416w10`; **the counter is zeroed at every state change** (`src/uart.vir:69-74`), so every bit — start, each data bit, stop — is exactly 417 cycles. There is no cumulative drift.
- `data_valid && sender_idle` latches `data` into `data_buf` and moves Idle→Start (`src/uart.vir:88-93`); Data advances `bit_idx` 0..7 then Stop → Idle. Note `data` is sampled off `data_buf[bit_idx]` during Data, so the strobe-loaded byte survives even if `data` changes mid-frame.

## `UartReceiver`

| Port | Dir | Meaning |
|---|---|---|
| `clock` | in | 48 MHz |
| `uart_rx` | in | the serial line |
| `data` | out | completed byte (just `shift`) |
| `data_valid` | out | 1-clk strobe when `state == #Stop && bit_time_done` |

Differences from the sender:

- **Midpoint sampling.** On the line's falling edge, the baud counter is loaded with **208** — half a bit (`src/uart.vir:171`) — so the start bit is verified at its **middle**, and every following bit is sampled on that same phase, well away from transition edges.
- **Glitch rejection** (`src/uart.vir:140-144`): if the line bounced back high by the midpoint, it wasn't a real start bit → back to `#Idle` without touching `shift`.
- The counter counts **down** to 0 within each phase; each phase end reloads the full 416 for the next bit (`src/uart.vir:172-185`).
- **MSB-first shift-in of an LSB-first wire** (`src/uart.vir:202-207`): each captured bit enters at the MSB side, `word(uart_rx, it[8..1])`; after eight captures the byte sits with its **LSB in `shift[0]`** — exactly the order UART sent it.
- `data_valid` strobes at **Stop completion**, once the stop bit has been sampled high.

One nuance the file comment calls out (`src/uart.vir:162-164`): an *external* async UART would shorten the last data bit slightly so the stop bit is sampled safely inside its cell; on the internal loopback wire both sides share one clock, so periods are exact and no guard is needed.

## Key takeaways

- 417 cycles/bit at 48 MHz = 115107 baud, −0.08% error (tolerance is ~2%).
- Both FSMs walk Idle → Start → Data → Stop; sender zeroes its counter at every state change so bits are exactly 417 cycles — no drift.
- The receiver samples each bit at its **midpoint** (208-cycle half-bit from the falling edge) and rejects a start bit that bounced high by then (glitch rejection).
- Data shifts in MSB-first, so after 8 captures the LSB — sent first — sits in `shift[0]`; `data_valid` strobes at Stop.
- Virdant syntax in one line: `reg ... on clock` = clocked register, `wire { it := ... }` = named net, `match`/`case` = condition dispatch, `enum type` = tagged states like `#Start`.