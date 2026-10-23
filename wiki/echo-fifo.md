# `\echo::EchoFifo` — the 16×8 circular FIFO

**Source:** `ext/echo_fifo.sv:32` (ext body for `src/echo.vir:30`) · **Back:** [Home.md](Home.md) · **Next:** [echo-server.md](echo-server.md)

`\echo::EchoFifo` is the elastic buffer between the USB core's byte stream (which arrives at USB packet rate, in bursts) and the UART sender (which drains one byte every ~417 cycles × 10 bits). It is hand-written Verilog because **Virdant cannot express indexed memories** inside a plain `mod` — the same limitation that keeps the USB core in ext/. The flow-control logic around it lives in Virdant ([echo-server.md](echo-server.md)).

## Interface

| Port | Dir | Meaning |
|---|---|---|
| `clock` | in | 48 MHz |
| `push_data` / `push_valid` | in | byte enters the **tail** on that clk edge |
| `pop_en` | in | consume the **head** byte this cycle |
| `pop_data` | out | the head byte, valid whenever `count != 0` |
| `count` | out | stored bytes, 0..16 |
| `room` | out | `count < 12` — flow control for the USB rx pop path |

## The circular FIFO

16 slots × 8 bits, a 4-bit write pointer and a 4-bit read pointer:

```verilog
reg [7:0] slots [0:15];
reg [3:0] wr_ptr;  // push to slots[wr_ptr], then wr_ptr += 1 (wraps)
reg [3:0] rd_ptr;  // pop  from slots[rd_ptr], then rd_ptr += 1 (wraps)
reg [4:0] count_q; // saturating byte count
```

Both pointers advance and wrap independently, giving true FIFO order — bytes come out exactly in the order they went in, with no stale slots and no loss, at any fill level. `count_q` saturates on both ends (`ext/echo_fifo.sv:63-68`): a simultaneous push+pop is a no-op, push at 16 stays 16, pop at 0 stays 0.

## The 16:1 read mux

The head is always `slots[rd_ptr]`, but a variable index into a reg array is not portable on the ice40 flow (nextpnr's path prefers a distributed mux; `iverilog` tolerates it, hence the explicit form):

```verilog
always @(*) case (rd_ptr)
    4'd0:  rd_mux = slots[0];
    ...
endcase
assign pop_data = rd_mux;
```

Spelled out (`ext/echo_fifo.sv:76-95`), that's a 16:1 mux of 8-bit words — cheap in LUT terms on iCE40.

## `room`: why `count < 12` and not `< 16`

The USB core's rx pop path is pipelined — a byte is in flight for a couple of cycles (`rx_pop_d → rx_valid_r → rx_valid_rr`) between the core deciding to pop and the byte actually landing here. If `room` went low only at `count == 16`, the core could pop one or two more bytes **after** `room` already deasserted, overflowing the FIFO. `room = (count_q < 12)` leaves 4 slack slots: the in-flight bytes in the core's pop pipeline always fit **above** the threshold, so no byte is ever dropped even though the core's flow control reacts one or two cycles late.

## The push/pop contract

- **Push:** `push_valid` puts `push_data` on the tail on that clk edge.
- **Pop:** `pop_data` **is the head**; it changes only on the edge that consumes it. So a pop strobe and the byte it delivers are **coincident**: the consumer can load `pop_data` combinationally on the same edge it asserts `pop_en` (that is exactly how `UartSender` loads `data` — `src/echo.vir:81-90`, see [uart-vir.md](uart-vir.md)).
- `pop_data` is valid exactly while a byte is being consumed — there is no separate "data valid" output; `count != 0` is the validity condition.

## Key takeaways

- A 16×8 circular FIFO with wrap-around `wr_ptr`/`rd_ptr` and a saturating `count`; bytes come out in exactly the order they went in.
- Verilog because Virdant can't express indexed memories; the read mux is spelled out for the ice40 flow.
- `room = count < 12`: the 4 slack slots absorb the USB core's in-flight rx pop pipeline so `room`-based flow control never drops a byte.
- `pop_data` is the head and is consumed on the same edge — load it combinationally on your `pop_en` edge.