# (better) ESP3D: buffered Marlin streaming

With `GCODE_HOST_FEATURE` enabled and the firmware target set to `MARLIN` or
`MARLIN_EMBEDDED`, `[ESP700]/SD/file.gcode` uses the buffered Marlin transport.
`/FS/file.gcode` and semicolon-separated scripts use the same transport. Other
firmware targets keep the existing host. The Orca upload/start compatibility
handler also reaches this transport through `processFile()`.

## Minimum hardware

**(better) ESP3D requires at minimum a dual-core ESP32.** Single-core ESP32
variants, ESP8266 and ESP8285 are outside this fork's supported hardware scope.
For the dedicated streaming worker, use an ESP32-connected SD card and a UART
connection to Marlin. Both CPU cores must be enabled in the build so the worker
can run on the core opposite the application loop.

## Changes at a glance

The fork adds a buffered, acknowledged Marlin transport and separates SD-to-UART
streaming from HTTP/WebSocket handling across the two cores. It also adds
controlled resend recovery, explicit heating waits, pause/resume/cancel,
queued nozzle and bed temperature changes, and serial/worker diagnostics.
Native tests exercise the protocol and its host/worker integration. The
[README overview](../README.md#what-changed) summarizes the user-facing changes;
the sections below describe their behavior and limits.

The Orca compatibility upload endpoint, `/upload_serial`, reuses ESP3D's SD upload
handler. Its `/command?plain=...` handling accepts the connection test and maps
`M23`/`M24` to selection and streaming of the uploaded ESP3D SD file. Starting with
`[ESP700]/SD/file.gcode` reaches the same host transport. This workflow stores the
file on ESP3D's SD card and sends its commands over the configured printer link.

## Printer requirements

This implementation targets a single-extruder Marlin printer connected to the
configured serial, serial bridge, or USB serial output. The printer must have:

- `ADVANCED_OK`: the initial numbered `M110 N0` must receive `ok N0`. Ordinary
  unnumbered `ok` responses cannot safely identify commands in a buffered session.
- `EMERGENCY_PARSER`: required to interrupt heating/user waits with `M108` and
  process `M112` promptly when a protocol or IO fault occurs.
- `HOST_KEEPALIVE_FEATURE` with an interval below 30 seconds, including keepalive
  during blocking operations. Marlin's periodic heating temperature reports also
  count as liveness.
- At least 128 bytes of UART RX storage and `MAX_CMD_SIZE >= 96`. These are
  printer-side requirements; increasing the ESP32 RX buffer does not change them.

Verify these requirements in the flashed printer firmware. Startup verifies
numbered acknowledgments, not emergency-parser operation or physical RX storage.

## Protocol and scheduling

The Arduino-independent `marlin_stream.cpp` engine retains up to eight commands
and at most 96 unacknowledged wire bytes, including numbering, checksum and
newline. The byte limit remains conservative even when the printer's command
queue fills. `ADVANCED_OK`'s `B` and `P` values describe command/planner slots and
are deliberately not used as UART byte credit.

Every command has an `N` number and XOR checksum. Only the expected numbered ACK
releases credit; duplicate older ACKs are ignored, and unexpected future ACKs
fail the session. A response line can span multiple input callbacks. `M105` is
isolated because Marlin's implementation emits an unnumbered temperature ACK;
plain `ok` alone never acknowledges it or any numbered command.

Checksum/sequence errors stop new transmission while waiting for `Resend`.
The retained suffix is replayed byte-for-byte after accepted preceding commands
have been acknowledged and trailing resend responses have drained. The plain
`ok` Marlin emits after `Resend` is ignored. Requests outside retained history,
malformed requests, or repeated retries without progress fail the session.
Timeouts never blindly resend potentially executed moves.

Blocking commands such as `M109`, `M190`, `G4`, homing and `M400` run in isolation.
Busy/temperature reports refresh liveness without acknowledging a command.
There is a 30-second silence timeout, a 120-second acknowledgment deadline for
ordinary commands, and a 24-hour deadline for recognized blocking commands.
An error awaiting `Resend` has a two-second deadline. Long ordinary moves that
block acknowledgment beyond 120 seconds also fail; these limits are explicit
constants in `marlin_stream.h`.

## ESP32 worker and core separation

For SD files and scripts sent through an ESP32 UART, `marlin_stream` runs in a
FreeRTOS task pinned to the core opposite the application task that calls
`begin()` (priority 2, 6 KiB stack). HTTP and WebSocket handling remain in the
application loop. The worker derives its core from the calling task rather than
requiring a board-specific `ARDUINO_RUNNING_CORE` setting. The retained single-core
fallback uses core 0 with preemptive scheduling, but is outside this fork's
supported hardware scope. This uses ESP-IDF's
[pinned-task API](https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/system/freertos.html).
Pinning does not isolate shared SPI, flash, UART or system networking resources;
physical throughput and task stack margins still need hardware measurement.

- The main task acquires and opens the source once. The worker owns the protocol
  engine, reads ahead in 512-byte blocks and sends directly to the selected UART.
  It never enters the HTTP/WebSocket dispatcher. Streamed writes retain the UART
  TX ring buffer and avoid a per-command transmission drain.
- UART callbacks feed a dedicated queue of 32 packets of at most 64 bytes before
  main-loop routing. Overflow faults the stream; protocol bytes are never
  silently discarded. Stale generations and delayed UI copies cannot ACK twice.
- Pause/resume/cancel use a separate eight-entry control queue. Emergency stop
  uses a separate flag, so RX/control congestion cannot queue it behind normal
  requests. A short mutex protects host state; UART ingress does not acquire it.
- Display replies use a separate sixteen-packet queue. When HTTP stops draining
  it, whole display lines can be dropped without blocking or losing protocol ACKs.
  Only the main task sends these copies to network clients.
- The worker yields every active iteration and wakes on UART/control notifications
  or a one-tick deadline. The main task closes/releases files after completion.
  The SD watchdog cannot release the worker's live lease during a pause or a
  blocked main loop. Service shutdown/restart stops the worker before closing its
  filesystem or UART; an active job attempts `M112` during shutdown.
- Changes to transport settings and serial-service controls are rejected while
  the stream owns the connection. Task/queue allocation failure rejects jobs
  requiring the worker rather than silently reverting to main-loop timing.

USB serial and LittleFS sources retain the cooperative host path. The inherited
ESP8266 path is also retained, but ESP8266 is not supported by this fork. In
particular, LittleFS source handles are not moved across tasks while HTTP uses
its global file-handle table. The dedicated worker currently covers the requested
ESP32 SD-to-UART print path and UART scripts.

## Controls and completion

| Command | Behavior |
| --- | --- |
| `[ESP701]` | Reports processing, pausing, pause, cancelling, or the final error. |
| `[ESP701]diag` | Returns JSON with the protocol state, recent reply prefixes, and ESP32 UART/worker diagnostics. Available after a fault; read before restarting ESP3D. |
| `[ESP701]action=PAUSE` | Stops reading/sending new source commands, drains accepted commands, sends `M400`, and reports pause only after its ACK. |
| `[ESP701]action=RESUME` | Resumes after pause has completed, preserving the next pending source line. |
| `[ESP701]action=ABORT` | Requests cooperative cancel: stops source feeding, sends `M108`, drains accepted commands, then acknowledges `M400`, `M104 S0`, `M140 S0`, and `M107` before releasing the file/connection. |
| `M104 S205 T0` / `M140 S60` from a client | Queues a nozzle / bed setpoint in the numbered stream, ahead of new file commands. Hotend `T` is optional. |
| Marlin `//action:cancel` | Requests the same cooperative cancellation as ESP701 ABORT; the signal alone does not trigger `M112`. |
| `M112` from a client | Emergency stop; faults the session and requires recovery below. |

Pause leaves heater targets unchanged and does not park/retract the head. It can
remain “pausing” while an already accepted heating command completes. Cancel is
not an immediate motion stop: already accepted motion finishes before heater/fan
shutdown. The shutdown sequence targets the active hotend and bed; multi-extruder
shutdown is outside this version's support. A real emergency uses `M112`.

At EOF, the final `M400` must be acknowledged before success is reported. File
progress follows acknowledged source offsets (not SD prefetch); an individual
motion ACK is not a claim that motion has finished. Trailing comments are counted
when the final barrier completes.

While the stream owns the printer connection, manual commands other than the
setpoints above and `M112` are rejected, including polling; HTTP `/command`
returns 409. Printer replies are copied to
network clients on a best-effort basis. `[ESP701]` remains available, as does a standalone `M112`.
Use Marlin automatic temperature reports (`M155`, if enabled in the printer)
when live temperature updates are needed during printing. Do not change baud,
output routing, firmware target or other transport configuration during a job.
Network controls still need the HTTP handler to accept the request; the worker
keeps printing through a blocked handler but does not make that handler responsive.

Manual temperature changes use a four-entry pending queue protected by the host
lock. The caller never writes UART bytes. The worker gives accepted changes
priority over unread file commands, with the same numbering, checksum, byte
window, ACK tracking and resend retention as file commands. HTTP 200 means
queued, not yet executed. A full queue rejects the request with HTTP 409.
Setpoints also work while paused, without resuming motion or changing file
progress. Only `M104 S... [T...]` and `M140 S...` are accepted; Marlin remains
responsible for clamping temperatures to its configured heater limits.

Already transmitted commands retain their order. An active `M109` / `M190`,
homing or probing operation must finish before a queued setpoint can execute.
Later temperature commands in the G-code file can override manual setpoints.
Cancellation discards unsent setpoints so they cannot reheat after shutdown.
Clean cancellation reports `no stream` with `result: cancelled` in the JSON
status response; genuine printer errors, including a halt following a cancel
signal, still fault the transport.

A timeout, printer error/restart, invalid source, exhausted resend recovery or
write/read failure attempts `M112` and latches the session fault. No queued script
starts afterward. Reset the printer and restart ESP3D before another job;
`CLEAR_ERROR` only clears the displayed error, not the transport lock. A failed
physical link can prevent the emergency command from reaching the printer.
A clean cooperative cancel permits another job without a reset.

## Investigating damaged replies

A hardware motion run produced `ok3N212 P0 B13` between valid N211 and N213
acknowledgments. The missing space makes N212 unrecognizable; the subsequent
N213 correctly faults the stream as out of order. Another run of the same file
completed. This establishes an intermittent failure, not its origin.

The inspected Marlin source (`GCodeQueue::RingBuffer::ok_to_send` in
`Marlin/src/gcode/queue.cpp`) explicitly writes a space before `N`. The active
STM32 transport uses the Arduino HardwareSerial TX ring and interrupt driver.
ESP3D's UART callback copies received bytes into the worker queue without text
conversion. Source inspection has not identified where the space changed.

A later print captured `ok2N406` in both the raw UART ingress tail and parser
history, with zero packet mismatches, queue overflows, or reported UART errors.
This narrowed that failure to corruption before worker processing, without
identifying its electrical or software origin. Transport diagnostics deliberately
retain malformed bytes; the parser does not repair or assume missing ACKs.

After a run, before restarting ESP3D, send `[ESP701]diag`:

- `data` preserves the ordinary ESP701 status fields for WebUI compatibility.
  Its `diagnostics` object contains `last_ack`, `expected_ack`, `pending`,
  `error`, and `reason` describing protocol state.
- `replies` retains four complete parser input lines, oldest first, with up to
  96 bytes per line in hex. CR and newline are excluded; `length` is the captured
  parser length (at most 255). NUL/overlong lines fault. History stops changing
  at a terminal state and resets when a new session starts.
- On the ESP32 UART worker path, `rx_tail_hex` contains the last 256 bytes copied
  from HardwareSerial, including delimiters, before worker queueing.
- Each worker packet gets a checksum at ingress and is verified on dequeue.
  A mismatch faults immediately. `packet_mismatches` and `queue_overflows`
  distinguish these paths from an already damaged incoming reply.
- `uart_errors_since_start` counts driver-reported break, RX buffer full, FIFO
  overflow, framing and parity events on the selected port. Late events remain
  counted after the stream stops, until the next session. Zero counters do not
  exclude corruption of a byte that still has valid UART framing (8N1).

If the damaged byte appears in both ingress and parser history with zero packet
mismatches, it was already damaged before worker processing. This still leaves
Marlin TX, the physical link and ESP32 UART reception as possibilities. A passive
logic-analyzer capture at printer TX and ESP32 RX would distinguish the wire
signals from software observations. These diagnostics are not a wire capture.

Compare a stationary file of 2,400 `G90` commands followed by `M400` against the
same non-heating motion file that failed. Record diagnostics after each run;
keep baud, wiring and UI polling unchanged initially. If a failure occurs,
save diagnostics before resetting both devices. Do not accept malformed ACKs
or blindly replay potentially executed commands to conceal the fault.

### Hardware validation scope

Stationary and non-heating motion jobs completed on an ESP32-to-Marlin setup,
including final barrier acknowledgments and zero queue-integrity errors. The
user also reported responsive WebUI interaction during a real print. These
observations cover one installation, not all supported boards or printer
firmware configurations. Link errors must be investigated independently; this
transport does not change the configured UART pins or baud rate.

## Input scope

Input is unnumbered ASCII G/M/T code. File semicolon and parenthesis comments are
removed; CRLF, empty lines, long comments and missing final newline are handled.
Script semicolons remain command separators. Existing numbered/checksummed input,
embedded ESP commands, transport/SD-control commands, quoted or escaped strings,
non-ASCII data and commands too long after numbering/checksum are rejected rather
than truncated. A source read failure before the expected file size is an error,
not a successful EOF.

## Local verification

Run from the repository root:

```sh
tests/marlin_stream/run.sh
tests/marlin_stream/run_worker.sh
SANITIZERS=thread tests/marlin_stream/run_worker.sh
UNICORE=1 tests/marlin_stream/run_worker.sh
MAIN_CORE=1 tests/marlin_stream/run_worker.sh
pio run -e esp32dev
```

`UNICORE=1` checks the retained compatibility fallback; passing it does not make
single-core deployment supported. The supported deployment requires two cores.

The native runner uses Clang by default (`CXX` can override it), strict warnings,
AddressSanitizer and UndefinedBehaviorSanitizer. LeakSanitizer is disabled because
it cannot run under the desktop sandbox's process tracing. Temporary binaries
are removed automatically.

- Protocol tests compile the actual firmware engine. They cover manual nozzle/bed
  controls, display cancellation, fragmented and
  duplicate ACKs, unrelated temperature responses, bounded credit, checksum and
  sequence recovery, malformed/out-of-window resends, heating waits, lost ACKs,
  clock rollover, pause/resume/cancel barriers, startup requirements, write
  failure, printer restart and thermal errors.
- An independent simulated printer validates checksum/sequence, limits its RX
  storage and command queue, corrupts three frames in a 1,000-command relative
  motion job, and checks that each command executes exactly once in order.
- Host integration tests compile the actual `gcode_host.cpp` with fake Arduino,
  filesystem, clock and message-dispatch boundaries. They cover read-ahead,
  comments/line endings, response origin, authentication propagation, pending
  lines across pause/resume, cancel without later source commands, file IO
  failures, resource cleanup and the legacy non-Marlin script path.

- Worker tests compile the actual task entry and host code against a small
  FreeRTOS boundary shim backed by real host threads. A 1,000-command job finishes
  with no main-loop service. Checks include bounded display backlog, wrong-port
  input, pause/cancel, RX overflow, emergency priority, shutdown/restart, task
  creation failure and single/dual-core selection. ASan/UBSan and ThreadSanitizer
  exercise the native task integration; they do not emulate the ESP32 scheduler.
- The captured damaged N212 reply is replayed at every callback split. Tests
  check the missing ACK, subsequent fault, single emergency stop, retained
  history, and a valid control sequence. A threaded test compares raw ingress
  and parser history after reusing caller buffers, and checks UART counter
  attribution and diagnostic retention after main-loop cleanup.

These checks do not measure UART throughput, timing under Wi-Fi load, motor
motion or heater behavior on hardware. Printer validation is left to the user.
Start with a short non-heating file and verify pause/resume/cancel, then heating
waits and a dense movement file; inspect the reported state and actual motion.
