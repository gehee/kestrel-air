# kestrel-air: how it is put together

kestrel-air runs the Caddx Ascent air unit in place of the stock air app: the
camera, the H.265 encoder, the AR8030 radio (through ar_libre) and the
messages with the ground. What it does the stock way was reverse-engineered
from the stock app; what is ours is said where it is done.

## Modules

| Folder | What it does | Talks to |
|---|---|---|
| `protocol/kestrel_air.h` | The wire format the ground reads: the packet around each slice, header bytes, feature bits, our commands | video, ground; a ground takes this file as it is |
| `src/app/` | Start-up (`app_run`), the stock settings file (`config.c`), the `KA_*` environment settings (`settings.c`), the flags the threads share (`state.h`) | everything |
| `src/camera/` | The Hi3516CV610 pipeline: MIPI, VI, ISP with the sensor's driver (libsns_*.so), VPSS (`pipeline.c`); the image settings and the tuning bin (`image.c`) | video (VPSS -> encoder) |
| `src/video/` | The encoder channel and its tuning (`encoder.c`), the packet around each slice and the sender (`video.c`), the local packet ring (`ring.c`), `--bb-verbose` timing (`stats.c`) | camera, radio, imu |
| `src/radio/` | ar_libre's client (`client.c`); the radio set up and run as stock does: requests, events, timers, link state (`radio.c`), channels, pairing, bandwidth, transmit power and standby, the chip's ADC | video, ground, board |
| `src/ground/` | Messages with the ground on radio port 2: the frame (`frame.c`, no I/O), the ground's commands, the air's reports | radio, camera, video, app |
| `src/unit/` | The board: pairing key, LEDs, debug commands (`board.c`); the flight controller over MSP (`fc.c`) | radio, ground |
| `src/imu/` | The ICM-40609-D, sampled at 1 kHz, its samples carried as SEI after each picture | video |
| `src/common/` | Clocks, CRC-32C and CRC-8, a small JSON tree | everything |

Inside a module, what its files share is in its `internal.h`; other modules
use only the module's own header.

## Data flow

```
sensor -> VI -> ISP -> VPSS -> encoder --(slices)--> read thread --> ring --> send thread --> radio port 3 --> ground
                                    ^                     |                         |
                     rate, keyframes, pause         IMU SEI appended         MCS, backlog, link
                                    |                                               |
ground --> radio port 2 --> ground rx thread --> commands --> camera / video / radio settings
air reports <-- tick thread (version, settings, telemetry, channels) <-- radio, image, board
```

## Threads

| Thread | Where | Job |
|---|---|---|
| main | `app/app.c` | Start everything, wait for a signal, then stop the video and the camera |
| ISP | `camera/pipeline.c` | `ss_mpi_isp_run`: the ISP's own loop |
| image | `camera/image.c` | Every 100 ms: the tuning bin for the gain level, with hysteresis |
| read | `video/video.c` | Take each slice from the encoder, build its packet, put it in the ring; the encoder's life (create, recreate, stop) |
| send | `video/video.c` | Write the ring to radio port 3, one packet at a time, pacing on the radio's backlog |
| drop | `video/video.c` | Pause the encoder's input while its output or the send path is backed up |
| socket readers | `radio/client.c` | One per radio socket: incoming data to its callback |
| bus watchdog | `radio/client.c` | A wedged radio bus: exit with status 2, which the wrapper answers with a reboot |
| events | `radio/radio.c` | The radio's events, in order, as stock handles them |
| timer | `radio/radio.c` | Every 100 ms: the delayed link-down, retransmission pressure, bandwidth (`r_bw_tick`) |
| standby | `radio/power.c` | Transmit power: low power (and the sensor too) while not flying with the ground's standby mode on, the ground's level otherwise |
| ADC | `radio/adc.c` | Every 500 ms: battery voltage, SoC temperature |
| ground rx | `ground/ground.c` | Frames from the ground: acknowledge, then the command |
| tick | `ground/reports.c` | The periodic reports to the ground |
| apply | `ground/commands.c` | One per INIT_CFG that changes something: apply it off the rx thread |
| key, LED, debug | `unit/board.c` | The pairing key, the LED state machine (100 ms), debug commands on UDP 4486 |
| fc rx, fc poll, fc forward | `unit/fc.c` | MSP from the flight controller, polling it every 20 ms, forwarding responses to the ground |
| IMU | `imu/imu.c` | The IMU's FIFO at 1 kHz |

What several threads read and write is either in `app/state.h` (C11 atomics)
or behind a mutex in the module that owns it.

## Tests

`make test` builds `tests/` with the host's compiler: the protocol header,
the ground frame, the checksums, JSON, the packet ring and the settings.
Everything that needs the SDK or the radio is checked on the unit.
