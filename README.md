# kestrel-air

## Overview
`kestrel-air` is the air-side application of the kestrel FPV system for the
**Caddx Ascent Lite and Lite+ air units** (HiSilicon Hi3516CV610, CV2004 sensor,
AR8030 radio). It takes the place of the stock air app and runs the unit as it does:
camera, H.265 encoder, the AR8030 radio and the ground's messages, so a stock
goggle and [kestrel-gnd](https://github.com/gehee/kestrel-gnd) both work with
it. What it does the stock way was reverse-engineered from the stock air app;
on top of that it sends each slice as soon as it is encoded, with intra
refresh instead of periodic keyframes, and tells kestrel-gnd its own timing.

The radio is reached through [ar_libre](https://github.com/gehee/ar_libre)'s
client library, straight through the radio's device file with no daemon:
either the stock `/dev/artosyn_sdio`, or `/dev/arlink0` with ar_libre's own
`arlink.ko`.

## How to Build
It links the Hi3516CV610 SDK's libraries (MPI, ISP and its algorithm
libraries), which are not part of this repository. Point the build at an ARM
musl toolchain and at a copy of the SDK:
```bash
make CROSS_COMPILE=<toolchain prefix> CV610_SDK_INC=<SDK root> \
     CV610_SDK_LIB=<SDK libraries> ARLIBRE=<ar_libre checkout>
```
`make test` builds and runs the host-side tests (`tests/`) with the host's
compiler: the protocol header, the ground messages' framing, the checksums,
JSON and the packet ring. They need neither the toolchain nor ar_libre.

`CV610_SDK_INC` is the SDK's root (its `kernel/include` and
`libraries/isp/include` trees), `CV610_SDK_LIB` the directory with its `.so`
files, `ARLIBRE` an ar_libre checkout whose library is built first:
`make -C $ARLIBRE/lib CROSS_COMPILE=<toolchain prefix>`.

The camera's sensor driver, `libsns_cv2004.so`, is installed
next to it. kestrel-air loads it
at run time. Units with another sensor than the CV2004 are not supported.

[fpvOS](https://github.com/gehee/fpvOS) builds all of it into an air unit image.

## Running on the unit
kestrel-air needs, next to it (`LD_LIBRARY_PATH`), `libsns_cv2004.so` and the
SDK libraries it links:
- `libss_mpi`, `libss_mpi_isp`, `libss_mpi_ae`, `libss_mpi_awb`, `libot_mpi_isp`
- `libss_mpi_sysmem`, `libss_mpi_sysbind`, `libot_osal`, `libsecurec`, `libbin`
- `libacs`, `libbnr`, `libcalcflicker`, `libdehaze`, `libdrc`,
  `libextend_stats`, `libir_auto`, `libldci`

It also needs ar_libre's `libar8030_client.so`.

The stock app holds the camera and the radio, so stop it first, and the stock
radio daemon with it:
```bash
killall ar_ldyhs_sky; killall daemon_sdiov12
LD_LIBRARY_PATH=/tmp/k /tmp/k/kestrel-air --bb-verbose
```
A camera already started by another program cannot be started again until
the unit reboots (`OT_MPI_ISP_MemInit failed`).

## Source layout
How the parts fit, and every thread: [docs/architecture.md](docs/architecture.md).
```
protocol/kestrel_air.h   what goes on the radio for the ground: packet, header bytes, features, commands
src/main.c               options
src/app/                 start-up, settings (/factory/fpv_config.json), the state the modules share
src/camera/              the Hi3516CV610 pipeline (MIPI, VI, ISP, VPSS) and the image settings
src/video/               encoder channel, packets around each slice, local ring, sender
src/radio/               the AR8030: ar_libre client, link setup, events, bandwidth, power, pairing
src/ground/              messages with the ground: framing, commands, periodic reports
src/unit/                the board (Lite or Lite+, key, LEDs, debug commands) and the flight controller link
src/imu/                 the IMU and its SEI messages
src/common/              clocks, checksums, JSON
tests/                   host-side tests (make test)
```

## Usage
```
kestrel-air [options]
```
The picture size, frame rate and camera angle come from the unit's own
`/factory/fpv_config.json`, as with the stock app.

| Option | | Default |
|---|---|---|
| `--bb-window N` | video writes to the radio allowed in flight, unacknowledged | `1` (as stock) |
| `--max-exposure-us US` | cap the sensor exposure | `0` (the tuning's own) |
| `--imu` | wake the unit's IMU and carry its samples in the video as SEI (below) | off |
| `--imu-dev PATH` | `--imu` on another spidev node | `/dev/spidev0.0` |
| `--bb-verbose` | per-stage timing and radio events in the log | off |
| `--tx-mode ar8030` | accepted for older start scripts; it is the only mode | |
| `--help` | the options | |

## Telling the ground who we are, and what the air side measures
kestrel-gnd tells kestrel-air from the stock air app, and reads the air side's own
timing, from bytes the stock app always sends as zero:
- every slice header: byte 33 `K`, byte 10 the protocol number, byte 11 feature
  bits (1 air-side times, 2 intra refresh, 4 IMU SEI, 8 us radio-clock stamps); the
  version message (cmd 0x04) carries the same in bytes 1..4 as `K` `A`, protocol,
  features;
- bytes 35..41 of each slice header: capture to encoder output, the time the slice
  waited in the air's packet ring, how long the previous slice's write to the radio
  took (10 us LE units), and the ring's depth. The layout is in
  `protocol/kestrel_air.h`;
- bytes 26..29 and 2: the radio's own clock (which the ground reads too) when the
  encoder handed the slice out, so the ground has each picture's capture on its own
  clock.

| Environment | | Default |
|---|---|---|
| `KA_IR` | intra refresh, CTU rows (or columns) per picture; 0 = the stock stream with periodic keyframes | `2` |
| `KA_IR_MODE` | 0 rows, 1 columns | `0` |
| `KA_IR_QP` | intra refresh's `request_i_qp` | `32` |
| `KA_GOP` | GOP length. With intra refresh it must be one sweep: the CV610 refreshes once per GOP and then stops until the next GOP start (a P picture there, not a keyframe) | one sweep with intra refresh (17 at 1080p with 2 rows), `25` without |
| `KA_LD_LINES` | VPSS hands the encoder lines in steps of this many | `64`, `128` with intra refresh (32 is 0.3 ms faster but sometimes halves the frame rate) |
| `KA_LATINFO` | `0` leaves header bytes 2, 10, 11, 26..29, 33, 35..41 as the stock app sends them | `1` |
| `KA_SLICE_COUNT` | slices a picture; the CTU rows each follow from the height | `2` above 60 fps, `4` at 60 and below |
| `KA_STRATEGY` | the video strategy, whatever the ground's settings say (0 low delay, 2 wing) | the ground's |
| `KA_DUMP`, `KA_DUMP_KB` | write the first `KA_DUMP_KB` KiB sent to a file, as an Annex-B stream | off, `8192` |
| `KA_BW40` | `1`: widen the video link to 40 MHz on its own when the link is strong (each change re-forms the link, about 0.7 s without picture), and take the goggle's Max Bandwidth cap | off |

Unset or empty means the default. kestrel-air logs the `KA_*` variables set
when it starts, and names any it does not know; the full list is in
`src/app/settings.c`.

Encoder settings can also be changed while it runs, to judge them on the screen:
write `name=value` lines to `/tmp/ka-tune` on the unit (RAM only, gone at the next
boot) and kestrel-air applies the file within half a second, logging `video: tune:` in
`/tmp/air.log`. Names: `ir`, `ir_qp`, `gop`, `ir_mode`, `row_qp` (rc `row_qp_delta`,
6), `i_prop` (`max_i_proportion`, 3), `min_qp`, `max_qp`, `cu` (1 stock's CU costs, 0
the encoder's own), `roi` (poor-link side regions: 0 never, 1 stock's QP 51, 2 QP +6),
`fgp` (foreground protection 0/1), `thr` (rc texture thresholds: 0 ours, 1 the SDK's),
`scene` (0..3, 1), `irlog` (1: intra area per picture once a second). A name left out
goes back to its default. `ir`, `ir_qp`, `gop` and `ir_mode` recreate the encoder
channel (a keyframe): a GOP changed on a running channel turns intra refresh off.
The VPSS step stays what the unit booted with (128 lines with intra refresh, 64
without), so turning intra refresh on live on a unit booted without it may halve
the frame rate.

A watchdog (`src/radio/client.c`) exits with status 2 if arlink.ko
reports a failure of the radio's bus; the unit's wrapper script answers with a reboot.

## IMU in the video stream
The Ascent has an ICM-40609-D (6-axis) on SPI0 chip select 0, `/dev/spidev0.0`.
The stock app never wakes it. With `--imu`, kestrel-air does: gyro ±2000 dps and
accelerometer ±32 g at 1 kHz, read from the chip's FIFO, stamped on the MPP
clock (`CLOCK_MONOTONIC`, the clock of the capture time in the radio packet
header). The samples ride in the video as H.265 suffix-SEI NAL units
(`user_data_unregistered`, UUID `kestrel-air-IMU1`) after the last slice of each
picture: about 14 bytes a sample, 155 kbit/s at 1 kHz. The layout is in the
header of `src/imu/imu.c`. Decoders ignore it (the decoded pictures are
bit-identical with and without), and the first-slice latency is unchanged.
It is off by default: stock grounds were never tried with it.

`kestrel-gnd` drops SEI before the decoder, so the DVR does not keep it yet.
`KA_DUMP=/tmp/s.h265 kestrel-air --imu ...` keeps the first 8 MiB sent (`KA_DUMP_KB`)
as an Annex-B stream, the IMU SEI included.

## License
GPL-3.0 ([LICENSE](LICENSE)), with an additional permission to combine
kestrel-air with the Hi3516CV610 SDK's libraries
([LICENSE.exception](LICENSE.exception)).
