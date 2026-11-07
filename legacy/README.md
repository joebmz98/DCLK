# DCLK legacy firmware

Every DCLK firmware release, including the current one. The working copy
of the firmware lives in [`src/dclk_midi_clock`](../src/dclk_midi_clock),
and the [web flasher](https://github.com/joebmz98/DCLK-Flasher) installs
the latest release.

Each `vX.Y` folder holds:

- `dclk_midi_clock_vX.Y.hex` / `.bin`: the release build, taken from the
  repo's build folder at that version (each binary contains its own
  version string, so they're checked against the folder name)
- `dclk_midi_clock/`: the sketch as it was at that release, ready to open
  in the Arduino IDE (build with the "DCLK Dual MIDI Clock" board, USB
  support None)

| Version | Date | Main change | Build |
|---|---|---|---|
| [v0.1](v0.1) | 2026-10-22 | First prototype firmware (plain buttons) | source only |
| [v1.0](v1.0) | 2026-10-23 | First release: clock dividers, quantised stop, LO / MID / HI LEDs, screen saver, MONITOR and ABOUT, BPM saved to flash | hex, bin |
| [v1.1](v1.1) | 2026-10-23 | Version number only (same code as 1.0) | hex, bin |
| [v1.2](v1.2) | 2026-10-23 | Screen rotated 180° | hex, bin |
| [v1.3](v1.3) | 2026-10-29 | Follow external USB MIDI clock (CLK SRC / EXT TPT), SysEx protocol for the web editor | hex, bin |
| [v1.4](v1.4) | 2026-11-03 | CAP1206 touch pads (final PCB), reversed encoder | hex, bin |
| [v1.5](v1.5) | 2026-11-04 | Enter DFU (update) mode from the menu or over USB | hex, bin |
| [v1.6](v1.6) | 2026-11-04 | Analogue sync outputs driven (2 PPQN) | hex, bin |
| [v1.6.1](v1.6.1) | 2026-11-04 | SYNC menu: per-output PPQN, default 24 | hex, bin |
| [v1.6.2](v1.6.2) | 2026-11-05 | CH1 / CH2 tabs on the main screen while playing; thinner beat bars | hex, bin |

From v1.4 the firmware expects the final PCB (CAP1206 touch pads); v1.3 and
earlier read plain buttons and match the prototype.

## Flashing an older version

In the web flasher, pick **Custom file** and choose the version's `.hex`.
With STM32CubeProgrammer, flash the `.hex` as is, or the `.bin` at
`0x08000000`. Every version uses the same settings layout, so saved
settings carry across; settings a version doesn't know about fall back to
their defaults.
