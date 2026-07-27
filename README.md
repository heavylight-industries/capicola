# Capicola

Auto-slicer / keyframe time-stretch module for the **Alchemy Lab V2** board
(STM32H750, 64 MB SDRAM, 48 kHz / 64-sample blocks), built on the
[alchemy-sdk](https://github.com/hermetic-modular/alchemy-sdk).

Two independent channels each run live capture → B-spline keyframe
sparsifier → granular playback, splicing onto fresh material on every
detected transient ("Keyframe Time Stretching via Extrema Sampling",
DAFx26). Pitch and stretch are decoupled; the stretch grid re-anchors on
triggers so tails ring out under the incoming rhythm.

**Panel reference: [MANUAL.md](MANUAL.md)** — pages, buttons, shift
functions, jack map, fixed internals.

## Layout

- `src/main.cpp` — the entire UI: `VirtualKnob` declarations carry all
  knob→unit mappings, composed into two page groups (B1: perf/depth on a
  `Pager`; B3: routing/secondary, hand-rolled); mod routing (`ModRouter`),
  CV IO, LED layers and presets all live here.
- `src/audio/` — `AudioEngine`: owns the two per-channel
  `KeyframeRecorder` chains, filtered feedback, dry/wet mix, and the
  output-side follower. Setters take engineering units.
- `lib/` — the DSP (`KeyframeRecorder` / `Analyzer` / `SparseLine` /
  `Granule` / `Detector`) plus the tabulated waveshapers
  (`Shapers` / `TabulatedFunction` / `Tabulator` / functors). Header-only.
  The waveshaper is applied to keyframes before interpolation, so drive
  cannot alias.
- `lib/alchemy-sdk/` — the SDK (submodule); vendors libDaisy under
  `lib/alchemy-sdk/vendor/libDaisy`.

## Build

Requires `cmake ≥ 3.21`, `ninja`, `arm-none-eabi-gcc` (builds with 10.2.1;
the SDK nominally asks for ≥ 12), and `dfu-util` for flashing.

```sh
git submodule update --init --recursive    # first checkout only
cmake --preset arm
cmake --build --preset arm
```

Outputs `build-arm/capicola.{elf,bin,hex}`.

The `arm-bench` preset builds the same image with a 1 Hz CPU/CV serial log
on the front USB instead of HostLink (they share the CDC port) into
`build-arm-bench/`.

## Flash (Daisy bootloader / front USB-C)

Put the module in update mode (hold **B3** through the ~2 s boot window, or
hold while powering on), then:

```sh
cmake --build --preset arm --target capicola-flash
```

`...-size` prints the memory footprint.

## License

Capicola is free software, released under the **GNU Affero General Public
License v3.0** — see [LICENSE](LICENSE). If you ship hardware running a
modified capicola, you must offer the corresponding source.

Third-party code keeps its own terms: `lib/alchemy-sdk` (and the libDaisy
it vendors) are MIT-licensed submodules; see their `LICENSE` files.
