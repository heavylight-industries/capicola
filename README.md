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
  knob→unit mappings, composed into three `Page`s on a `Pager`; mod
  routing (`ModRouter`), CV IO, LED layers and presets all live here.
- `src/audio/` — `AudioEngine`: owns the two per-channel
  `KeyframeRecorder` chains, filtered feedback, dry/wet mix, and the
  output-side follower. Setters take engineering units.
- `lib/` — the DSP (`KeyframeRecorder` / `Analyzer` / `SparseLine` /
  `Granule` / `Detector`). Header-only.
- `lib/alchemy-sdk/` — the SDK (submodule); vendors libDaisy under
  `lib/alchemy-sdk/vendor/libDaisy`.
- `patches/` — snapshots of local submodule drift (see below).

## Build

Requires `cmake ≥ 3.21`, `ninja`, `arm-none-eabi-gcc` (builds with 10.2.1;
the SDK nominally asks for ≥ 12), and `dfu-util` for flashing.

```sh
git submodule update --init --recursive    # first checkout only
cmake --preset arm
cmake --build --preset arm
```

Outputs `build-arm/capicola.{elf,bin,hex}`.

## Flash (Daisy bootloader / front USB-C)

Put the module in update mode (hold **B3** through the ~2 s boot window, or
hold while powering on), then:

```sh
cmake --build --preset arm --target capicola-flash
```

`...-size` prints the memory footprint.

## Debug

`cmake --preset arm-debug` builds a SWD/SRAM image (`-Og -g3`, no
bootloader) that the probe loads straight into SRAM — F5 with the
cortex-debug config in `.vscode/launch.json`. The defines it needs
(`DAISY_FORCE_FULL_INIT`, `VECT_TAB_SRAM`) come from the root
`CMakeLists.txt`, but the force-full-init *handling* is a local patch to
the vendored libDaisy (see below) — the debug image dies in pre-main
without it.

## Submodule drift — do not `git submodule update` casually

`lib/alchemy-sdk`'s vendored libDaisy carries one local uncommitted patch,
snapshotted in `patches/`:

- `vendor/libDaisy/src/daisy_seed.cpp` — `DAISY_FORCE_FULL_INIT` (full
  clock + SDRAM bring-up for the SWD/SRAM debug image).

A recursive `git submodule update` destroys it — reapply from `patches/`
if that happens.

## License

Capicola is free software, released under the **GNU Affero General Public
License v3.0** — see [LICENSE](LICENSE). If you ship hardware running a
modified capicola, you must offer the corresponding source.

Third-party code keeps its own terms: `lib/alchemy-sdk` (and the libDaisy
it vendors) are MIT-licensed submodules; see their `LICENSE` files.
