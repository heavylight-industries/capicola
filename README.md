# Capicola
Real-time time stretcher, pitch shifter, transient detector, and envelope follower module built on the Hermetic Modular Alchemy Lab platform. This is the hardware embodiment of my independently authored, peer reviewed DAFx26 paper titled "Keyframe Time Stretching via Extrema Sampling". 

The core idea of the module is this: Incoming stereo audio gets stretched out, and in parallel its envelope and transients are extracted. When a transient passes the threshold, it causes the lagging read head to snap to the current time with a short crossfade while continuing to stretch from the new position. This provides a means of time stretching while staying true to the rhythm of the input. The block diagram is shown below.

*INSERT BLOCK DIAGRAM HERE*

The input and output envelope followers and their transients don't just live internal to the module. They're always accessible on the CV outputs of the module to trigger and modulate external devices. And, each of the performance controls is normalized to the envelope followers, enabling the dynamics of the signal to modulate the controls in real time, leading to some bizarre and creative self patching that's clearly visualized by the LED rings and can be as subtle or extreme as desired. 

KeyframeRecorder.h is the core method as described in the paper, and is set up for offline recording and playback use in contexts other than the Alchemy Lab. Additionally, lib contains several header files that are useful in their own right, namely:
- An implementation of Vadim Zavalashin's "Zero Delay Feedback" filters (1-pole, state variable, and 4 pole ladder)
- "Deluxe" delay lines that offers not just Hermite or B-Spline interpolated reads from a ring buffer, but also high quality first and second derivatives along with the Teager-Kaiser Energy Operator (TKEO).
- Tabulated function storage and recall for convenient low cost tanh(), sin(), and so on.
- A TKEO driven peak detector with adaptive thresholding

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
