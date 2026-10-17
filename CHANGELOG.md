# Changelog

All notable changes to Capicola. Versions follow [semver](https://semver.org);
the version string lives in `src/main.cpp` and is stamped into the HostLink
descriptor, so what the programmer reports is always what is running.

## [1.2.0] — 2026-10-15

Alchemy SDK 0.12, and a web programmer that describes the module properly.
Presets from 1.1.0 load as they are.

### Changed

- **Alchemy SDK 0.10.0 → 0.12.0.** What you'll notice:
  - **Smoother knobs.** Every pot runs through a 20 Hz filter, and a knob that
    has caught holds its value exactly until it moves, so values no longer
    drift with ADC noise.
  - **Stricter CV calibration checks.** A stored calibration for the CV jacks
    that fails the new checks is not used, and the jacks fall back to nominal
    scaling until the module is recalibrated.
  - **The programmer names each jack by its current role.** Switch Trig 1 /
    Env 1 to Outputs, or reassign Trig 2 / Env 2, and the programmer updates
    — the descriptor now refreshes at runtime.
- **Controls are named for what they are.** The buttons are "Page",
  "Slice / Flip" and "Page / Shift" instead of B1–B3; the Depth knobs are
  "Pitch Mod Depth" and so on, the routing fields "Pitch Mod Source" and so on
  ("Thresh." where "Threshold" ran off the edge); while Trig 1 and Env 1 are
  inputs they are "CV 2 / Trig 1" and "CV 3 / Env 1", ±5 V, with the printed
  names kept as their panel labels. Knob and button names now come from the
  manual like its text.
- **Every page has its color and its "About this page".** Mod Routing and
  Secondary are settings pages in the programmer, drawn on the panel, and carry
  their overview as page help; Primary and Depth now emit their orange and pink
  instead of falling back to gold.
- **Stretch reads as a direction:** Reverse / Freeze / Forward instead of a
  percent.
- **Cross-links where settings meet:** each option links to what it acts on
  (Shift Snap Target → B3 and presets, the jack options → their jacks) and the
  jacks link back; Smoothing → Threshold, Drive ↔ Character, Feedback Tone →
  Feedback, Mix → the outputs.
- **The Guide's Gestures section lists the gestures** — save, load, factory
  reset, polarity flip, Shift snap, slice — with their buttons and hold times.
- **The programmer's help text carries the details again** — hold times, pulse
  widths, pip assignments, detents, the jack defaults — appended from the
  manual's facts tables, which 1.1.0 kept on the manual page only.
- **The Buttons section is leaner.** One sentence per button; each gesture shows
  its first paragraph and its facts, since the save history and the snap target
  already have their own help in the programmer.
- **"Reading the Panel" is back** in the manual's Gestures tab and the
  programmer's guide: the page-colored buttons, B2's dark blink on a slice, the
  white confirm flashes, and the 6pm pips always showing the four followers.
- **The description sent to the programmer gets a 64 KiB buffer**, and its
  runtime refresh a 64 KiB workspace. With the full manual on board it is
  about 24.7 KB, just past the SDK's 24 KiB default — which would leave the
  programmer with an empty module.

Thanks to Luke at Hermetic for PR #1, which these changes carry forward onto 1.1,
and for SDK 0.12.

## [1.1.0] — 2026-10-08

### Added

- **Save history.** B1+B2 saves into slot 1 (the boot slot), and the save it
  replaces moves into slots 2–16, so your last sixteen are kept; once they're
  full, the oldest is replaced. Every save costs the same two writes, however
  long the history. Older saves are recalled from the web programmer — load
  one, then save from the panel to make it the latest. Panel saves name
  themselves "Save 1", "Save 2", … so the programmer shows their order; a name
  you set in the programmer is kept. Records move byte-exact, names included.
- **Load from the panel.** Hold B2+B3 for 0.5 s to bring slot 1 back without
  power-cycling. An empty slot does nothing.
- **SHIFT snap.** Hold B3 (printed SHIFT) and knobs stop adjusting and toggle:
  nudge one toward its snap target and it snaps there, nudge it back and it
  returns. Every page, all six knobs — depths to exactly zero, routing back to
  the input follower. A hold that snapped anything doesn't change the page.
- **Two more CV inputs, on by default.** Trig 1 and Env 1 are now CV inputs out
  of the box — CV 2 and CV 3 — and every routing knob has two more segments,
  purple and green, for them. Trig 2 and Env 2 each carry whichever follower
  signal you pick (input or output triggers, input or output envelope); by
  default the input triggers and envelope, so nothing is lost. Set Trig 1 /
  Env 1 Jacks to Outputs in the web programmer to use them as printed. Saved
  with each preset. The 6pm pips always show the followers, whatever the jacks
  carry. The routing decision lives in `src/jack_routing.h`, and the host suite
  checks every combination of it.
- **Routing rings show live sources.** The selected segment glows with its
  source's level — an envelope as it moves, a CV by its voltage — so the page
  shows at a glance whether anything is arriving.
- **Shift Snap Target option** (web programmer, Options): snap to factory, or to
  your latest save in slot 1. A new preset component, so the preset schema
  changes — slots from 1.0.0 need migrating in the programmer, or a fresh save.

### Changed

- **Routing colors, CCW → CW:** orange (input follower), blue (output follower),
  white (CV In — purple in 1.0.0), purple (CV 2), green (CV 3), so neighbouring
  segments never share a hue family. With five zones the two end LEDs stay dark
  for contrast.
- **Each button pair is one state move:** B1+B2 saves, B2+B3 loads, B1+B3 goes
  home to factory.
- **Save, load and reset confirm with three bright white flashes** on all three
  buttons, timed from when the operation completes; 1.0.0 blinked them dark.
- **B1+B3 resets every page**, not just the visible one. The save is untouched,
  so B2+B3 undoes it.
- **B2 slices on release, and a polarity flip no longer slices.** Hold B2 for a
  tenth of a second on the Primary page and Pitch/Stretch become direction
  controls, as before. If either was flipped, the release doesn't slice, so the
  head never snaps to now; a hold with no flip slices like a slow tap. Chorded
  presses (save, load) never slice.
- **Polarity flip reads the pot only.** Modulation riding P1/P2 no longer flips
  polarity while B2 is held; in 1.0.0 an LFO into P1 flipped on its own.
- **Factory defaults:** Stretch 0.9 of the sweep (≈ 1.75× slower), from 0.75 —
  not realtime, where a pitched grain at "now" is just the overlap-add taking
  over. Grain Size 0.15 (642 keyframes), from 0.3.
- **Quality (P5) CCW end widened** to ε 0.08, from 1.0.0's 0.03. 0.03 was not
  crunchy enough; the pre-release 0.1 was a bit much.

### Fixed

- **A polarity flip wasn't saved.** The flipped value lived in the gesture's own
  catch layer, so a save stored the pot's side: flip Pitch to reverse, save, and
  it loaded back forward. Flips are now the knob's stored value.
- **Loads from the programmer didn't re-arm catch.** A load-from-slot or live
  push over USB could be undone by the B1 page held under a B3 page, or
  overwritten by the pots with Secondary open. Every preset apply now re-arms
  catch, whoever started it.
- **The git stamp HostLink reports** is regenerated on every build and marked
  `-dirty` for uncommitted trees, so the programmer can tell builds apart.

### Docs

- **One manual.** The interactive manual (`docs/manual.html`) is the reference.
  Pick a page and the faceplate, its knob names and its colors follow; click a
  knob, button or jack — on the plate or in the list — for its details, with
  links between related settings. A Gestures tab covers factory reset, save,
  load, polarity flip, SHIFT snap, pot catch and slicing. `MANUAL.md`,
  `docs/manual-copy.md` and `tools/manual_copy.py` are gone.
- **The web programmer's text comes from the manual.** `tools/manual_sync.mjs`
  generates `src/capicola_manual_text.h` from the manual's data block, and the
  build reruns it whenever the manual changes, so the two never drift.

## [1.0.0] — 2026-10-01

First public release. Earlier version strings (0.3.0 and below) were never
tagged or published.

### Added

- **Bidirectional stretch.** P2 is bipolar: a true freeze at noon, realtime
  forward at the CW stop, realtime reverse at CCW. The keyframe grid can now
  walk backward through the buffer.
- **Bidirectional pitch.** P1 is grain playback speed, bipolar and linear:
  forward above noon, reversed below, up to 2× either way, and a true stop at
  noon. Unity sits at ¼ and ¾ with a detent holding each. The programmer shows
  the speed directly (e.g. `1.25x`).
- **Ping-pong gesture.** Hold B2 and P1 or P2 becomes a direction control: the
  rate pins where it is and a nudge of the knob flips polarity *at that same
  rate*, instead of dragging it down through the centre and back. A pitch flip
  hands off between the two grains over one crossfade, so it never clicks.
  Release and
  the knob waits behind a catch pip, exactly like a knob after a page switch. A
  CV riding the knob nudges it too, so an LFO into P1 ping-pongs on its own.
- **Buffer-exhaustion handling.** Freezing or reversing long enough that the
  grid runs out of room behind the writer now re-anchors to the delayed tap over
  a full fixed fade. Works on a partially filled ring as well as a lapped one.

### Changed

- **The save gesture moved to B1+B2** (held 0.33 s), mirroring B1+B3 = reset. B2
  alone is the ping-pong gesture, so the two cannot collide — and while B1 is
  down, a knob nudge cannot flip a polarity.
- **Quality (P5) range narrowed** to ε 0.03–0.001, from 0.1–0.001. The old CCW
  end measured around 2.5 dB reconstruction SNR on speech — past crunchy into
  broken — and because the map is linear in ε it consumed half the sweep. The
  whole travel is now usable.

### Fixed

- **Windows only stepped forward.** A cursor leaving its keyframe bracket
  *backward* was never noticed, so the window went stale for the rest of the
  pass and the read stuck at a clamped endpoint — silently, with no click and no
  NaN. Unreachable before bidirectional playback existed; a prerequisite for it.
- **Crossfades never finished at negative pitch.** The splice phase advanced by
  `increment × pitch`, which runs backward when the head does. It now advances
  by `|pitch|`; at exactly zero a fade simply holds until the head moves again.

### Notes

- Saved presets from pre-release firmware will not read correctly: P1, P2 and P5
  all changed meaning. Hold B1+B3 to reset a page to factory defaults, then
  B1+B2 to save.
