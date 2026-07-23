# Capicola — Panel Reference

> **⚠️ Out of date.** This describes the 2026-07-10 pre-release panel layout.
> The controls have since been reworked (pages, routing view, jack map) and a
> full rewrite is in progress.

*Auto-slicer / keyframe time-stretcher, Alchemy Lab V2. Firmware branch `v2-sdk-migration`, 2026-07-10.*

Capicola listens constantly. Two independent channels (L/R) each run a transient
detector over the input; every detected transient — or a press of **B1** — splices
the granular playback head onto fresh material. Pitch and time-stretch are fully
decoupled, and the stretch grid re-anchors on every trigger, so stretched tails
ring out underneath while the output stays locked to the incoming rhythm.

The wet path's latency is the OLA crossfade: **20 ms** (960 samples, fixed
internal). That fade is also the retrigger floor — splices can't come faster.

---

## Getting around

**B2** cycles the three pages. The active page is announced by the **mode
color**, worn simultaneously by all three button LEDs:

| Page | What it is | Ring color | Mode color |
|---|---|---|---|
| 1 | Performance | blue | **orange** |
| 2 | Mod depth (the CV page) | violet/orange bipolar | **white** |
| 3 | Mod routing | source colors | **teal-green** |

Button LEDs are never off — so all activity feedback is a **dark flash**:
B1 blinks off when it fires a trigger, B3 goes dark while held (shift active),
and all three blink off together to confirm a settings save.

**Pot catch:** every page keeps stored values per knob. After boot or a page
switch, a knob is *decoupled* until the physical pot sweeps through the stored
value (the ring shows a muted pip at the stored position) — then it catches and
tracks. Nothing ever jumps. On page 3 the catch is zone-granular: entering the
stored zone is enough.

**Settings:** hold **B1+B2 for 1.5 s** to save everything (all pages) as the
boot state — QSPI preset slot 0, confirmed by the triple dark blink. On boot
the module restores that save, or falls back to factory defaults. Hold
**B1+B3 for 0.5 s** to reset the *current page only* to factory defaults
(same blink; the QSPI save is untouched until the next B1+B2). For either
chord, press the other button before B1 — B1 alone fires a trigger.

---

## Page 1 — Performance (blue)

| Knob | Function | Range | Notes |
|---|---|---|---|
| P1 | Pitch | ±12 semitones | Unity at noon. Grain pitch only — time is untouched. |
| P2 | Stretch | 1.0 → 0.01 | CCW = realtime, CW = deep stretch (100×). Exponential. |
| P3 | Threshold | ratio 0–8 | See below — it's relative, not absolute. |
| P4 | Grain size | 32–4096 keyframes | The adaptive splice leash. |
| P5 | Mix | dry/wet | See below. |
| P6 | Feedback | 0–2 loop gain | See below. |

**P3 — threshold is self-calibrating.** The detector tracks a running average
of the envelope's own peaks and arms at `knob × average` — a *ratio*, not an
absolute level, so it needs no gain-staging and survives level changes. The
first 90% of the sweep spans ratio 0 (keep everything) to 4 (transients only);
the last stretch climbs to 8 (very picky). **The very top of the knob
hard-mutes auto-triggering** — the module keeps playing its current material
and only B1 (or a re-open of the knob) splices in new audio. Release hysteresis
is fixed at 0.95.

**P5 — mix.** The wet path lags the dry by the 20 ms fade, so mid-mix settings
comb/phase against sustained material — expected, and a sound of its own at
unity pitch. Full wet (CW) is the classic Capicola behavior and the boot
default. Mix is a first-class mod destination like every other perf knob.

**P6 — feedback.** The previous block's *wet* output → `tanh` → 480 Hz
state-variable filter (50/50 highpass+bandpass — strips DC and lows, keeps
bite) → back into the recorder input, where it gets re-sliced and re-pitched.
The knob is loop gain: **above 1.0 is deliberately unstable** (chaos zone),
with a hard ±1 clamp on the injection as the runaway guard. The feedback tap
sits *before* the mix blend, so turning P5 down doesn't starve the loop.

**6-o'clock pips** (always on, every page): the six pips are a live mirror of
the six CV jacks. P1/P2 show the two CV *inputs* in the modular-standard
bipolar colors — green = positive, red = negative — with brightness tracking
level. P3–P6 mirror the four *outputs* — in-gate (P3, also lights on a B1
press), out-gate (P4), in-envelope (P5), out-envelope (P6). The panel always
shows what the jacks are doing.

---

## Page 2 — Mod depth (white)

Each pot is the **bipolar modulation depth** for the same-numbered performance
knob: noon = off, CW (violet) = positive, CCW (orange) = negative. A faint
page-hued glow marks a centered (inactive) depth.

Modulation is applied as an offset to the perf knob's *position* —
`source × depth` — then runs through that knob's normal range mapping. Follower
sources are unipolar 0..1; the external CV sources are bipolar ±1.

**Shift-clear:** hold **B3** and turn any pot — that depth snaps to zero
(noon) and the pot decouples. The fast way to wipe a patch's modulation:
hold shift, brush every knob.

---

## Page 3 — Mod routing (teal-green)

Each pot is a **4-zone selector** choosing the mod source for the
same-numbered performance knob, CCW → CW:

| Zone | Source | Color | Character |
|---|---|---|---|
| 1 | Input follower | blue | unipolar 0..1 |
| 2 | Output follower | orange | unipolar 0..1 |
| 3 | CV in 1 (jack CV1) | violet | bipolar ±1 |
| 4 | CV in 2 (jack CV4) | mint | bipolar ±1 |

The ring always shows all four zones dim with the selected zone at full
brightness — it doubles as the position map. Default source is the input
follower, which makes the module self-modulating out of the box (depth is all
you need to dial in).

The two followers are the detectors' own smoothed TKEO envelopes (input side =
max of L/R; output side runs on the final post-mix mono sum). They share the
P3 threshold and the fixed smoothing, but the output follower never triggers
splices — it's a signal source only.

---

## Buttons

| | Idle | Action |
|---|---|---|
| **B1** | mode color | Force-trigger a splice on both channels (works even when P3 is at mute; suppressed while B2 or B3 is down). Dark blink on press. |
| **B2** | mode color | Click: next page. Held with B1 1.5 s: save settings. |
| **B3** | mode color | **SHIFT** — dark while held. Page-local: see below. Held with B1 0.5 s: reset current page to defaults. |

**Shift functions** (stored-value based — the pot decouples, nothing jumps):

- *Page 1:* hold + turn **P1** — pitch steps in exact integer semitones, ±1
  per ~1/24 of pot travel. The only reliable route back to true 0.
- *Page 2:* hold + turn **any pot** — that mod depth resets to zero.

---

## Jacks (bottom 2×5 grid)

Canonical names are panel positions: CV1–CV3 across the top row, CV4–CV6
across the bottom.

| | Left | | | | Right |
|---|---|---|---|---|---|
| top | **IN L** | **CV1** in — mod CV 1 | **CV2** out — in-gate | **CV3** out — out-gate | **OUT L** |
| bottom | **IN R** | **CV4** in — mod CV 2 | **CV5** out — in-env | **CV6** out — out-env | **OUT R** |

- **Inputs (left CV column):** eurorack bipolar, routable to any knob via
  page 3. *(CV1 read dead in bench tests under its former trig-in role — the
  1 Hz serial log prints all six raw CV values for diagnosis; unresolved.)*
- **Gate outs (top row):** 0/5 V kept-event monostables, ~20 ms,
  retriggerable — one per follower. These fire on *kept* transients, i.e.
  exactly what splices the engine (input side).
- **Envelope outs (bottom row):** the follower envelopes, 0–5 V — the same
  signals available internally as mod sources, mirrored on the P5/P6 pips.

Output is the P5 mix; there is no dry normalling in hardware.

---

## Fixed internals (former knobs, now constants in `audio_engine.cpp`)

| Constant | Value | What it was |
|---|---|---|
| `kFadeSamples` | 960 (20 ms) | OLA crossfade = latency = retrigger floor |
| `kTkeoFc` | 0.0014 (~34 Hz) | Follower smoothing — bench-picked midpoint of the old P5 sweep |
| analyzer ε | 0.001 | Keyframe quality threshold |
| hysteresis | 0.95 | Detector release ratio |
| `kFbFilterFc` | 0.02 (~480 Hz) | Feedback-path filter cutoff |
| boundary keyframes | gap-gated, fade/2 (~10 ms) | coverage guard frames — written only when no extremum has been logged recently (was: every block) |
