# Capicola — Panel Reference

*Auto-slicer / keyframe time-stretcher, Alchemy Lab V2. Firmware 0.3.0, 2026-07-27.*

> **Interactive version:**
> [heavylight-industries.github.io/capicola/manual.html](https://heavylight-industries.github.io/capicola/manual.html)
> — the same reference drawn on the real faceplate, with live LED state per page.

Capicola listens constantly. Two independent channels (L/R) each run a transient
detector over the input; every detected transient — or a press of **B2**, or a
rising edge on **TRIG IN** — splices the granular playback head onto fresh
material. Pitch and time-stretch are fully decoupled, and the stretch grid
re-anchors on every trigger, so stretched tails ring out underneath while the
output stays locked to the incoming rhythm.

The wet path's latency is the OLA crossfade: **20 ms** by default, adjustable
10–250 ms on the secondary page. The fade is also the retrigger floor —
splices can't come faster.

---

## Getting around

Two page groups, each rotated by a single press of its own button. Each group
remembers where it was left: pressing the *other* group's button jumps straight
to that group's remembered page, and the next press of the same button advances
within the group.

| Button | Pages (in order) | Mode colors |
|---|---|---|
| **B1** | Perf → Depth | **orange** → **pink** |
| **B3** | Routing → Secondary | **blue** → **teal** |

The active page's mode color is worn simultaneously by all three button LEDs.
Button LEDs are never off, so activity feedback is a **dark flash**: B2 blinks
off when a trigger fires, and all three blink off three times to confirm a
settings save or reset.

**Pot catch:** every page keeps stored values per knob. After boot or a page
switch, a knob is *decoupled* until the physical pot sweeps through the stored
value (the ring shows a muted pip at the stored position) — then it catches and
tracks. Nothing ever jumps. On the routing page the catch is zone-granular:
entering the stored zone is enough.

**Settings:** hold **B2 for 1.5 s** to save everything (all four pages) as the
boot state — QSPI preset slot 0, confirmed by the triple dark blink. (The press
still fires its slice on the way down.) On boot the module restores that save,
or falls back to factory defaults. Hold **B1+B3 for 0.5 s** to reset the
*currently visible page only* to factory defaults (same blink; the QSPI save is
untouched until the next B2 hold).

---

## Perf (B1 · page 1 · orange)

| Knob | Function | Range | Notes |
|---|---|---|---|
| P1 | Pitch | ±12 semitones | Unity at noon, with a ±0.2 st detent that snaps to true 0. Grain pitch only — time is untouched. |
| P2 | Stretch | 1.0 → frozen | CCW = realtime; noon ≈ 6× slow; full CW is a **true freeze** (the grid stops). Taper is (1−x)^2.5. |
| P3 | Threshold | ratio 0–8 | See below — it's relative, not absolute. |
| P4 | Grain size | 32–4096 keyframes | The adaptive splice leash. |
| P5 | Quality | ε 0.1 → 0.001 | Analyzer keyframe threshold. CCW = crunchy sparse reconstruction, CW = max fidelity. |
| P6 | Feedback | 0–1.5 loop gain | See below. |

**P3 — threshold is self-calibrating.** The detector tracks a running average
of its own envelope and keeps events at `knob × average` — a *ratio*, not an
absolute level, so it needs no gain-staging and survives level changes. The
first 90% of the sweep spans ratio 0 (keep everything) to 4 (transients only);
the last stretch climbs to 8 (very picky). **The very top of the knob
hard-mutes auto-triggering** — the module keeps playing its current material
and only B2 or TRIG IN splices in new audio.

**P6 — feedback.** The previous block's *wet* output → sinc saturator (table
lookup, unity small-signal gain) → state-variable **bandpass** (centre set by
the secondary page's P6, ~480 Hz by default) → back into the recorder input,
where it gets re-sliced and re-pitched. The knob is loop gain: **above 1.0 is
deliberately unstable** (chaos zone), with a hard ±1 clamp on the injection as
the runaway guard. The feedback tap sits *before* the mix blend, so turning mix
down doesn't starve the loop.

**6-o'clock pips** (always on, every page): a live mirror of the six jacks.
P1/P2 show the two inputs (TRIG IN, CV IN) in bipolar colors — green positive,
red negative. P3–P6 mirror the four outputs — in-gate (P3, also lights on a B2
press), out-gate (P4), in-envelope (P5, turns red when the input clips),
out-envelope (P6).

---

## Depth (B1 · page 2 · pink)

Each pot is the **bipolar modulation depth** for the same-numbered performance
knob: noon = off, CW (pink) = positive, CCW (orange) = negative. A faint
page-hued glow marks a centered (inactive) depth. B1+B3 zeroes the whole page.

Modulation is applied as an offset to the perf knob's *position* —
`source × depth` — then runs through that knob's normal range mapping. Follower
sources are unipolar 0..1; the CV IN jack is bipolar ±1.

---

## Routing (B3 · page 1 · blue)

Each pot is a **3-zone selector** choosing the mod source for the same-numbered
performance knob, CCW → CW:

| Zone | Source | Color | Character |
|---|---|---|---|
| 1 | Input follower | orange | unipolar 0..1 |
| 2 | Output follower | blue | unipolar 0..1 |
| 3 | CV IN jack | violet | bipolar ±1 |

The ring always shows all three zones dim with the selected zone at full
brightness — it doubles as the position map. Default source is the **output**
follower, which makes the module self-modulating out of the box — and since
the output envelope reflects what the module itself is doing, depth alone
closes a feedback loop through the modulation.

The two followers are the detectors' own smoothed TKEO envelopes (input side =
max of L/R; output side runs on the final post-mix mono sum). They share the
P3 threshold and the smoothing, but the output follower never triggers
splices — it's a signal source only.

---

## Secondary (B3 · page 2 · teal)

Deeper voicing controls. Saved with everything else in the preset store.

| Knob | Function | Range | Default |
|---|---|---|---|
| P1 | Envelope smoothing | follower cutoff, exp — ~1 Hz at CCW | ~34 Hz |
| P2 | Fade | OLA crossfade 10–250 ms, exp — latency, retrigger floor and punch length in one | 20 ms |
| P3 | Drive | keyframe shaper gain 0.5–4.0 | 1.0 |
| P4 | Drive character | quake ← clean → sinc | full sinc |
| P5 | Mix | dry/wet | full wet |
| P6 | Feedback tone | feedback bandpass centre, exp | ~480 Hz |

**P3/P4 — the keyframe waveshaper.** Drive is applied to the *keyframes
themselves*, before interpolation — the nonlinearity never sees the sample
rate, so it can't fold aliases into the band no matter how hard you push it.
Character morphs the shape: full CCW is the quake curve (saturates hard but
drops out near silence — gated, gnarly), noon is clean bypass, full CW is the
sinc shaper (the default coloration).

**P2 — fade.** One knob, three tied consequences: the wet path's latency, the
splice crossfade length, and the retrigger floor. Short = snappy, clicky
splices at high rates; long = smeared, washy takeovers that cap the splice
rate at a few Hz. Changing it never moves a playing grain — the new length
applies from the next splice on.

**P5 — mix.** The wet path lags the dry by the fade, so mid-mix settings
comb/phase against sustained material — expected, and a sound of its own at
unity pitch. Full wet is the classic Capicola behavior.

---

## Buttons

| | Idle | Action |
|---|---|---|
| **B1** | mode color | Click: perf ↔ depth (or return to the B1 group). Held with B3 0.5 s: reset visible page. |
| **B2** | mode color | Press: force-trigger a splice on both channels (works even at threshold mute). Held 1.5 s: save settings. Dark blink on trigger. |
| **B3** | mode color | Click: routing ↔ secondary (or return to the B3 group). |

---

## Jacks (bottom 2×5 grid)

| | Left | | | | Right |
|---|---|---|---|---|---|
| top | **IN L** | **TRIG IN** | **TRIG 1** out — in-gate | **TRIG 2** out — out-gate | **OUT L** |
| bottom | **IN R** | **CV IN** | **ENV 1** out — in-env | **ENV 2** out — out-env | **OUT R** |

- **TRIG IN:** external slice trigger — rising edge (fires ~+2 V, re-arms
  ~+1 V), always accepted, even at threshold mute. Shares B2's dark flash.
- **CV IN:** the external mod source, eurorack bipolar, routable to any knob
  via the routing page.
- **Trig outs:** 0/5 V kept-event monostables, ~20 ms, retriggerable — these
  fire on *kept* transients, i.e. exactly what splices the engine (input side).
- **Env outs:** the follower envelopes, 0–5 V — the same signals available
  internally as mod sources, mirrored on the P5/P6 pips.

Output is the mix; there is no dry normalling in hardware.

---

## Fixed internals

| Constant | Value | What it is |
|---|---|---|
| `kFbSatGain` | 0.5 | feedback saturator scaling (unity small-signal loop gain) |
| `kMaxDriftSamples` | 48000 (1 s) | stereo re-alignment guardrail — if one channel auto-fires and the grids have drifted > 1 s, the quiet channel is forced to re-anchor |
| boundary keyframes | gap-gated, fade/2 | coverage guard frames, written only when no extremum has been logged recently |
