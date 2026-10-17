#!/usr/bin/env node
/* manual_sync.mjs — the interactive manual is the one source of Capicola's
 * prose. This evaluates docs/manual.html's <script id="manual-data"> block (it
 * never touches the DOM), turns each entry's HTML into the one layer of
 * markdown the HostLink descriptor carries, and writes
 * src/capicola_manual_text.h for capicola_manual.cpp. CMake reruns it whenever
 * the manual changes and node is available; the header is checked in so a
 * build without node still works.
 *
 *   node tools/manual_sync.mjs          write the header
 *   node tools/manual_sync.mjs --check  exit 1 if the header is stale
 */
import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import vm from "node:vm";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const htmlPath = join(root, "docs/manual.html");
const outPath = join(root, "src/capicola_manual_text.h");

const html = readFileSync(htmlPath, "utf8");
const block = html.match(/<script id="manual-data">([\s\S]*?)<\/script>/);
if (!block) throw new Error("no <script id=\"manual-data\"> in docs/manual.html");
const M = vm.runInNewContext(
  block[1] + "\n;({ TAGLINE, PREAMBLE, HISTORY, SNAP_TARGET, JACK_MODE, JACK_OUTS, BUTTONS, JACKS, VIEWS })");

/* HTML prose → markdown. Only the tags the data block's header allows; any
 * other tag is an error, so the firmware never ships raw markup. */
function md(h) {
  let s = h.replace(/\s+/g, " ");
  s = s.replace(/<a [^>]*href="(https?:[^"]+)"[^>]*>(.*?)<\/a>/g, "[$2]($1)")   // web links stay links
       .replace(/<a [^>]*>(.*?)<\/a>/g, "$1")                    // in-manual links read as text
       .replace(/<h4>(.*?)<\/h4>/g, "\n\n**$1**\n\n")
       .replace(/<ul>/g, "\n\n").replace(/<\/ul>/g, "\n\n")
       .replace(/<li>\s*(.*?)\s*<\/li>/g, "\n- $1")
       .replace(/<div class="legend"[^>]*>/g, "\n\n").replace(/<\/div>/g, "\n\n")
       .replace(/<span><i [^>]*><\/i>(.*?)<\/span>/g, "\n- $1")
       .replace(/<p>/g, "\n\n").replace(/<\/p>/g, "\n\n")
       .replace(/<strong>(.*?)<\/strong>/g, "**$1**")
       .replace(/<em>(.*?)<\/em>/g, "*$1*")
       .replace(/<code>(.*?)<\/code>/g, "`$1`")
       .replace(/<kbd>(.*?)<\/kbd>/g, "$1")                      // key names: plain, may sit inside bold
       .replace(/<sup>(.*?)<\/sup>/g, "^$1");
  const left = s.match(/<\/?[a-z][^>]*>/i);
  if (left) throw new Error(`unconverted tag ${left[0]} in: ${h.slice(0, 80)}`);
  s = s.replace(/&amp;/g, "&").replace(/&lt;/g, "<").replace(/&gt;/g, ">");
  return s.split(/\n{2,}/)
          .map(par => par.split("\n").map(l => l.trim()).filter(Boolean).join("\n"))
          .filter(Boolean).join("\n\n");
}

/* C string literal, wrapped at word boundaries for a readable diff. */
function lit(text, indent = "    ") {
  const esc = text.replace(/\\/g, "\\\\").replace(/"/g, '\\"').replace(/\n/g, "\\n");
  const parts = [];
  let rest = esc;
  while (rest.length > 68) {
    let cut = rest.lastIndexOf(" ", 68);
    if (cut < 20) cut = 68;
    parts.push(rest.slice(0, cut + 1));
    rest = rest.slice(cut + 1);
  }
  parts.push(rest);
  return parts.map(p => `${indent}"${p}"`).join("\n");
}

const out = [];
const one = (name, text) => out.push(`inline constexpr const char* ${name} =\n${lit(md(text))};\n`);
const many = (name, texts) => out.push(
  `inline constexpr const char* ${name}[${texts.length}] = {\n` +
  texts.map(t => lit(md(t)) + ",").join("\n") + "\n};\n");
const names = (name, texts) => out.push(
  `inline constexpr const char* ${name}[${texts.length}] = {\n` +
  texts.map(t => `    "${t}",`).join("\n") + "\n};\n");

const V = M.VIEWS, G = id => V.gestures.items.find(it => it.id === id);

/* An entry's text for the programmer: its body, then its facts rows as a list
 * — hold times, pulse widths, pips, detents: details the programmer can't
 * derive. Rows it already shows are skipped: a knob field's Range and Default
 * come from its declaration, a jack's Range from its signal type. */
const KNOB_SKIP = ["Range", "Default"], OTHER_SKIP = ["Range"];
function full(entry, skip = OTHER_SKIP) {
  const rows = (entry.facts || []).filter(([k]) => !skip.includes(k));
  if (!rows.length) return entry.body;
  return entry.body + "<ul>" +
    rows.map(([k, v]) => `<li><strong>${k}:</strong> ${v}</li>`).join("") + "</ul>";
}
const knob = k => full(k, KNOB_SKIP);

one("kTagline", M.TAGLINE);
one("kPreamble", M.PREAMBLE);
out.push("/* Pages with a firmware Page object */");
one("kPrimaryPage", V.perf.overview);
one("kDepthPage", V.depth.overview);
names("kPrimaryName", V.perf.knobs.map(k => k.l));
many("kPrimaryKnob", V.perf.knobs.map(knob));
names("kDepthName", V.depth.knobs.map(k => k.l));
many("kDepthKnob", V.depth.knobs.map(knob));
out.push("/* Routing and Secondary are preset components: per-field names and help */");
names("kRouteName", V.routing.knobs.map(k => k.l));
many("kRouteHelp", V.routing.knobs.map(knob));
names("kSecName", V.secondary.knobs.map(k => k.l));
many("kSecHelp", V.secondary.knobs.map(knob));
out.push("/* Manual sections: the overviews with no entity to hang on, and the\n" +
         " * gestures that belong to no button. */");
one("kRoutingOverview", V.routing.overview);
one("kSecondaryOverview", V.secondary.overview);
/* The same overviews as ready-made "pageHelp" JSON arrays, for the two
 * settings components' "About this page". */
const pageHelp = (name, html) => out.push(
  `inline constexpr const char* ${name} =\n${lit(JSON.stringify([md(html)]))};\n`);
pageHelp("kRoutingPageHelpJson", V.routing.overview);
pageHelp("kSecondaryPageHelpJson", V.secondary.overview);
one("kGesturesOverview", V.gestures.overview);
one("kPotCatch", full(G("catch")));
one("kSlicing", full(G("slice")));
one("kReadingPanel", full(G("panel")));
out.push("/* Buttons, jacks (panel grid order: top row L→R, then bottom row) */");
/* Buttons: name and one sentence. The programmer lists each button's gestures
 * as chips, so the facts would only repeat them. */
names("kButtonName", M.BUTTONS.map(b => b.l));
many("kButton", M.BUTTONS.map(b => b.body));
/* Each button's first facts row is its plain press ("Click" / "Tap"). */
many("kButtonTap", M.BUTTONS.map(b => b.facts[0][1]));
many("kJack", M.JACKS.map(j => full(j)));
out.push("/* Button gestures */");
/* A gesture chip gets its first paragraph and its facts. The longer paragraphs
 * — the save history, the snap target — are the presets help and the Options
 * help in the programmer already. */
const chip = g => full({ body: g.body.split("</p>")[0] + "</p>", facts: g.facts });
one("kGestureReset", chip(G("reset")));
one("kGestureSave", chip(G("save")));
one("kGestureLoad", chip(G("load")));
one("kGestureFlip", chip(G("pingpong")));
one("kGestureSnap", chip(G("snap")));
out.push("/* Shared paragraphs: the presets panel and the Options fields */");
one("kHistory", M.HISTORY);
one("kSnapTarget", M.SNAP_TARGET);
one("kJackMode", M.JACK_MODE);
one("kJackOuts", M.JACK_OUTS);

const header =
`/* GENERATED by tools/manual_sync.mjs from docs/manual.html — do not edit.
 * Change the manual's <script id="manual-data"> block and rebuild. */
#pragma once

namespace manual_text {

${out.join("\n")}
} // namespace manual_text
`;

if (process.argv.includes("--check")) {
  let cur = "";
  try { cur = readFileSync(outPath, "utf8"); } catch {}
  if (cur !== header) { console.error("src/capicola_manual_text.h is stale"); process.exit(1); }
} else {
  writeFileSync(outPath, header);
}
