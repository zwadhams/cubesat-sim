/* cubesat shared js module: glyph — the severity language every viewer
   speaks: critical ▲, warning ◆, good/recovery ●, neutral ○ — always
   shape + color, never color alone (CVD-safe). SEV values are CSS var()
   strings: fine for DOM/SVG styling, not for canvas fillStyle (resolve
   those through getComputedStyle). Inlined at render time by
   frontend/dashboard.py. */
var SEV = { critical: "var(--critical)", warning: "var(--warning)",
            good: "var(--good)", neutral: "var(--muted)" };
function sevCss(sev) { return SEV[sev] || SEV.neutral; }
var SEV_GLYPH = { critical: "▲", warning: "◆", good: "●", neutral: "○" };
function sevGlyph(sev) { return SEV_GLYPH[sev] || SEV_GLYPH.neutral; }
