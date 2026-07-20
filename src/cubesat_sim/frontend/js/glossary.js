/* cubesat shared js module: glossary — every term of art teaches itself
   on hover. GLOSS (term -> definition, set by the page before this
   module) feeds dotted-underline .term spans wrapped around matches in
   visible text; needs a #tooltip element in the page. Defines, at page
   scope: tooltip, termActive, defFor, glossify, placeTip, showTermTip.
   Inlined at render time by frontend/dashboard.py — edit this file, not
   the rendered pages. */
var tooltip = document.getElementById("tooltip");
var termActive = false;
var ALIAS = { "ground contact": "pass" };
var GLOSS_LC = {};
function normTerm(s) { return String(s).toLowerCase().replace(/-/g, " "); }
Object.keys(GLOSS).forEach(function (k) { GLOSS_LC[normTerm(k)] = k; });
function defFor(name) {
  var lc = normTerm(name);
  if (ALIAS[lc]) lc = normTerm(ALIAS[lc]);
  var k = GLOSS_LC[lc];
  return k ? { name: k, def: GLOSS[k] } : null;
}
function termRegex(keys, flags) {
  keys = keys.slice().sort(function (a, b) { return b.length - a.length; });
  var alts = keys.map(function (k) {
    return k.replace(/[.*+?^${}()|[\]\\]/g, "\\$&").replace(/[ -]/g, "[ -]");
  });
  // leading boundary is a captured group (no lookbehind for reach);
  // an optional trailing "s"/"es" lets "SEUs" and "passes" resolve
  return new RegExp("(^|[^A-Za-z0-9_-])(" + alts.join("|") +
                    ")((?:es|s)?)(?![A-Za-z0-9_-])", flags);
}
var _acr = [], _phr = [];
Object.keys(GLOSS).forEach(function (k) {
  (/[A-Z]/.test(k) ? _acr : _phr).push(k);
});
var reACR = _acr.length ? termRegex(_acr, "") : null;      // case matters
var rePHR = _phr.length ? termRegex(_phr, "i") : null;     // it doesn't

function glossifyNode(textNode) {
  var s = textNode.nodeValue, out = [], pos = 0, hits = 0;
  while (pos < s.length) {
    var rest = s.slice(pos), bm = null, bat = Infinity;
    [reACR, rePHR].forEach(function (re) {
      if (!re) return;
      var m = rest.match(re);
      if (m && m.index + m[1].length < bat) {
        bat = m.index + m[1].length; bm = m;
      }
    });
    if (!bm) break;
    var start = pos + bat, len = bm[2].length + bm[3].length;
    var d = defFor(bm[2]);
    if (d) {
      out.push(s.slice(pos, start));
      out.push({ text: s.slice(start, start + len), d: d });
      hits++;
    } else {
      out.push(s.slice(pos, start + len));
    }
    pos = start + len;
  }
  if (!hits) return;
  out.push(s.slice(pos));
  var frag = document.createDocumentFragment();
  out.forEach(function (o) {
    if (typeof o === "string") {
      if (o) frag.appendChild(document.createTextNode(o));
      return;
    }
    var sp = document.createElement("span");
    sp.className = "term"; sp.textContent = o.text;
    sp.dataset.name = o.d.name; sp.dataset.def = o.d.def;
    frag.appendChild(sp);
  });
  textNode.parentNode.replaceChild(frag, textNode);
}
function glossify(root) {
  if (!root) return;
  var walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT, null);
  var nodes = [];
  while (walker.nextNode()) {
    var p = walker.currentNode.parentNode;
    if (p.closest && p.closest(".term,.gloss,.evkind,.catchip,script,style")) continue;
    nodes.push(walker.currentNode);
  }
  nodes.forEach(glossifyNode);
}
function placeTip(ev) {
  var tw = tooltip.offsetWidth, th = tooltip.offsetHeight;
  var x = ev.clientX + 14, y = ev.clientY + 12;
  if (x + tw > window.innerWidth - 8) x = ev.clientX - tw - 14;
  if (y + th > window.innerHeight - 8) y = ev.clientY - th - 12;
  tooltip.style.left = x + "px"; tooltip.style.top = y + "px";
}
/* deliberately not the page's div() helper — the two pages disagree on
   its signature, and this module must work in both */
function ttPart(cls, text) {
  var d = document.createElement("div");
  d.className = cls; d.textContent = text;
  tooltip.appendChild(d);
}
function showTermTip(ev, name, def) {
  tooltip.textContent = "";
  ttPart("tt-t", name);
  ttPart("tt-d", def);
  tooltip.style.display = "block";
  placeTip(ev);
}
document.addEventListener("mouseover", function (ev) {
  var t = ev.target.closest && ev.target.closest(".term");
  if (t) { termActive = true; showTermTip(ev, t.dataset.name, t.dataset.def); }
});
document.addEventListener("mouseout", function (ev) {
  var t = ev.target.closest && ev.target.closest(".term");
  if (t) { termActive = false; tooltip.style.display = "none"; }
});
document.addEventListener("click", function (ev) {  // touch: tap toggles
  var t = ev.target.closest && ev.target.closest(".term");
  if (t) { termActive = true; showTermTip(ev, t.dataset.name, t.dataset.def); }
  else if (termActive) { termActive = false; tooltip.style.display = "none"; }
});
