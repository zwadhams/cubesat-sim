/* cubesat shared js module: theme — cycles auto -> light -> dark on the
   #themebtn, persisted under THEME_KEY (set by the page before this
   module, so each viewer remembers its own choice). file:// may deny
   storage — degrade to per-load. Inlined at render time by
   frontend/dashboard.py. */
(function () {
  var modes = ["auto", "light", "dark"];
  var btn = document.getElementById("themebtn"), cur = "auto";
  try { cur = localStorage.getItem(THEME_KEY) || "auto"; } catch (e) {}
  function apply() {
    if (cur === "auto") document.documentElement.removeAttribute("data-theme");
    else document.documentElement.setAttribute("data-theme", cur);
    btn.textContent = "theme: " + cur;
  }
  btn.addEventListener("click", function () {
    cur = modes[(modes.indexOf(cur) + 1) % 3];
    try { localStorage.setItem(THEME_KEY, cur); } catch (e) {}
    apply();
  });
  apply();
})();
