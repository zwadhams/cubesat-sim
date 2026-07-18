"""Frontend: the flight-report dashboard and the live mission console.

Both viewers render self-contained HTML/CSS/JS (embedded here as Python
string templates) over the SQLite flight recordings the backend produces,
and they share one teaching layer — the GLOSSARY / EVENT_GLOSS term
definitions and the catalog-signature detectors in ``dashboard.py``.

    python -m cubesat_sim.frontend.dashboard runs/phase6_link.db
    python -m cubesat_sim.frontend.live --replay runs/phase6_link.db
"""
