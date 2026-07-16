"""Deterministic per-consumer random streams.

Every component gets its own stream derived from (root seed, component name),
so runs are reproducible from the root seed alone and adding a component
never perturbs the randomness other components see.
"""

from __future__ import annotations

import hashlib

import numpy as np


def stream(root_seed: int, name: str) -> np.random.Generator:
    """Return an RNG stream unique to `name`, deterministic in `root_seed`.

    Independent of registration order: the stream key is a hash of the name,
    not a spawn counter.
    """
    digest = hashlib.sha256(name.encode("utf-8")).digest()
    key = int.from_bytes(digest[:8], "little")
    return np.random.default_rng(np.random.SeedSequence([root_seed, key]))
