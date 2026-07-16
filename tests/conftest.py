import shutil
import subprocess
from pathlib import Path

import pytest

from cubesat_sim.mission import (
    C_EPS_BIN,
    C_OBC_BIN,
    CPP_COMMS_BIN,
    REPO_ROOT,
    RUST_ADCS_BIN,
)


@pytest.fixture(scope="session")
def c_obc_binary():
    if shutil.which("make") is None or shutil.which("cc") is None:
        pytest.skip("no C toolchain available")
    subprocess.run(["make", "-C", str(C_OBC_BIN.parent)],
                   check=True, capture_output=True)
    return C_OBC_BIN


@pytest.fixture(scope="session")
def c_eps_binary():
    if shutil.which("make") is None or shutil.which("cc") is None:
        pytest.skip("no C toolchain available")
    subprocess.run(["make", "-C", str(C_EPS_BIN.parent)],
                   check=True, capture_output=True)
    return C_EPS_BIN


@pytest.fixture(scope="session")
def cpp_comms_binary():
    if shutil.which("make") is None or shutil.which("c++") is None:
        pytest.skip("no C++ toolchain available")
    subprocess.run(["make", "-C", str(CPP_COMMS_BIN.parent)],
                   check=True, capture_output=True)
    return CPP_COMMS_BIN


@pytest.fixture(scope="session")
def rust_adcs_binary():
    cargo = shutil.which("cargo") or str(Path.home() / ".cargo" / "bin" / "cargo")
    if not Path(cargo).exists():
        pytest.skip("no Rust toolchain available")
    subprocess.run([cargo, "build", "--release"],
                   cwd=REPO_ROOT / "rust" / "adcs",
                   check=True, capture_output=True)
    return RUST_ADCS_BIN
