"""
Pytest configuration for VHDL testbenches.

This module provides infrastructure to:
1. Discover VHDL testbenches in subdirectories
2. Run simulations and parse their output
3. Report individual test results from each testbench
"""

import subprocess
import re
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

import pytest

# Paths are relative to the tests root (where conftest.py lives)
VHDL_TESTBENCHES_TO_RUN = [
    "i2c_cbor/full_test",
    # "spi_cbor/full_test"
]

@dataclass
class VhdlTestResult:
    """Result of a single test within a VHDL testbench."""
    number: int
    name: str
    passed: bool
    raw_line: str


@dataclass
class VhdlSimulationResult:
    """Result of running a complete VHDL simulation."""
    testbench_path: Path
    return_code: int
    stdout: str
    stderr: str
    tests: list[VhdlTestResult]
    timed_out: bool = False


def parse_vhdl_test_output(output: str) -> list[VhdlTestResult]:
    """
    Parse VHDL testbench output to extract individual test results.

    Matches lines like:
      @123ns [INF] ======== Test #1 PASS: Test Name
      @456ns [ERR] ======== Test #2 FAIL: Test Name

    ANSI escape codes are stripped before parsing.
    """
    # Strip ANSI escape codes
    ansi_escape = re.compile(r'\x1b\[[0-9;]*m')
    clean_output = ansi_escape.sub('', output)

    # Match test result lines
    # Format: @<time> [INF/ERR] ======== Test #N PASS/FAIL: <name>
    pattern = re.compile(
        r'@[^\[]+\[(INF|ERR)\]\s+========\s+Test\s+#(\d+)\s+(PASS|FAIL):\s+(.+?)(?:\s*$)',
        re.MULTILINE
    )

    results = []
    for match in pattern.finditer(clean_output):
        level, num, status, name = match.groups()
        results.append(VhdlTestResult(
            number=int(num),
            name=name.strip(),
            passed=(status == "PASS"),
            raw_line=match.group(0)
        ))

    return results


def run_vhdl_simulation(testbench_dir: Path, timeout: int = 300) -> VhdlSimulationResult:
    """
    Build and run a VHDL simulation.

    Args:
        testbench_dir: Path to directory containing the testbench Makefile
        timeout: Maximum time to wait for simulation (seconds)

    Returns:
        VhdlSimulationResult with parsed test results
    """
    # First, build the testbench
    build_result = subprocess.run(
        ["make"],
        cwd=testbench_dir,
        capture_output=True,
        text=True,
        timeout=timeout
    )

    if build_result.returncode != 0:
        return VhdlSimulationResult(
            testbench_path=testbench_dir,
            return_code=build_result.returncode,
            stdout=build_result.stdout,
            stderr=build_result.stderr,
            tests=[],
            timed_out=False
        )

    # Find the executable (usually named 'tb')
    tb_executable = testbench_dir / "tb"
    if not tb_executable.exists():
        # Try to find any executable
        for candidate in testbench_dir.iterdir():
            if candidate.is_file() and candidate.stat().st_mode & 0o111:
                if candidate.suffix not in ['.vhd', '.ghw', '.gtkw', '.mk', '.so']:
                    tb_executable = candidate
                    break

    if not tb_executable.exists():
        return VhdlSimulationResult(
            testbench_path=testbench_dir,
            return_code=-1,
            stdout="",
            stderr=f"No testbench executable found in {testbench_dir}",
            tests=[],
            timed_out=False
        )

    # Run the simulation
    timed_out = False
    try:
        result = subprocess.run(
            [str(tb_executable)],
            cwd=testbench_dir,
            capture_output=True,
            text=True,
            timeout=timeout
        )
        return_code = result.returncode
        stdout = result.stdout
        stderr = result.stderr
    except subprocess.TimeoutExpired as e:
        timed_out = True
        return_code = -1
        stdout = e.stdout.decode() if e.stdout else ""
        stderr = e.stderr.decode() if e.stderr else ""

    # Parse test results from output
    tests = parse_vhdl_test_output(stdout)

    return VhdlSimulationResult(
        testbench_path=testbench_dir,
        return_code=return_code,
        stdout=stdout,
        stderr=stderr,
        tests=tests,
        timed_out=timed_out
    )


def discover_vhdl_testbenches(base_dir: Path) -> list[Path]:
    """
    Discover VHDL testbench directories, filtered by VHDL_TESTBENCHES_TO_RUN.
    """
    selected = {
        (base_dir / Path(p)).resolve()
        for p in VHDL_TESTBENCHES_TO_RUN
    }

    testbenches = []

    for makefile in base_dir.rglob("Makefile"):
        tb_dir = makefile.parent.resolve()

        if tb_dir == base_dir:
            continue

        if tb_dir.name == "src":
            continue

        src_dir = tb_dir / "src"
        if not src_dir.exists():
            continue

        if not list(src_dir.glob("*.vhd")):
            continue

        if selected and tb_dir not in selected:
            continue

        testbenches.append(tb_dir)

    missing = selected - set(testbenches)
    if missing:
        raise RuntimeError(
            "Configured VHDL testbenches not found:\n"
            + "\n".join(str(p) for p in missing)
        )

    return sorted(testbenches)

    # """
    # Discover VHDL testbench directories.

    # A valid testbench directory contains:
    # - A Makefile
    # - A src/ subdirectory with tb.vhd or similar
    # """
    # testbenches = []

    # for makefile in base_dir.rglob("Makefile"):
    #     tb_dir = makefile.parent

    #     # Skip if this is the top-level tests Makefile
    #     if tb_dir == base_dir:
    #         continue

    #     # Skip src/ directories (those contain source Makefiles, not build Makefiles)
    #     if tb_dir.name == "src":
    #         continue

    #     # Check if this looks like a testbench directory
    #     # It should have a src/ subdir with VHDL files
    #     src_dir = tb_dir / "src"
    #     if src_dir.exists():
    #         vhdl_files = list(src_dir.glob("*.vhd"))
    #         if vhdl_files:
    #             testbenches.append(tb_dir)

    # return sorted(testbenches)


# Cache for simulation results to avoid re-running
_simulation_cache: dict[Path, VhdlSimulationResult] = {}


def get_simulation_result(testbench_dir: Path, timeout: int = 300) -> VhdlSimulationResult:
    """Get or compute simulation result (cached)."""
    if testbench_dir not in _simulation_cache:
        _simulation_cache[testbench_dir] = run_vhdl_simulation(testbench_dir, timeout)
    return _simulation_cache[testbench_dir]


def clear_simulation_cache():
    """Clear the simulation result cache."""
    _simulation_cache.clear()


# Pytest hooks and fixtures

def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line(
        "markers", "vhdl: mark test as a VHDL simulation test"
    )


@pytest.fixture(scope="session")
def tests_dir():
    """Return the base tests directory."""
    return Path(__file__).parent


@pytest.fixture(scope="session")
def vhdl_testbenches(tests_dir):
    """Discover all VHDL testbenches."""
    return discover_vhdl_testbenches(tests_dir)
