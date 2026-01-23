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
    "spi_cbor/full_test"
]

@dataclass
class VhdlTestResult:
    """Result of a single test within a VHDL testbench."""
    number: int
    name: str
    passed: bool
    raw_line: str
    log_context: str = ""  # Log lines around this test


@dataclass
class VhdlSimulationResult:
    """Result of running a complete VHDL simulation."""
    testbench_path: Path
    return_code: int
    stdout: str
    stderr: str
    tests: list[VhdlTestResult]
    timed_out: bool = False


def strip_ansi(text: str) -> str:
    """Remove ANSI escape codes from text."""
    ansi_escape = re.compile(r'\x1b\[[0-9;]*m')
    return ansi_escape.sub('', text)


def extract_test_log_context(output: str, test_number: int, context_lines: int = 20) -> str:
    """
    Extract log lines around a specific test result.

    Returns lines from the previous test result (or start) up to and including
    this test's result line, plus a few lines after for context.
    """
    lines = output.split('\n')

    # Find the line with this test's result
    test_pattern = re.compile(rf'========\s+Test\s+#{test_number}\s+(PASS|FAIL):')
    prev_test_pattern = re.compile(r'========\s+Test\s+#(\d+)\s+(PASS|FAIL):')

    result_line_idx = None
    prev_test_line_idx = 0

    for i, line in enumerate(lines):
        if test_pattern.search(line):
            result_line_idx = i
            break
        # Track where the previous test ended
        match = prev_test_pattern.search(line)
        if match and int(match.group(1)) == test_number - 1:
            prev_test_line_idx = i + 1

    if result_line_idx is None:
        return ""

    # Extract from previous test to this test + a few lines
    start_idx = max(0, prev_test_line_idx)
    end_idx = min(len(lines), result_line_idx + 5)

    return '\n'.join(lines[start_idx:end_idx])


def parse_vhdl_test_output(output: str) -> list[VhdlTestResult]:
    """
    Parse VHDL testbench output to extract individual test results.

    Matches lines like:
      @123ns [INF] ======== Test #1 PASS: Test Name
      @456ns [ERR] ======== Test #2 FAIL: Test Name

    ANSI escape codes are stripped before parsing.
    """
    clean_output = strip_ansi(output)

    # Match test result lines
    # Format: @<time> [INF/ERR] ======== Test #N PASS/FAIL: <name>
    pattern = re.compile(
        r'@[^\[]+\[(INF|ERR)\]\s+========\s+Test\s+#(\d+)\s+(PASS|FAIL):\s+(.+?)(?:\s*$)',
        re.MULTILINE
    )

    results = []
    for match in pattern.finditer(clean_output):
        level, num, status, name = match.groups()
        test_num = int(num)
        results.append(VhdlTestResult(
            number=test_num,
            name=name.strip(),
            passed=(status == "PASS"),
            raw_line=match.group(0),
            log_context=extract_test_log_context(clean_output, test_num)
        ))

    return results


def get_reports_dir() -> Path:
    """Get the reports directory, creating it if needed."""
    reports_dir = Path(__file__).parent / "reports" / "logs"
    reports_dir.mkdir(parents=True, exist_ok=True)
    return reports_dir


def save_simulation_log(testbench_dir: Path, stdout: str, stderr: str) -> Path:
    """Save simulation output to a log file."""
    reports_dir = get_reports_dir()

    # Create a filename from the testbench path (e.g., i2c_cbor_full_test.log)
    base_dir = Path(__file__).parent
    try:
        rel_path = testbench_dir.relative_to(base_dir)
        log_name = str(rel_path).replace("/", "_").replace("\\", "_") + ".log"
    except ValueError:
        log_name = testbench_dir.name + ".log"

    log_path = reports_dir / log_name

    # Write the log with both stdout and stderr
    with open(log_path, 'w') as f:
        f.write("=" * 70 + "\n")
        f.write(f"VHDL Simulation Log: {testbench_dir}\n")
        f.write("=" * 70 + "\n\n")

        f.write("-" * 70 + "\n")
        f.write("STDOUT:\n")
        f.write("-" * 70 + "\n")
        f.write(strip_ansi(stdout) if stdout else "(empty)\n")

        if stderr:
            f.write("\n" + "-" * 70 + "\n")
            f.write("STDERR:\n")
            f.write("-" * 70 + "\n")
            f.write(strip_ansi(stderr))

    return log_path


def run_vhdl_simulation(testbench_dir: Path, timeout: int = 300) -> VhdlSimulationResult:
    """
    Build and run a VHDL simulation.

    Args:
        testbench_dir: Path to directory containing the testbench Makefile
        timeout: Maximum time to wait for simulation (seconds)

    Returns:
        VhdlSimulationResult with parsed test results
    """
    print(f"[VHDL] Building and running testbench in {testbench_dir}")

    # Build and run the testbench
    # Using 'make run' ensures simulation runs even if already built
    build_result = subprocess.run(
        ["make", "run"],
        cwd=testbench_dir,
        capture_output=True,
        text=True,
        timeout=timeout
    )

    # Save the full simulation log to a file
    log_path = save_simulation_log(testbench_dir, build_result.stdout, build_result.stderr)
    print(f"[VHDL] Log saved to: {log_path}")

    # Try to parse test results from make output even if it returned non-zero,
    # because make returns non-zero both for build failures AND simulation failures
    # (when tests fail, simulation exits non-zero, causing make to fail)
    tests = parse_vhdl_test_output(build_result.stdout)
    if not tests and build_result.stderr:
        tests = parse_vhdl_test_output(build_result.stderr)

    if build_result.returncode != 0:
        if tests:
            # Simulation ran but some tests failed - this is fine, we have results
            print(f"[VHDL] Build/simulation finished (exit code {build_result.returncode}), parsed {len(tests)} test results")
            return VhdlSimulationResult(
                testbench_path=testbench_dir,
                return_code=build_result.returncode,
                stdout=build_result.stdout,
                stderr=build_result.stderr,
                tests=tests,
                timed_out=False
            )
        else:
            # Actual build failure - no tests parsed
            print(f"[VHDL] Build FAILED (exit code {build_result.returncode})")
            print(f"[VHDL] stderr: {build_result.stderr[:500]}")
            return VhdlSimulationResult(
                testbench_path=testbench_dir,
                return_code=build_result.returncode,
                stdout=build_result.stdout,
                stderr=build_result.stderr,
                tests=[],
                timed_out=False
            )

    # Make succeeded and simulation ran - parse and return results
    # (make already ran the simulation as part of its default target)
    print(f"[VHDL] Build and simulation OK, parsed {len(tests)} test results")

    return VhdlSimulationResult(
        testbench_path=testbench_dir,
        return_code=build_result.returncode,
        stdout=build_result.stdout,
        stderr=build_result.stderr,
        tests=tests,
        timed_out=False
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


# Test collection for VHDL tests

@dataclass
class VhdlTestInfo:
    """Information about a single VHDL test for pytest."""
    testbench_path: Path
    classname: str
    test_number: Optional[int]
    test_name: Optional[str]
    passed: bool
    log_context: str
    full_log: str


def collect_vhdl_tests() -> list[VhdlTestInfo]:
    """
    Collect all VHDL tests with their log context.
    """
    base_dir = Path(__file__).parent
    testbenches = discover_vhdl_testbenches(base_dir)
    all_tests = []

    for tb_path in testbenches:
        # Find the configured relative path for reporting
        rel_path = next((p for p in VHDL_TESTBENCHES_TO_RUN if p in str(tb_path)))
        # Use dots for classname (Java package style, what Jenkins expects)
        classname = rel_path.replace("/", ".")

        result = get_simulation_result(tb_path)
        full_log = strip_ansi(result.stdout)

        if result.tests:
            for t in result.tests:
                all_tests.append(VhdlTestInfo(
                    testbench_path=tb_path,
                    classname=classname,
                    test_number=t.number,
                    test_name=t.name,
                    passed=t.passed,
                    log_context=t.log_context,
                    full_log=full_log
                ))
        else:
            # No individual tests -> treat whole testbench as a single test
            passed = result.return_code == 0 and not result.timed_out
            all_tests.append(VhdlTestInfo(
                testbench_path=tb_path,
                classname=classname,
                test_number=None,
                test_name=None,
                passed=passed,
                log_context="",
                full_log=full_log
            ))

    return all_tests


def pytest_generate_tests(metafunc):
    """Parametrize vhdl_test fixture with all discovered VHDL tests."""
    if "vhdl_test" in metafunc.fixturenames:
        all_tests = collect_vhdl_tests()
        metafunc.parametrize(
            "vhdl_test",
            all_tests,
            ids=[
                f"{t.classname}::{t.test_number:02d}_{t.test_name.replace(' ', '_')}"
                if t.test_name else f"{t.classname}::simulation"
                for t in all_tests
            ]
        )
