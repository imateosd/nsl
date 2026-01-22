"""
VHDL testbench integration with pytest.

This module runs VHDL simulations and reports individual test results.

Usage:
    pytest                           # Run all VHDL tests
    pytest -k i2c_cbor               # Run only i2c_cbor tests
    pytest --tb=short                # Show short tracebacks
    pytest -v                        # Verbose output

To add a new testbench:
    1. Create a directory with Makefile and src/tb.vhd
    2. Ensure tb.vhd uses log_test_result() for each test
    3. The testbench will be auto-discovered
"""

import pytest
from pathlib import Path

from conftest import (
    discover_vhdl_testbenches,
    get_simulation_result,
    VhdlSimulationResult,
)


def get_testbench_id(tb_path: Path, base_dir: Path) -> str:
    """Generate a readable ID for a testbench."""
    try:
        relative = tb_path.relative_to(base_dir)
        return str(relative).replace("/", "::")
    except ValueError:
        return tb_path.name


def collect_vhdl_tests():
    """
    Collect all VHDL tests by discovering testbenches and running simulations.

    Returns a list of (testbench_id, test_number, test_name, testbench_path) tuples.
    """
    base_dir = Path(__file__).parent
    testbenches = discover_vhdl_testbenches(base_dir)

    tests = []
    for tb_path in testbenches:
        tb_id = get_testbench_id(tb_path, base_dir)

        # Run simulation to discover tests
        result = get_simulation_result(tb_path)

        if result.tests:
            # Add each individual test
            for test in result.tests:
                test_id = f"{tb_id}::test_{test.number:02d}_{test.name.replace(' ', '_')}"
                tests.append((test_id, tb_path, test.number, test.name))
        else:
            # No individual tests found - add the whole testbench as one test
            tests.append((f"{tb_id}::simulation", tb_path, None, None))

    return tests


# Collect tests at module load time
# This runs simulations once during collection
_collected_tests = None


def get_collected_tests():
    global _collected_tests
    if _collected_tests is None:
        _collected_tests = collect_vhdl_tests()
    return _collected_tests

def pytest_generate_tests(metafunc):
    """Generate test parameters dynamically."""
    if "vhdl_test_info" in metafunc.fixturenames:
        tests = get_collected_tests()
        metafunc.parametrize(
            "vhdl_test_info",
            tests,
            ids=[t[0] for t in tests]
        )

@pytest.mark.vhdl
def test_vhdl_simulation(vhdl_test_info):
    """
    Run and verify a VHDL simulation test.

    This test is parametrized for each individual test case found in the
    VHDL testbench output.
    """
    test_id, tb_path, test_number, test_name = vhdl_test_info

    # Get the cached simulation result
    result = get_simulation_result(tb_path)

    # Check for build/simulation errors
    if result.timed_out:
        pytest.fail(f"Simulation timed out for {tb_path}")

    if result.return_code != 0 and not result.tests:
        # Build or simulation failed before producing any test output
        error_msg = result.stderr or result.stdout or "Unknown error"
        pytest.fail(f"Simulation failed (exit code {result.return_code}):\n{error_msg[:1000]}")

    if test_number is None:
        # This is a whole-testbench test (no individual tests parsed)
        if result.return_code != 0:
            pytest.fail(f"Simulation failed with exit code {result.return_code}")
        return

    # Find the specific test result
    test_result = None
    for t in result.tests:
        if t.number == test_number:
            test_result = t
            break

    if test_result is None:
        pytest.fail(f"Test #{test_number} '{test_name}' not found in simulation output")

    # Check if the test passed
    if not test_result.passed:
        pytest.fail(f"VHDL test failed: {test_result.name}")


# Alternative approach: Class-based tests per testbench
class TestVhdlTestbenches:
    """
    Alternative test class that groups tests by testbench.

    Use this if you prefer class-based organization:
        pytest test_vhdl.py::TestVhdlTestbenches -v
    """

    @pytest.fixture(scope="class")
    def testbenches(self):
        base_dir = Path(__file__).parent
        return discover_vhdl_testbenches(base_dir)

    def test_all_testbenches_pass(self, testbenches):
        """Verify all discovered testbenches pass their simulations."""
        failures = []

        for tb_path in testbenches:
            result = get_simulation_result(tb_path)

            if result.timed_out:
                failures.append(f"{tb_path}: Simulation timed out")
            elif result.return_code != 0:
                failed_tests = [t for t in result.tests if not t.passed]
                if failed_tests:
                    for t in failed_tests:
                        failures.append(f"{tb_path}: Test #{t.number} '{t.name}' failed")
                else:
                    failures.append(f"{tb_path}: Exit code {result.return_code}")

        if failures:
            pytest.fail("\n".join(failures))
