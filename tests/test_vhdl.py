import pytest
from pathlib import Path
from conftest import VHDL_TESTBENCHES_TO_RUN, discover_vhdl_testbenches, get_simulation_result

BASE_DIR = Path(__file__).parent
CONFIGURED_PATHS = [(Path(p).resolve(), p) for p in VHDL_TESTBENCHES_TO_RUN]

# Collect all testcases at collection time
def collect_vhdl_tests():
    """
    Return a list of tuples:
    (testbench_full_path, testbench_relative_path, test_number, test_name)
    """
    testbenches = discover_vhdl_testbenches(BASE_DIR)
    all_tests = []

    for tb_path in testbenches:
        # Determine relative path from configuration
        rel_path = next((cfg for resolved, cfg in CONFIGURED_PATHS if resolved == tb_path), str(tb_path))
        result = get_simulation_result(tb_path)

        if result.tests:
            for t in result.tests:
                all_tests.append((tb_path, rel_path, t.number, t.name, t.passed))
        else:
            # No individual tests -> include as a single test
            all_tests.append((tb_path, rel_path, None, None, result.return_code == 0))

    return all_tests

# Generate tests dynamically
def pytest_generate_tests(metafunc):
    if "vhdl_test" in metafunc.fixturenames:
        all_tests = collect_vhdl_tests()
        metafunc.parametrize(
            "vhdl_test",
            all_tests,
            ids=[
                f"{rel_path}::test_{tnum:02d}_{tname.replace(' ', '_')}" if tname else f"{rel_path}::simulation"
                for _, rel_path, tnum, tname, _ in all_tests
            ]
        )

@pytest.mark.vhdl
def test_vhdl(vhdl_test, request):
    """
    Run a single VHDL test (or whole testbench if individual tests not parsed)
    """
    tb_path, rel_path, tnum, tname, passed = vhdl_test

    # Override classname for JUnit XML
    # This will appear as the "test suite" in the XML
    request.node._nodeid = f"{rel_path}::{tname or 'simulation'}"

    if not passed:
        name = tname if tname else "simulation"
        pytest.fail(f"{rel_path} -> Test {name} failed")
