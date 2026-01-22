import pytest
from pathlib import Path
from conftest import VHDL_TESTBENCHES_TO_RUN, discover_vhdl_testbenches, get_simulation_result

BASE_DIR = Path(__file__).parent

# Map resolved paths to configured relative paths for reporting
CONFIGURED_PATHS = [(Path(p).resolve(), p) for p in VHDL_TESTBENCHES_TO_RUN]

def collect_vhdl_tests():
    """
    Collect all VHDL tests as tuples:
    (testbench_path, testbench_rel_path, test_number, test_name, passed)
    """
    testbenches = discover_vhdl_testbenches(BASE_DIR)
    all_tests = []

    for tb_path in testbenches:
        # Find the configured relative path for reporting
        rel_path = next( (p for p in VHDL_TESTBENCHES_TO_RUN if p in str(tb_path)) )
        rel_path = rel_path.replace("/", "::")
        # rel_path = next((cfg for resolved, cfg in CONFIGURED_PATHS if resolved == tb_path), str(tb_path))
        result = get_simulation_result(tb_path)

        if result.tests:
            for t in result.tests:
                all_tests.append((tb_path, rel_path, t.number, t.name, t.passed))
        else:
            # No individual tests → treat whole testbench as a single test
            passed = result.return_code == 0 and not result.timed_out
            all_tests.append((tb_path, rel_path, None, None, passed))

    return all_tests

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

    classname = rel_path
    
    # Override nodeid so pytest/junitxml shows the correct suite and test
    test_name = tname if tname else "simulation"
    request.node._nodeid = f"{rel_path}::{test_name}"
    request.node.user_properties.append(("classname", classname))

    # Attach classname for JUnit XML
    request.node._report_sections = []
    request.node.user_properties.append(("classname", classname))

    if not passed:
        pytest.fail(f"{rel_path} -> Test {test_name} failed")
