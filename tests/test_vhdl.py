"""
VHDL testbench integration with pytest.

Produces JUnit XML with:
- classname = testbench path (e.g., "i2c_cbor.full_test") -> shows as "package" in Jenkins
- name = individual test name -> shows as test within the package
"""

import pytest
from pathlib import Path
from conftest import VHDL_TESTBENCHES_TO_RUN, discover_vhdl_testbenches, get_simulation_result

BASE_DIR = Path(__file__).parent


def collect_vhdl_tests():
    """
    Collect all VHDL tests as tuples:
    (testbench_path, classname, test_number, test_name, passed)
    """
    testbenches = discover_vhdl_testbenches(BASE_DIR)
    all_tests = []

    for tb_path in testbenches:
        # Find the configured relative path for reporting
        rel_path = next((p for p in VHDL_TESTBENCHES_TO_RUN if p in str(tb_path)))
        # Use dots for classname (Java package style, what Jenkins expects)
        classname = rel_path.replace("/", ".")

        result = get_simulation_result(tb_path)

        if result.tests:
            for t in result.tests:
                all_tests.append((tb_path, classname, t.number, t.name, t.passed))
        else:
            # No individual tests -> treat whole testbench as a single test
            passed = result.return_code == 0 and not result.timed_out
            all_tests.append((tb_path, classname, None, None, passed))

    return all_tests


def pytest_generate_tests(metafunc):
    if "vhdl_test" in metafunc.fixturenames:
        all_tests = collect_vhdl_tests()
        metafunc.parametrize(
            "vhdl_test",
            all_tests,
            ids=[
                f"{classname}::{tnum:02d}_{tname.replace(' ', '_')}" if tname else f"{classname}::simulation"
                for _, classname, tnum, tname, _ in all_tests
            ]
        )


@pytest.mark.vhdl
def test_vhdl(vhdl_test, record_xml_attribute):
    """
    Run a single VHDL test (or whole testbench if individual tests not parsed).

    Uses record_xml_attribute to set classname and name in JUnit XML output,
    which Jenkins uses for hierarchical display.
    """
    tb_path, classname, tnum, tname, passed = vhdl_test

    # Build test name for JUnit XML
    if tname:
        test_name = f"test_{tnum:02d}_{tname.replace(' ', '_')}"
    else:
        test_name = "simulation"

    # Set JUnit XML attributes for proper Jenkins grouping
    # classname -> "package" in Jenkins test results view
    # name -> individual test within the package
    record_xml_attribute("classname", classname)
    record_xml_attribute("name", test_name)

    if not passed:
        pytest.fail(f"VHDL test failed: {tname or 'simulation'}")
