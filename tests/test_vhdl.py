"""
VHDL testbench integration with pytest.

Produces JUnit XML with:
- classname = testbench path (e.g., "i2c_cbor.full_test") -> shows as "package" in Jenkins
- name = individual test name -> shows as test within the package
"""

import pytest


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
