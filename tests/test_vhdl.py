"""
VHDL testbench integration with pytest.

Produces JUnit XML with:
- classname = testbench path (e.g., "i2c_cbor.full_test") -> shows as "package" in Jenkins
- name = individual test name -> shows as test within the package
- system-out = VHDL log context for this test
"""

import pytest


class VhdlTestFailure(Exception):
    """Custom exception for VHDL test failures with clean traceback."""
    pass


@pytest.mark.vhdl
def test_vhdl(vhdl_test, record_xml_attribute):
    """
    Run a single VHDL test (or whole testbench if individual tests not parsed).

    Uses record_xml_attribute to set classname and name in JUnit XML output,
    which Jenkins uses for hierarchical display.
    """
    # Build test name for JUnit XML
    if vhdl_test.test_name:
        test_name = f"test_{vhdl_test.test_number:02d}_{vhdl_test.test_name.replace(' ', '_')}"
    else:
        test_name = "simulation"

    # Set JUnit XML attributes for proper Jenkins grouping
    record_xml_attribute("classname", vhdl_test.classname)
    record_xml_attribute("name", test_name)

    # Print log context - this goes into <system-out> in JUnit XML
    if vhdl_test.log_context:
        print("\n" + "=" * 60)
        print(f"VHDL Log for: {vhdl_test.test_name or 'simulation'}")
        print("=" * 60)
        print(vhdl_test.log_context)
        print("=" * 60 + "\n")

    # Check if test passed
    if not vhdl_test.passed:
        # Build a clean failure message from VHDL log
        if vhdl_test.test_name:
            # Find error/fail lines and assertion warnings in log context
            fail_info = []
            for line in vhdl_test.log_context.split('\n'):
                line = line.strip()
                # Include assertion warnings/errors from GHDL
                if '(assertion' in line.lower():
                    fail_info.append(line)
                # Include explicit error/fail lines from logging
                elif '[ERR]' in line or 'FAIL' in line:
                    fail_info.append(line)

            if fail_info:
                error_msg = f"VHDL Test #{vhdl_test.test_number} FAILED: {vhdl_test.test_name}\n\n"
                error_msg += "Error details from VHDL log:\n"
                error_msg += "\n".join(f"  {line}" for line in fail_info)
            else:
                error_msg = f"VHDL Test #{vhdl_test.test_number} FAILED: {vhdl_test.test_name}"
        else:
            # Simulation-level failure
            error_msg = f"VHDL simulation failed for {vhdl_test.classname}"
            # Show last few lines of log
            if vhdl_test.full_log:
                last_lines = vhdl_test.full_log.strip().split('\n')[-10:]
                error_msg += "\n\nLast lines of log:\n"
                error_msg += "\n".join(f"  {line}" for line in last_lines)

        pytest.fail(error_msg, pytrace=False)
