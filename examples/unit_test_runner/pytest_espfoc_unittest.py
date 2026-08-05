# SPDX-License-Identifier: MIT
# Run espFoC Unity unit tests: default on QEMU (CI and local); optional on device.
import pytest
from pytest_embedded import Dut
from pytest_embedded_idf.utils import idf_parametrize


# Default: run on QEMU (no hardware). Use for CI and local "pytest -m qemu".
@pytest.mark.host_test
@pytest.mark.qemu
@pytest.mark.parametrize('embedded_services', ['idf,qemu'], indirect=True)
@idf_parametrize('target', ['esp32'], indirect=['target'])
def test_espfoc_unit_tests_qemu(dut: Dut) -> None:
    dut.expect_exact('Press ENTER to see the list of tests.')
    dut.write('*')
    dut.expect(r'\d+ Tests 0 Failures \d* Ignored', timeout=120)


# Run on real device when ESPPORT is set (e.g. self-hosted runner).
@pytest.mark.generic
@idf_parametrize('target', ['esp32', 'esp32s3'], indirect=['target'])
def test_espfoc_unit_tests_device(dut: Dut) -> None:
    dut.expect_exact('Press ENTER to see the list of tests.')
    dut.write('*')
    dut.expect(r'\d+ Tests 0 Failures \d* Ignored', timeout=120)
