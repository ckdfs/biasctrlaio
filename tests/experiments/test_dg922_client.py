"""Unit tests for dg922_client.py safety interlocks, readback checks, and RF-off guarantees."""

from __future__ import annotations

import math
import unittest

from scripts.experiments.dg922_client import DG922Client, SCPIError, dbm_to_vpp
from scripts.experiments.sim_plant import SimulatedDG922Socket


class TestDG922Client(unittest.TestCase):
    def test_dbm_to_vpp_and_safety_ceiling(self) -> None:
        # 0 dBm = 1 mW in 50 ohm -> Vpk = sqrt(0.1), Vpp = 2*sqrt(0.1) = 0.6324555 V
        self.assertAlmostEqual(dbm_to_vpp(0.0), 2.0 * math.sqrt(0.1), places=6)
        self.assertAlmostEqual(dbm_to_vpp(13.0), 2.0 * math.sqrt(100.0 * (10.0 ** -1.7)), places=6)
        # 24 dBm exceeds 10.0 Vpp safety cap -> ValueError
        with self.assertRaises(ValueError):
            dbm_to_vpp(24.0)

    def test_load_and_offset_readback_enforced_before_rf_on(self) -> None:
        sock = SimulatedDG922Socket()
        client = DG922Client(cmd_settle_s=0.0, socket_factory=lambda *a, **kw: sock)
        client.connect()
        # Bench starts at LOAD=9.9E+37 (High-Z) and OFFS=+0.125 V -> output_on MUST fail!
        with self.assertRaises(SCPIError):
            client.output_on(1)
        self.assertEqual(sock.output, 0)

        # Configure properly -> verifies LOAD=50, OFFS=0, Vpp<=10, and SYST:ERR?=0.
        vpp = client.configure_rf_dbm(20e6, 6.0)
        self.assertAlmostEqual(vpp, dbm_to_vpp(6.0), places=5)
        client.output_on(1)
        self.assertEqual(sock.output, 1)
        client.close()

    def test_rf_off_guaranteed_on_exception_and_scpi_error(self) -> None:
        sock = SimulatedDG922Socket()
        with self.assertRaises(RuntimeError):
            with DG922Client(cmd_settle_s=0.0, socket_factory=lambda *a, **kw: sock) as dg:
                dg.configure_rf_dbm(25e6, 3.0)
                dg.output_on(1)
                self.assertEqual(sock.output, 1)
                raise RuntimeError("Simulated mid-experiment crash")
        # Context manager exit MUST have turned RF output OFF and closed socket.
        self.assertEqual(sock.output, 0)
        self.assertTrue(sock.closed)

        # Non-zero :SYST:ERR? must fast-fail.
        sock2 = SimulatedDG922Socket()
        sock2.errors.append('-222,"Data out of range"')
        with DG922Client(cmd_settle_s=0.0, socket_factory=lambda *a, **kw: sock2) as dg2:
            with self.assertRaises(SCPIError):
                dg2.check_errors()


if __name__ == "__main__":
    unittest.main()
