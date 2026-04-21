"""Motor / control analysis library for TunerStudio.

Pure numpy; no Qt or GUI dependency. Reused by the GUI's Analysis tab,
the CLI snapshot report, and the offline golden-vector tests.
"""

from .analysis import (
    MotorParams,
    PiGains,
    mpz_design,
    closed_loop_tf,
    loop_gain_tf,
    step_response,
    bode,
    pole_zero_map,
    root_locus,
)

__all__ = [
    "MotorParams",
    "PiGains",
    "mpz_design",
    "closed_loop_tf",
    "loop_gain_tf",
    "step_response",
    "bode",
    "pole_zero_map",
    "root_locus",
]
