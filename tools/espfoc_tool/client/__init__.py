"""Host-side device API shared by espFoC Tool GUI and espfocctl."""

from ..protocol import TunerClient, TunerError, AxisStateFlag, ParamId, Op
from ..protocol.tuner import ConnectInfo, TUNER_FIRMWARE_TYPE_TSGX

EspFocClient = TunerClient

__all__ = [
    "EspFocClient",
    "TunerClient",
    "TunerError",
    "AxisStateFlag",
    "ParamId",
    "Op",
    "ConnectInfo",
    "TUNER_FIRMWARE_TYPE_TSGX",
]
