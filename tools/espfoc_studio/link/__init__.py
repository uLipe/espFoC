"""Wire framing for the espFoC tuner protocol — Python mirror of
source/motor_control/esp_foc_link.c. Cross-validated against the
firmware-side encoder/decoder via tools/espfoc_studio/tests/.
"""

from .codec import (
    SYNC,
    HEADER_BYTES,
    TRAILER_BYTES,
    MAX_PAYLOAD,
    MAX_FRAME,
    Channel,
    Status,
    LinkError,
    crc16_ccitt,
    encode,
    Decoder,
)
from .transport import Transport, LoopbackTransport

__all__ = [
    "SYNC",
    "HEADER_BYTES",
    "TRAILER_BYTES",
    "MAX_PAYLOAD",
    "MAX_FRAME",
    "Channel",
    "Status",
    "LinkError",
    "crc16_ccitt",
    "encode",
    "Decoder",
    "Transport",
    "LoopbackTransport",
]
