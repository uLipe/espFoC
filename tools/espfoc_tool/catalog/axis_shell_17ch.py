"""Channel catalog for examples/axis_shell (17ch ESPF stream per axis)."""

from __future__ import annotations

from dataclasses import dataclass

Q16 = 1.0 / 65536.0

# Applied D/Q and phase duties are per-unit Q16 in [0, 1] on the wire.


@dataclass(frozen=True)
class ChannelDesc:
    index: int
    name: str
    unit: str
    scale: float

    def to_display(self, raw_q16: int) -> float:
        return raw_q16 * self.scale


EXPECTED_N_CH = 17

CHANNELS: tuple[ChannelDesc, ...] = (
    ChannelDesc(0, "Target D-Current", "A", Q16),
    ChannelDesc(1, "Measured D-Current", "A", Q16),
    ChannelDesc(2, "Target Q-Current", "A", Q16),
    ChannelDesc(3, "Measured Q-Current", "A", Q16),
    ChannelDesc(4, "Applied D-Voltage", "pu", Q16),
    ChannelDesc(5, "Applied Q-Voltage", "pu", Q16),
    ChannelDesc(6, "Rotor mechanical position", "turn", Q16),
    ChannelDesc(7, "Rotor electrical position", "turn", Q16),
    ChannelDesc(8, "Rotor estimated speed", "turn/s", Q16),
    ChannelDesc(9, "Phase U duty", "pu", Q16),
    ChannelDesc(10, "Phase V duty", "pu", Q16),
    ChannelDesc(11, "Phase W duty", "pu", Q16),
    ChannelDesc(12, "Measured phase U current", "A", Q16),
    ChannelDesc(13, "Measured phase V current", "A", Q16),
    ChannelDesc(14, "Alpha current", "A", Q16),
    ChannelDesc(15, "Beta current", "A", Q16),
    ChannelDesc(16, "ISR hot-path execution", "µs", Q16),
)

BY_INDEX = {c.index: c for c in CHANNELS}
