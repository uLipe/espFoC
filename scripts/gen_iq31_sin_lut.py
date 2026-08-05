#!/usr/bin/env python3
"""
Generate esp_foc_iq31_sin_lut.inc: one 8192-entry Q1.31 sin table.
cos(angle) is obtained from the same table as sin(angle + pi/2), i.e. index_cos = (index_sin + 2048) & 8191.
Run from component root: python3 scripts/gen_iq31_sin_lut.py
"""
import math
import os

IQ31_MAX = 2**31 - 1
IQ31_MIN = -2**31
LUT_SIZE = 8192


def to_q31(x):
    v = round(x * (1 << 31))
    if v > IQ31_MAX:
        v = IQ31_MAX
    if v < IQ31_MIN:
        v = IQ31_MIN
    return int(v)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(script_dir, "..", "source", "motor_control", "esp_foc_iq31_sin_lut.inc")
    lines = [
        "/* sin(2*pi*i/%d) in Q1.31, i=0..%d. cos(angle)=lut[(idx+%d)&%d] */"
        % (LUT_SIZE, LUT_SIZE - 1, LUT_SIZE // 4, LUT_SIZE - 1)
    ]
    for i in range(LUT_SIZE):
        s = math.sin(2 * math.pi * i / LUT_SIZE)
        lines.append("    (iq31_t)%d," % to_q31(s))
    with open(out_path, "w") as f:
        f.write("\n".join(lines))
    print("Generated %s (%d entries)" % (out_path, LUT_SIZE))


if __name__ == "__main__":
    main()
