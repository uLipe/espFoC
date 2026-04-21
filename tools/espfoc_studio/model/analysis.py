"""Control-loop analysis helpers.

All models mirror the firmware design:

    plant   : G(s) = 1 / (R + L*s)   (first-order PMSM current loop)
    ZOH     : alpha = exp(-R*Ts/L)
    PI      : espFoC "delayed forward Euler" shape
                  u[k]  = Kp*e[k] + I[k-1]
                  I[k]  = I[k-1] + Ki*e[k]*Ts
    MPZ     : Kp = R*(1 - beta)/(1 - alpha),  Ki = R*(1 - beta)/Ts
              with beta = exp(-2*pi*bw_hz*Ts)

Functions return numpy arrays; nothing here touches Qt.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class MotorParams:
    """Motor parameters used everywhere in the analysis helpers."""
    r_ohm: float     # phase resistance
    l_h: float       # phase inductance
    ts_s: float      # control-loop sample period
    v_max: float = 12.0  # voltage ceiling (for anti-windup / saturation)

    def __post_init__(self) -> None:
        for name, v in (("r_ohm", self.r_ohm),
                        ("l_h", self.l_h),
                        ("ts_s", self.ts_s)):
            if v <= 0:
                raise ValueError(f"{name} must be positive, got {v}")


@dataclass(frozen=True)
class PiGains:
    kp: float  # V/A
    ki: float  # V/(A*s)
    int_lim: float = 0.0  # integrator anti-windup clamp, volts


# --- Design ------------------------------------------------------------------

def mpz_design(motor: MotorParams, bandwidth_hz: float) -> PiGains:
    """Run the same MPZ synthesis the firmware uses."""
    if bandwidth_hz <= 0:
        raise ValueError("bandwidth_hz must be positive")
    if bandwidth_hz * motor.ts_s >= 0.5:
        raise ValueError(
            f"bandwidth {bandwidth_hz} Hz above Nyquist for Ts={motor.ts_s}s"
        )
    alpha = np.exp(-motor.r_ohm * motor.ts_s / motor.l_h)
    beta = np.exp(-2.0 * np.pi * bandwidth_hz * motor.ts_s)
    one_minus_alpha = 1.0 - alpha
    one_minus_beta = 1.0 - beta
    kp = motor.r_ohm * one_minus_beta / one_minus_alpha
    ki = motor.r_ohm * one_minus_beta / motor.ts_s
    return PiGains(kp=kp, ki=ki, int_lim=motor.v_max)


# --- Transfer functions (discrete) ------------------------------------------

def _plant_tf(motor: MotorParams) -> Tuple[np.ndarray, np.ndarray]:
    """Return (num, den) polynomials of the ZOH-discretized plant
    G(z) = (1-alpha)/R * z^-1 / (1 - alpha z^-1).
    Using positive-power polynomials: num = [(1-alpha)/R, 0] at order 1,
    den = [1, -alpha]."""
    alpha = np.exp(-motor.r_ohm * motor.ts_s / motor.l_h)
    num = np.array([0.0, (1.0 - alpha) / motor.r_ohm])
    den = np.array([1.0, -alpha])
    return num, den


def _pi_tf(gains: PiGains, ts_s: float) -> Tuple[np.ndarray, np.ndarray]:
    """PI (delayed forward Euler) as positive-power polynomials:
         C(z) = [Kp - (Kp - Ki*Ts) z^-1] / (1 - z^-1)
       as positive powers: num_pos = [Kp, -(Kp - Ki*Ts)], den_pos = [1, -1].
    """
    num = np.array([gains.kp, -(gains.kp - gains.ki * ts_s)])
    den = np.array([1.0, -1.0])
    return num, den


def _poly_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.convolve(a, b)


def _poly_add(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    n = max(len(a), len(b))
    pa = np.concatenate([np.zeros(n - len(a)), a])
    pb = np.concatenate([np.zeros(n - len(b)), b])
    return pa + pb


def loop_gain_tf(motor: MotorParams, gains: PiGains) -> Tuple[np.ndarray, np.ndarray]:
    """L(z) = C(z) * G(z) — positive-power polynomials."""
    cn, cd = _pi_tf(gains, motor.ts_s)
    pn, pd = _plant_tf(motor)
    num = _poly_mul(cn, pn)
    den = _poly_mul(cd, pd)
    return num, den


def closed_loop_tf(motor: MotorParams, gains: PiGains) -> Tuple[np.ndarray, np.ndarray]:
    """T(z) = L(z) / (1 + L(z))."""
    ln, ld = loop_gain_tf(motor, gains)
    num = ln
    den = _poly_add(ld, ln)
    return num, den


# --- Simulation & frequency response ---------------------------------------

def step_response(motor: MotorParams, gains: PiGains,
                  n_samples: int = 200,
                  reference: float = 1.0) -> Tuple[np.ndarray, np.ndarray]:
    """Simulate the closed-loop step response to a unit step, using the
    same PID structure the firmware implements. Returns (t, i)."""
    alpha = np.exp(-motor.r_ohm * motor.ts_s / motor.l_h)
    i = np.zeros(n_samples)
    integ_prev = 0.0
    integ = 0.0
    u_max = motor.v_max
    for k in range(n_samples - 1):
        err = reference - i[k]
        u = gains.kp * err + integ_prev
        u = max(-u_max, min(u_max, u))
        integ_prev = integ
        integ = integ + gains.ki * err * motor.ts_s
        if gains.int_lim > 0:
            integ = max(-gains.int_lim, min(gains.int_lim, integ))
        i[k + 1] = alpha * i[k] + (1.0 - alpha) / motor.r_ohm * u
    t = np.arange(n_samples) * motor.ts_s
    return t, i


def bode(motor: MotorParams, gains: PiGains,
         n_points: int = 400,
         f_min_hz: float = 0.1,
         f_max_hz: float = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (freq_hz, magnitude_db, phase_deg) of the loop gain L(z)
    evaluated at z = exp(j*2pi*f*Ts). Nyquist caps f at 1/(2*Ts)."""
    nyquist = 1.0 / (2.0 * motor.ts_s)
    if f_max_hz is None:
        f_max_hz = 0.95 * nyquist
    f_max_hz = min(f_max_hz, 0.99 * nyquist)
    freqs = np.logspace(np.log10(f_min_hz), np.log10(f_max_hz), n_points)
    num, den = loop_gain_tf(motor, gains)
    z = np.exp(1j * 2.0 * np.pi * freqs * motor.ts_s)
    # polyval evaluates highest-degree-first; our polys are already that form.
    H = np.polyval(num, z) / np.polyval(den, z)
    mag_db = 20.0 * np.log10(np.abs(H) + 1e-24)
    phase_deg = np.unwrap(np.angle(H)) * 180.0 / np.pi
    return freqs, mag_db, phase_deg


def pole_zero_map(motor: MotorParams, gains: PiGains
                  ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return (open_loop_poles, open_loop_zeros, closed_loop_poles, closed_loop_zeros)
    as complex numpy arrays on the z-plane."""
    ln, ld = loop_gain_tf(motor, gains)
    cn, cd = closed_loop_tf(motor, gains)
    ol_poles = np.roots(ld) if len(ld) > 1 else np.array([])
    ol_zeros = np.roots(ln) if len(ln) > 1 else np.array([])
    cl_poles = np.roots(cd) if len(cd) > 1 else np.array([])
    cl_zeros = np.roots(cn) if len(cn) > 1 else np.array([])
    return ol_poles, ol_zeros, cl_poles, cl_zeros


def root_locus(motor: MotorParams, gains: PiGains,
               scale_min: float = 0.05,
               scale_max: float = 4.0,
               n_points: int = 200) -> Tuple[np.ndarray, np.ndarray]:
    """Trace the closed-loop poles as Kp is scaled from scale_min to scale_max
    around the current design. Ki is kept proportional so the PI zero keeps
    cancelling the plant pole (same ratio as the MPZ design).

    Returns (k_values, poles) where poles.shape = (n_points, n_cl_poles).
    """
    ks = np.linspace(scale_min, scale_max, n_points)
    k_ratio = gains.ki / gains.kp if gains.kp != 0 else 1.0
    # Collect closed-loop poles for each scaled Kp.
    all_poles = []
    for k in ks:
        scaled = PiGains(kp=gains.kp * k, ki=gains.kp * k * k_ratio,
                         int_lim=gains.int_lim)
        _, den = closed_loop_tf(motor, scaled)
        all_poles.append(np.roots(den) if len(den) > 1 else np.array([]))
    # Pad to square matrix so the caller can index per-pole trajectory.
    n_max = max(len(p) for p in all_poles)
    mat = np.full((n_points, n_max), np.nan + 1j * np.nan, dtype=complex)
    for i, p in enumerate(all_poles):
        mat[i, :len(p)] = p
    return ks, mat
