"""Reproducible state packets and a causal fixed-gain centroidal observer.

This is a reduced-state sensor exercise, not an IMU or contact-state estimator.
Noise units are metres, kg m/s and kg m²/s for c, p and L respectively.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from centroidal_estimation import predict_delayed_state
from centroidal_model import CentroidalConfig, Contacts


@dataclass(frozen=True)
class SensorConfig:
    """Packet noise, transport delay and loss, indexed by capture tick."""

    com_std: float = 0.0005
    momentum_std: float = 0.01
    angular_std: float = 0.002
    delay_steps: int = 1
    dropout_probability: float = 0.1
    blackout: tuple[int, int] = (0, 0)
    seed: int = 0

    def __post_init__(self) -> None:
        """Reject invalid distributions and ambiguous timestamp settings."""
        noise = [self.com_std, self.momentum_std, self.angular_std]
        if not np.isfinite(noise).all() or min(noise) < 0:
            raise ValueError("Noise standard deviations must be finite and nonnegative")
        if not 0 <= self.dropout_probability <= 1:
            raise ValueError("Dropout probability must be in [0, 1]")
        if (
            not isinstance(self.delay_steps, int)
            or self.delay_steps < 0
            or not isinstance(self.seed, int)
            or self.seed < 0
            or len(self.blackout) != 2
            or not all(isinstance(i, int) for i in self.blackout)
            or not 0 <= self.blackout[0] <= self.blackout[1]
        ):
            raise ValueError("Invalid delay, seed or half-open blackout interval")


@dataclass(frozen=True)
class StatePacket:
    """One sensor sample, labelled with its capture index rather than arrival time."""

    index: int
    state: np.ndarray


class StateSensor:
    """Sample once per tick; do not redraw noise when a packet is delayed or lost."""

    def __init__(self, config: SensorConfig) -> None:
        """Create a private random stream independent of controller execution."""
        self.config = config
        self.rng = np.random.default_rng(config.seed)
        self.packets: list[StatePacket | None] = []
        self.dropped = 0

    def capture(self, state: np.ndarray, index: int) -> None:
        """Record a sample or a lost packet; both consume the same random draws."""
        state = np.asarray(state, dtype=float)
        if (
            index != len(self.packets)
            or state.shape != (9,)
            or not np.isfinite(state).all()
        ):
            raise ValueError("Capture needs a finite state and consecutive tick index")
        scale = np.repeat(
            [self.config.com_std, self.config.momentum_std, self.config.angular_std], 3
        )
        measured = state + self.rng.normal(size=9) * scale
        random_loss = self.rng.random() < self.config.dropout_probability
        burst = self.config.blackout[0] <= index < self.config.blackout[1]
        lost = random_loss or burst
        self.dropped += int(lost)
        measured.setflags(write=False)
        self.packets.append(None if lost else StatePacket(index, measured))

    def receive(self, current_index: int) -> StatePacket | None:
        """Return only the packet arriving at this tick, never a future sample."""
        if not 0 <= current_index < len(self.packets):
            raise ValueError("Current tick has not been captured")
        index = current_index - self.config.delay_steps
        return self.packets[index] if index >= 0 else None


class PacketStateEstimator:
    """Predict/correct at the measurement timestamp, then predict to control time.

    Gain 1 is the raw-packet predictor baseline; lower gain blends the innovation.
    The known initial condition is a simulated reset prior, not a live truth input.
    """

    def __init__(self, initial: np.ndarray, gain: float = 0.65) -> None:
        """Initialize the posterior at reset time with an explicit fixed gain."""
        self.posterior = np.asarray(initial, dtype=float).copy()
        if (
            self.posterior.shape != (9,)
            or not np.isfinite(self.posterior).all()
            or not np.isfinite(gain)
            or not 0 < gain <= 1
        ):
            raise ValueError("Invalid initial state or observer gain")
        self.gain = gain
        self.index = 0
        self.last_packet_index = -1
        self.last_control_index = -1
        self.corrections = 0

    def estimate(
        self,
        packet: StatePacket | None,
        current_index: int,
        commands: list[np.ndarray],
        contacts: Contacts,
        config: CentroidalConfig,
    ) -> np.ndarray:
        """Use past commands and at most one new packet; reject backwards time."""
        if current_index < self.last_control_index or current_index > len(commands):
            raise ValueError("Control timestamp is backwards or lacks past commands")
        if packet is not None:
            if (
                not 0 <= packet.index <= current_index
                or packet.index < self.last_packet_index
                or packet.state.shape != (9,)
                or not np.isfinite(packet.state).all()
            ):
                raise ValueError("Invalid or out-of-order measurement packet")
            if packet.index > self.last_packet_index:
                prior = predict_delayed_state(
                    self.posterior, self.index, packet.index, commands, contacts, config
                )
                self.posterior = prior + self.gain * (packet.state - prior)
                self.index = packet.index
                self.last_packet_index = packet.index
                self.corrections += 1
        self.last_control_index = current_index
        return predict_delayed_state(
            self.posterior, self.index, current_index, commands, contacts, config
        )
