"""Stability gate used before the GUI displays its first detected position."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class InitialDisplayGate:
    """Require repeated high-confidence evidence before lighting the map.

    The recognition state machine intentionally starts at TR. This UI-only
    gate prevents that internal default, or a single false-positive TR frame,
    from looking like a confirmed visual recognition.
    """

    required_updates: int = 4
    confidence_threshold: float = 0.60
    activation_time: float = 0.0
    last_update: float = 0.0
    candidate: Optional[str] = None
    consecutive_updates: int = 0
    armed: bool = False

    def reset(self, activation_time: float = 0.0) -> None:
        self.activation_time = float(activation_time)
        self.last_update = float(activation_time)
        self.candidate = None
        self.consecutive_updates = 0
        self.armed = False

    def update(self, detection) -> bool:
        """Consume each fresh inference result once and return the armed state."""
        if self.armed:
            return True

        updated_at = float(getattr(detection, "updated_at", 0.0) or 0.0)
        if updated_at <= self.last_update or updated_at < self.activation_time:
            return False
        self.last_update = updated_at

        observed = getattr(detection, "observed_position", None)
        confirmed = getattr(detection, "confirmed_position", None)
        confidence = getattr(detection, "observed_confidence", None)
        is_valid = (
            observed is not None
            and observed == confirmed
            and getattr(detection, "safe_state", None) == "CONFIRMED"
            and confidence is not None
            and float(confidence) >= self.confidence_threshold
        )

        if not is_valid:
            self.candidate = None
            self.consecutive_updates = 0
            return False

        if observed == self.candidate:
            self.consecutive_updates += 1
        else:
            self.candidate = observed
            self.consecutive_updates = 1

        if self.consecutive_updates >= self.required_updates:
            self.armed = True
        return self.armed
