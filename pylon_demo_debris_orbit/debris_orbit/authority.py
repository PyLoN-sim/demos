"""Lifecycle and heartbeat bookkeeping for shared active_vessel authority."""

from dataclasses import dataclass


@dataclass(frozen=True)
class LeaseAction:
    action: str
    vessel_id: str


class SharedAuthority:
    """IDs in telemetry identify observations, never competing PyLoN owners."""

    def __init__(self, renew_period_sec):
        self.renew_period_sec = renew_period_sec
        self.vessel_id = ""
        self.generation = None
        self.owned = False
        self.requested = False
        self.next_request_at = 0.0

    def observe_vessel(self, vessel_id, active, generation):
        vessel_id = vessel_id if active else ""
        if (vessel_id, generation) == (self.vessel_id, self.generation):
            return False
        self.vessel_id, self.generation = vessel_id, generation
        self.owned = self.requested = False
        self.next_request_at = 0.0
        return True

    def observe_authority(self, vessel_id, owned, generation):
        if (vessel_id, generation) != (self.vessel_id, self.generation):
            return False
        lost = self.owned and not owned
        self.owned = bool(owned)
        return lost

    def due_action(self, now_sec):
        if not self.vessel_id or now_sec < self.next_request_at:
            return None
        self.next_request_at = now_sec + self.renew_period_sec
        action = "renew" if self.requested and self.owned else "acquire"
        self.requested = True
        return LeaseAction(action, self.vessel_id)

    def release_action(self):
        # Cancel a pending acquire too, including shutdown before its ACK.
        if not self.vessel_id or not self.requested:
            return None
        self.owned = self.requested = False
        return LeaseAction("release", self.vessel_id)
