"""Stable status codes shared by the Gazebo management nodes."""

from enum import IntEnum


class StatusCode(IntEnum):
    """Machine-readable operation status published through diagnostics."""

    READY = 0
    RESETTING = 10
    RESET_SUCCEEDED = 11
    ATTACH_REJECTED_MODE = 20
    ATTACH_REJECTED_STATE = 21
    ATTACH_REJECTED_GATE = 22
    ATTACH_REQUESTED = 23
    ATTACH_CONFIRMED = 24
    DETACH_REQUESTED = 30
    DETACH_CONFIRMED = 31
    GZ_SERVICE_UNAVAILABLE = 40
    GZ_SERVICE_FAILED = 41
    CONTROLLER_UNAVAILABLE = 42
    HOME_FAILED = 43
    TIMEOUT = 44
    INVALID_REQUEST = 45
    PHYSICS_STATE_UNAVAILABLE = 46
    COLLISION_STATE_UNAVAILABLE = 47
    PHYSICS_MODEL_UNVALIDATED = 48
    RESET_STATE_INVALID = 49
    INTERNAL_ERROR = 99


def service_message(code: StatusCode, detail: str) -> str:
    """Encode the stable numeric and symbolic code in a Trigger response."""
    return f"{code.name}({int(code)}): {detail}"
