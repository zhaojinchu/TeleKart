"""Per-device input profiles: presets, persistence, and device matching.

A profile is a mapping plus a chain configuration, keyed by the device's SDL
GUID. The GUID encodes vendor, product and version, so unplugging the wheel and
plugging it back into another port keeps its calibration, and a second identical
wheel inherits it.

The built-in presets exist so that the first drive with a new device works
without opening the wizard at all. They are starting points, not measurements:
the ranges are the manufacturer's nominal values, and the calibration step
replaces them with what the hardware actually does.
"""

from __future__ import annotations

import enum
import json
import os
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Callable, Iterator

from .chain import (
    AxisCalibration,
    AxisChainConfig,
    ChainConfig,
    ChainConfigError,
    OneEuroConfig,
    default_chain_config,
)
from .curves import LINEAR
from .mapping import (
    Action,
    ActionBinding,
    Control,
    ControlBinding,
    InputMap,
    MappingError,
    axis,
    button,
    hat,
    key,
)
from .mapping import (
    KEY_BACKSPACE,
    KEY_C,
    KEY_DOWN,
    KEY_H,
    KEY_L,
    KEY_LEFT,
    KEY_M,
    KEY_P,
    KEY_R,
    KEY_RETURN,
    KEY_RIGHT,
    KEY_SPACE,
    KEY_UP,
)

SCHEMA_VERSION = 1


class ProfileError(ValueError):
    """A stored profile cannot be understood."""


class DeviceKind(enum.Enum):
    """What sort of thing is driving.

    The kind is what decides the *defaults*: a wheel with digital pedals needs a
    much gentler rate limit than one with analog pedals, because with digital
    pedals the rate limit is the only thing standing between the driver and an
    on/off switch.
    """

    WHEEL_ANALOG = "wheel_analog"
    WHEEL_DIGITAL = "wheel_digital"
    GAMEPAD = "gamepad"
    KEYBOARD = "keyboard"
    SYNTHETIC = "synthetic"


@dataclass(frozen=True, slots=True)
class DeviceProfile:
    """Everything the input subsystem needs to turn one device into commands."""

    profile_id: str
    name: str
    kind: DeviceKind
    mapping: InputMap
    chain: ChainConfig
    #: Full mechanical travel of the wheel, degrees. Zero means unknown, which
    #: is the honest answer for most generic wheels.
    device_rotation_deg: float = 0.0
    #: How much of that travel the driver wants to use for full lock. Zero means
    #: "all of it". The effect is carried by the steering calibration's pre_gain;
    #: this field exists so the wizard can show and re-edit the number.
    rotation_lock_deg: float = 0.0
    #: Which preset this started life as. Useful when a preset is improved and
    #: the app wants to offer a re-seed.
    preset_id: str = ""
    updated: str = ""

    def with_chain(self, chain: ChainConfig) -> "DeviceProfile":
        return replace(self, chain=chain, updated=_now_iso())

    def with_mapping(self, mapping: InputMap) -> "DeviceProfile":
        return replace(self, mapping=mapping, updated=_now_iso())

    @property
    def digital_pedals(self) -> bool:
        return self.mapping.throttle.is_digital or self.mapping.brake.is_digital

    def to_dict(self) -> dict[str, Any]:
        return {
            "profile_id": self.profile_id,
            "name": self.name,
            "kind": self.kind.value,
            "mapping": self.mapping.to_dict(),
            "chain": self.chain.to_dict(),
            "device_rotation_deg": self.device_rotation_deg,
            "rotation_lock_deg": self.rotation_lock_deg,
            "preset_id": self.preset_id,
            "updated": self.updated,
        }

    @classmethod
    def from_dict(cls, data: object) -> "DeviceProfile":
        if not isinstance(data, dict):
            raise ProfileError(f"profile must be an object, got {type(data).__name__}")
        raw_kind = data.get("kind", DeviceKind.WHEEL_ANALOG.value)
        try:
            kind = DeviceKind(raw_kind)
        except ValueError as exc:
            raise ProfileError(f"unknown device kind {raw_kind!r}") from exc
        try:
            mapping = InputMap.from_dict(data.get("mapping", {}))
            chain = ChainConfig.from_dict(data.get("chain", {}))
        except (MappingError, ChainConfigError) as exc:
            raise ProfileError(str(exc)) from exc
        return cls(
            profile_id=str(data.get("profile_id", "")),
            name=str(data.get("name", "")),
            kind=kind,
            mapping=mapping,
            chain=chain,
            device_rotation_deg=float(data.get("device_rotation_deg", 0.0)),
            rotation_lock_deg=float(data.get("rotation_lock_deg", 0.0)),
            preset_id=str(data.get("preset_id", "")),
            updated=str(data.get("updated", "")),
        )


# --------------------------------------------------------------------------
# Presets
# --------------------------------------------------------------------------


#: Face-button layout shared by the wheel presets, in priority order.
_WHEEL_LAYOUT: tuple[tuple[Action, int], ...] = (
    # E-stop goes on button 0 on purpose. It is the largest, most central button
    # on every wheel hub that follows the PlayStation layout, and the one a
    # driver can hit without looking. Putting it somewhere less reachable to
    # avoid accidental presses gets the trade-off backwards: an unwanted stop
    # costs a restart, and an unreachable stop costs a wall.
    (Action.ESTOP, 0),
    (Action.HORN, 1),
    (Action.DISARM, 2),
    (Action.ARM, 3),
    (Action.PIT_LIMITER, 4),
    (Action.REVERSE, 5),
    (Action.HEADLIGHTS, 6),
    (Action.MARK_LAP, 8),
    (Action.RESET_ODOM, 9),
)


def _wheel_actions(*, avoid: frozenset[int] = frozenset()) -> tuple[ActionBinding, ...]:
    """Wheel button bindings, stepping around buttons that are pedals.

    `avoid` exists for the digital-pedal case: on that wheel buttons 6 and 7 are
    the brake and the throttle, and a preset that also put "headlights" on 6
    would flash the lights every time the driver braked. Actions shift up to the
    next free button instead, deterministically, so the preset stays a fixed
    table rather than something that has to be hand-maintained per device.
    """
    used = set(avoid)
    out: list[ActionBinding] = []
    for action, preferred in _WHEEL_LAYOUT:
        index = preferred
        while index in used:
            index += 1
        used.add(index)
        out.append(ActionBinding(action, button(index)))
    out.append(ActionBinding(Action.CYCLE_CAMERA, hat(0, 0, 1)))
    out.append(ActionBinding(Action.TOGGLE_HUD, hat(0, 1, 1)))
    return tuple(out)


def preset_logitech_wheel(profile_id: str = "", name: str = "") -> DeviceProfile:
    """Logitech G29 / G920 and the Driving Force family.

    Axes 0 / 2 / 5 for steering / throttle / brake, and the pedals idle at +1.0
    and read -1.0 when fully depressed -- the calibration below encodes that
    directly rather than pretending it is an inversion.
    """
    chain = default_chain_config(digital_pedals=False)
    chain = ChainConfig(
        steer=replace(
            chain.steer,
            calibration=AxisCalibration.steering(-1.0, 0.0, 1.0, pre_gain=2.5),
            deadzone=0.02,
            saturation=0.01,
        ),
        throttle=replace(
            chain.throttle, calibration=AxisCalibration.pedal(1.0, -1.0)
        ),
        brake=replace(chain.brake, calibration=AxisCalibration.pedal(1.0, -1.0)),
        speed_sensitive_steering=chain.speed_sensitive_steering,
        brake_cuts_throttle=False,
    )
    return DeviceProfile(
        profile_id=profile_id,
        name=name or "Logitech racing wheel",
        kind=DeviceKind.WHEEL_ANALOG,
        mapping=InputMap(
            steer=ControlBinding(axis(0)),
            throttle=ControlBinding(axis(2)),
            brake=ControlBinding(axis(5)),
            actions=_wheel_actions(),
        ),
        chain=chain,
        device_rotation_deg=900.0,
        rotation_lock_deg=360.0,
        preset_id="logitech_wheel",
        updated=_now_iso(),
    )


def preset_digital_wheel(profile_id: str = "", name: str = "") -> DeviceProfile:
    """A generic USB wheel whose pedals are buttons rather than potentiometers.

    Cheap wheels wire the pedals to plain switches, and the previous generation
    of this project ran on exactly such a wheel with throttle on button 7 and
    brake on button 6. The steering is still a real analog axis; only the pedals
    are digital.

    The important consequence is in the chain, not the mapping: a curve applied
    to a signal that is only ever 0 or 1 does nothing at all, so the rate limit
    becomes the pedal travel. `default_chain_config(digital_pedals=True)` sets
    it to about 600 ms from lift to full throttle, which is roughly how long a
    real pedal takes when driven briskly, and 250 ms to release.
    """
    chain = default_chain_config(digital_pedals=True)
    chain = replace(
        chain,
        steer=replace(
            chain.steer,
            calibration=AxisCalibration.steering(-1.0, 0.0, 1.0),
            deadzone=0.05,
            saturation=0.03,
        ),
        # Both pedal buttons can be held at once, and on a switch there is no
        # "how hard" to arbitrate with. Brake wins.
        brake_cuts_throttle=True,
    )
    return DeviceProfile(
        profile_id=profile_id,
        name=name or "Generic wheel (digital pedals)",
        kind=DeviceKind.WHEEL_DIGITAL,
        mapping=InputMap(
            steer=ControlBinding(axis(0)),
            throttle=ControlBinding(button(7)),
            brake=ControlBinding(button(6)),
            actions=_wheel_actions(avoid=frozenset({6, 7})),
        ),
        chain=chain,
        device_rotation_deg=270.0,
        rotation_lock_deg=0.0,
        preset_id="digital_wheel",
        updated=_now_iso(),
    )


def preset_gamepad(profile_id: str = "", name: str = "") -> DeviceProfile:
    """Xbox / DualShock-style controller through SDL's game-controller mapping.

    Triggers rest at -1.0 and read +1.0 fully pressed. The steering deadzone is
    larger than a wheel's because a thumbstick's centre is genuinely noisier, and
    the steering rate is lower because a stick reaches full deflection in about
    80 ms and the car cannot.
    """
    chain = default_chain_config(digital_pedals=False)
    chain = ChainConfig(
        steer=replace(
            chain.steer,
            calibration=AxisCalibration.steering(-1.0, 0.0, 1.0),
            deadzone=0.10,
            saturation=0.03,
            rate_rise=4.0,
            rate_fall=5.0,
            smoothing=OneEuroConfig(min_cutoff=3.0, beta=0.30),
        ),
        throttle=replace(
            chain.throttle, calibration=AxisCalibration.pedal(-1.0, 1.0), deadzone=0.06
        ),
        brake=replace(
            chain.brake, calibration=AxisCalibration.pedal(-1.0, 1.0), deadzone=0.06
        ),
        speed_sensitive_steering=0.25,
        brake_cuts_throttle=False,
    )
    return DeviceProfile(
        profile_id=profile_id,
        name=name or "Gamepad",
        kind=DeviceKind.GAMEPAD,
        mapping=InputMap(
            steer=ControlBinding(axis(0)),
            throttle=ControlBinding(axis(5)),
            brake=ControlBinding(axis(4)),
            actions=(
                ActionBinding(Action.HORN, button(0)),
                ActionBinding(Action.ESTOP, button(1)),
                ActionBinding(Action.MARK_LAP, button(2)),
                ActionBinding(Action.HEADLIGHTS, button(3)),
                ActionBinding(Action.REVERSE, button(4)),
                ActionBinding(Action.PIT_LIMITER, button(5)),
                ActionBinding(Action.DISARM, button(6)),
                ActionBinding(Action.ARM, button(7)),
                ActionBinding(Action.CYCLE_CAMERA, hat(0, 0, 1)),
                ActionBinding(Action.TOGGLE_HUD, hat(0, 1, 1)),
            ),
        ),
        chain=chain,
        preset_id="gamepad",
        updated=_now_iso(),
    )


def preset_keyboard() -> DeviceProfile:
    """Arrow keys. Every control is digital, including steering.

    Steering gets the gentlest rate limit in the app: a key press is an
    instantaneous demand for full lock, and without a ramp the car would slam the
    servo to its stop every time. 0.45 s lock to lock is close to what the HS-311
    manages anyway, so the limit costs nothing real.
    """
    chain = default_chain_config(digital_pedals=True)
    chain = replace(
        chain,
        steer=replace(
            chain.steer,
            calibration=AxisCalibration.steering(-1.0, 0.0, 1.0),
            deadzone=0.0,
            saturation=0.0,
            curve=LINEAR,
            rate_rise=2.2,
            rate_fall=4.4,
            smoothing=OneEuroConfig(min_cutoff=6.0, beta=0.10),
        ),
    )
    return DeviceProfile(
        profile_id="keyboard",
        name="Keyboard",
        kind=DeviceKind.KEYBOARD,
        mapping=InputMap(
            steer=ControlBinding(key(KEY_RIGHT), key(KEY_LEFT)),
            throttle=ControlBinding(key(KEY_UP)),
            brake=ControlBinding(key(KEY_DOWN)),
            actions=(
                ActionBinding(Action.ESTOP, key(KEY_SPACE)),
                ActionBinding(Action.ARM, key(KEY_RETURN)),
                ActionBinding(Action.DISARM, key(KEY_BACKSPACE)),
                ActionBinding(Action.REVERSE, key(KEY_R)),
                ActionBinding(Action.HORN, key(KEY_H)),
                ActionBinding(Action.HEADLIGHTS, key(KEY_L)),
                ActionBinding(Action.PIT_LIMITER, key(KEY_P)),
                ActionBinding(Action.MARK_LAP, key(KEY_M)),
                ActionBinding(Action.CYCLE_CAMERA, key(KEY_C)),
            ),
        ),
        chain=chain,
        preset_id="keyboard",
        updated=_now_iso(),
    )


def identity_profile(profile_id: str = "script", name: str = "Scripted") -> DeviceProfile:
    """Pass-through, for `ScriptSource` and `ReplaySource`.

    Those sources already emit normalized values, so calibration, deadzone and
    curves would all be wrong to apply. The rate limits are set high rather than
    removed, because "high enough to be invisible" keeps one code path instead of
    adding a bypass that only tests would exercise.
    """
    fast = 1000.0
    return DeviceProfile(
        profile_id=profile_id,
        name=name,
        kind=DeviceKind.SYNTHETIC,
        mapping=InputMap(),
        chain=ChainConfig(
            steer=AxisChainConfig(
                calibration=AxisCalibration.identity(Control.STEER),
                curve=LINEAR,
                rate_rise=fast,
                rate_fall=fast,
            ),
            throttle=AxisChainConfig(
                calibration=AxisCalibration.identity(Control.THROTTLE),
                curve=LINEAR,
                rate_rise=fast,
                rate_fall=fast,
            ),
            brake=AxisChainConfig(
                calibration=AxisCalibration.identity(Control.BRAKE),
                curve=LINEAR,
                rate_rise=fast,
                rate_fall=fast,
            ),
            brake_cuts_throttle=False,
        ),
        preset_id="identity",
        updated=_now_iso(),
    )


PRESET_BUILDERS: dict[str, Callable[..., DeviceProfile]] = {
    "logitech_wheel": preset_logitech_wheel,
    "digital_wheel": preset_digital_wheel,
    "gamepad": preset_gamepad,
    "keyboard": preset_keyboard,
    "identity": identity_profile,
}

PRESET_LABELS: dict[str, str] = {
    "logitech_wheel": "Logitech G29 / G920 / Driving Force",
    "digital_wheel": "Generic wheel with digital pedals",
    "gamepad": "Gamepad",
    "keyboard": "Keyboard",
    "identity": "Pass-through (scripted input)",
}

_WHEEL_NAME_HINTS = (
    "g29",
    "g920",
    "g923",
    "g27",
    "g25",
    "driving force",
    "logitech",
)
_GAMEPAD_NAME_HINTS = (
    "xbox",
    "x-box",
    "dualshock",
    "dualsense",
    "playstation",
    "ps4",
    "ps5",
    "gamepad",
    "controller",
    "8bitdo",
    "switch pro",
)
_WHEEL_GENERIC_HINTS = ("wheel", "racing", "steering", "ffb", "thrustmaster", "fanatec")


def preset_for_device(
    *,
    guid: str,
    name: str,
    num_axes: int,
    num_buttons: int,
    num_hats: int = 0,
) -> DeviceProfile:
    """Best guess for a device seen for the first time.

    The name is checked before the control counts because it is the only signal
    that distinguishes a wheel from a gamepad reliably -- both report two or more
    axes and a dozen buttons. The axis count is the tiebreaker, and it is what
    catches the case this app was built around: a wheel reporting one or two
    axes has no analog pedals, whatever its name says.
    """
    lowered = name.lower()
    looks_like_wheel = any(h in lowered for h in _WHEEL_NAME_HINTS) or any(
        h in lowered for h in _WHEEL_GENERIC_HINTS
    )
    looks_like_gamepad = any(h in lowered for h in _GAMEPAD_NAME_HINTS)

    if any(h in lowered for h in _WHEEL_NAME_HINTS) and num_axes >= 3:
        return preset_logitech_wheel(profile_id=guid, name=name)
    if looks_like_gamepad and not looks_like_wheel and num_axes >= 4:
        return preset_gamepad(profile_id=guid, name=name)
    if num_axes >= 3 and not looks_like_gamepad:
        # Three or more axes on something wheel-shaped: assume the extra axes
        # are pedals, in the order Logitech established and nearly everyone
        # copied. Calibration will correct it if not.
        return preset_logitech_wheel(profile_id=guid, name=name)
    if num_axes >= 4:
        return preset_gamepad(profile_id=guid, name=name)
    # One or two axes: steering only. The pedals must be buttons.
    profile = preset_digital_wheel(profile_id=guid, name=name)
    if 0 < num_buttons < 8:
        # Not enough buttons for the default 6/7 pedal pair. Fall back to the
        # last two, which is where cheap wheels put the paddles, and rebuild the
        # action layout around them so nothing lands on a pedal.
        top = max(num_buttons - 1, 1)
        second = max(top - 1, 0)
        profile = replace(
            profile,
            mapping=InputMap(
                steer=profile.mapping.steer,
                throttle=ControlBinding(button(top)),
                brake=ControlBinding(button(second)),
                actions=_wheel_actions(avoid=frozenset({top, second})),
            ),
        )
    return profile


# --------------------------------------------------------------------------
# Persistence
# --------------------------------------------------------------------------


def default_profile_path() -> Path:
    """Where profiles live.

    `platformdirs` is a declared dependency, but the import is guarded: this
    module has to stay importable in a bare test environment, and a wrong-but-
    conventional path is a much better failure than an ImportError during
    collection.
    """
    try:
        from platformdirs import user_config_dir

        base = Path(user_config_dir("TeleKart", "TeleKart"))
    except ImportError:
        base = Path.home() / ".config" / "telekart"
    return base / "input_profiles.json"


class ProfileStore:
    """JSON-backed profile storage, keyed by device GUID.

    Writes are atomic (temp file plus rename) because the alternative is a
    truncated profile file after a crash, and a truncated profile file means the
    next launch silently reverts to preset calibration on a car that is already
    on the ground.
    """

    __slots__ = ("_path", "_profiles", "last_error")

    def __init__(self, path: str | Path | None = None) -> None:
        self._path = Path(path) if path is not None else default_profile_path()
        self._profiles: dict[str, DeviceProfile] = {}
        self.last_error: str = ""
        self.load()

    @property
    def path(self) -> Path:
        return self._path

    def __len__(self) -> int:
        return len(self._profiles)

    def __iter__(self) -> Iterator[DeviceProfile]:
        return iter(self._profiles.values())

    def __contains__(self, profile_id: object) -> bool:
        return profile_id in self._profiles

    def all(self) -> tuple[DeviceProfile, ...]:
        return tuple(self._profiles.values())

    def get(self, profile_id: str) -> DeviceProfile | None:
        return self._profiles.get(profile_id)

    def put(self, profile: DeviceProfile) -> None:
        """Store in memory only. Call `save` to persist."""
        if not profile.profile_id:
            raise ProfileError("a profile needs a non-empty profile_id")
        self._profiles[profile.profile_id] = profile

    def remove(self, profile_id: str) -> bool:
        return self._profiles.pop(profile_id, None) is not None

    def resolve(
        self,
        *,
        guid: str,
        name: str,
        num_axes: int,
        num_buttons: int,
        num_hats: int = 0,
    ) -> DeviceProfile:
        """The stored profile for this device, or a freshly seeded preset."""
        stored = self._profiles.get(guid)
        if stored is not None:
            return stored
        return preset_for_device(
            guid=guid,
            name=name,
            num_axes=num_axes,
            num_buttons=num_buttons,
            num_hats=num_hats,
        )

    # -- disk ---------------------------------------------------------------

    def load(self) -> None:
        """Read from disk, replacing whatever is in memory.

        Never raises. A profile file that cannot be read is set aside and the app
        continues on presets -- refusing to start because of a bad settings file
        would be a worse outcome than losing a calibration.
        """
        self._profiles = {}
        self.last_error = ""
        if not self._path.exists():
            return
        try:
            raw = self._path.read_text(encoding="utf-8")
            data = json.loads(raw)
        except (OSError, json.JSONDecodeError) as exc:
            self.last_error = f"{self._path}: {exc}"
            self._quarantine()
            return
        if not isinstance(data, dict):
            self.last_error = f"{self._path}: expected a JSON object"
            self._quarantine()
            return

        entries = data.get("profiles", {})
        if not isinstance(entries, dict):
            self.last_error = f"{self._path}: 'profiles' must be an object"
            self._quarantine()
            return

        problems: list[str] = []
        for profile_id, entry in entries.items():
            try:
                profile = DeviceProfile.from_dict(entry)
            except (ProfileError, ValueError) as exc:
                # One bad profile must not cost the others. This is the case
                # that happens for real: a parameter's range tightened between
                # builds and one saved value now falls outside it.
                problems.append(f"{profile_id}: {exc}")
                continue
            self._profiles[str(profile_id)] = replace(
                profile, profile_id=str(profile_id)
            )
        if problems:
            self.last_error = "; ".join(problems)

    def save(self) -> None:
        """Persist atomically. Raises OSError only if the directory is unusable."""
        payload = {
            "schema": SCHEMA_VERSION,
            "updated": _now_iso(),
            "profiles": {pid: p.to_dict() for pid, p in self._profiles.items()},
        }
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._path.with_name(self._path.name + ".tmp")
        text = json.dumps(payload, indent=2, sort_keys=True)
        with open(tmp, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            # fsync before the rename: on a laptop that loses power mid-save,
            # rename-without-flush can leave a zero-length file where the old
            # good one used to be.
            os.fsync(handle.fileno())
        os.replace(tmp, self._path)

    def save_profile(self, profile: DeviceProfile) -> None:
        self.put(profile)
        self.save()

    def _quarantine(self) -> None:
        """Move an unreadable profile file aside so it is not overwritten.

        Keeping it costs nothing and it is the only copy of a calibration that
        may have taken twenty minutes to produce.
        """
        try:
            backup = self._path.with_name(
                self._path.name + ".corrupt-" + time.strftime("%Y%m%d-%H%M%S")
            )
            os.replace(self._path, backup)
        except OSError:
            pass


def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")
