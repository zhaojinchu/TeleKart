"""The single source of visual truth for the desktop application.

Both the QSS template and every ``QPainter``-drawn instrument resolve their
colours, spacing, radii and type sizes from the one ``Theme`` instance exported
here. That is not a stylistic preference: a stylesheet palette and a painter
palette that drift apart produce an application that looks like two programs
stitched together, and the drift is invisible until someone puts a styled
QPushButton next to a hand-painted gauge.

Nothing in this module touches a QApplication, so it is importable from tests
and from tooling that never opens a window.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from PySide6.QtGui import QColor

# --------------------------------------------------------------------------
# Palette
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Palette:
    """The complete colour vocabulary. Thirteen entries, and no others.

    Anything that looks like a fourteenth colour -- a hover tint, a pressed
    state, a disabled label, a scrim -- is *derived* from these by ``mix()`` or
    ``with_alpha()``. Deriving rather than adding is what keeps the surface
    count low enough that a new screen cannot accidentally invent its own
    slightly-different grey.

    Colour semantics, which are enforced by convention and reviewed as such:

    ``good`` / ``bad``
        **Deltas only.** Green means *faster* and nothing else, anywhere in the
        application. The moment green also means "connected" or "healthy", a
        driver glancing at a delta bar mid-corner has to stop and read a label,
        and the widget has failed at its one job. ``bad`` carries a single
        documented exception -- see ``fault``/``estop`` in ``StateColors``,
        because a critical fault indicator that is not red is a safety defect,
        and red is not ambiguous in that context.

    ``accent``
        Brand, focus, selection, "this is the active thing", and the driver's
        own commanded inputs. Never a status.

    ``warn``
        Caution: a degrading link, a limiter engaging, a slower sector.

    ``cyan`` / ``purple``
        Data series. Cyan is *measured*, purple is *target/reference*. Holding
        that assignment across the HUD, the tuning plots and the track map is
        what lets someone read an unfamiliar chart without a legend.
    """

    bg_base: str = "#0B0D10"
    bg_raised: str = "#14171C"
    bg_overlay: str = "#1B1F26"
    line: str = "#262B33"
    text_primary: str = "#E8EBF0"
    text_secondary: str = "#98A1AE"
    text_tertiary: str = "#5F6875"
    accent: str = "#FF4D2E"
    good: str = "#2FD07A"
    warn: str = "#FFB020"
    bad: str = "#FF4757"
    purple: str = "#B36BFF"
    cyan: str = "#33C9FF"


class QtPalette:
    """``QColor`` twins of every :class:`Palette` entry, built once at import.

    Instruments read these directly in ``paintEvent``. Constructing a QColor
    from a hex string re-parses the string every time; a dozen instruments at
    60 Hz makes that measurable, and there is no reason to pay it repeatedly
    for values that never change.
    """

    __slots__ = (
        "bg_base",
        "bg_raised",
        "bg_overlay",
        "line",
        "text_primary",
        "text_secondary",
        "text_tertiary",
        "accent",
        "good",
        "warn",
        "bad",
        "purple",
        "cyan",
    )

    bg_base: QColor
    bg_raised: QColor
    bg_overlay: QColor
    line: QColor
    text_primary: QColor
    text_secondary: QColor
    text_tertiary: QColor
    accent: QColor
    good: QColor
    warn: QColor
    bad: QColor
    purple: QColor
    cyan: QColor

    def __init__(self, palette: Palette) -> None:
        for name in QtPalette.__slots__:
            setattr(self, name, QColor(getattr(palette, name)))


# --------------------------------------------------------------------------
# Semantic mappings
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class StateColors:
    """Vehicle state -> palette key, resolved once so no screen invents its own.

    Keys, not colours, so this stays declarative and stays in the token file.
    ``SAFE`` is deliberately a *neutral* grey rather than green: disarmed is not
    an achievement, it is the absence of one, and painting it green would put a
    second meaning on the delta colour.
    """

    boot: str = "text_tertiary"
    safe: str = "text_secondary"
    armed: str = "accent"
    failsafe: str = "warn"
    estop: str = "bad"
    fault: str = "bad"


@dataclass(frozen=True, slots=True)
class SeriesColors:
    """Chart and instrument series roles. Same assignment everywhere."""

    measured: str = "cyan"
    target: str = "purple"
    command: str = "accent"
    limit: str = "warn"
    reference: str = "text_tertiary"
    left: str = "cyan"
    right: str = "purple"


@dataclass(frozen=True, slots=True)
class LinkColors:
    """Link health. Explicitly not green -- see the note in :class:`Palette`."""

    nominal: str = "cyan"
    degraded: str = "warn"
    down: str = "bad"
    idle: str = "text_tertiary"


# --------------------------------------------------------------------------
# Geometry, type, motion
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Spacing:
    """A six-step scale. Any layout margin not on this list is a bug.

    Restricting the scale is what makes independently-written screens line up
    when they are finally placed side by side.
    """

    xs: int = 4
    sm: int = 8
    md: int = 12
    lg: int = 16
    xl: int = 24
    xxl: int = 32

    @property
    def scale(self) -> tuple[int, int, int, int, int, int]:
        return (4, 8, 12, 16, 24, 32)


@dataclass(frozen=True, slots=True)
class Radii:
    """Three radii, tied to elevation rather than to taste.

    Controls sit on a surface (4), cards *are* the surface (8), and overlays
    float above everything (12). A control with a card's radius reads as a
    card, which is exactly the kind of ambiguity that makes an interface feel
    imprecise.
    """

    control: int = 4
    card: int = 8
    overlay: int = 12


@dataclass(frozen=True, slots=True)
class TypeScale:
    """Point sizes, tuned for macOS rendering (the primary target).

    ``hero`` and ``readout`` are for numbers only and must be paired with the
    numeric font role, which is tabular. Proportional digits in a live readout
    make the whole number shuffle sideways every time a 1 becomes a 7.
    """

    hero: int = 44
    readout: int = 28
    display: int = 34
    title: int = 20
    heading: int = 15
    body: int = 13
    label: int = 11
    micro: int = 10


@dataclass(frozen=True, slots=True)
class Weights:
    """QFont weight integers, spelled out so no widget hardcodes ``75``."""

    regular: int = 400
    medium: int = 500
    semibold: int = 600
    bold: int = 700


@dataclass(frozen=True, slots=True)
class FontStacks:
    """Family preference chains, best first.

    Declared here rather than in ``fonts.py`` so the whole visual definition
    stays in one file; ``fonts.py`` owns the *resolution* of these against what
    the machine actually has installed.
    """

    ui: tuple[str, ...] = (
        "Inter",
        "Inter Display",
        "SF Pro Text",
        "Helvetica Neue",
        "Segoe UI Variable Text",
        "Segoe UI",
        "Roboto",
        "Noto Sans",
        "DejaVu Sans",
    )
    #: Every number in the application is rendered with this stack. All of them
    #: are monospaced, hence tabular by construction, which is the cheap way to
    #: guarantee non-jittering readouts even on a machine whose fonts do not
    #: expose the `tnum` OpenType feature.
    numeric: tuple[str, ...] = (
        "JetBrains Mono",
        "IBM Plex Mono",
        "Roboto Mono",
        "SF Mono",
        "Menlo",
        "Consolas",
        "DejaVu Sans Mono",
        "Courier New",
    )
    mono: tuple[str, ...] = (
        "JetBrains Mono",
        "IBM Plex Mono",
        "SF Mono",
        "Menlo",
        "Consolas",
        "DejaVu Sans Mono",
        "Courier New",
    )


@dataclass(frozen=True, slots=True)
class Durations:
    """Motion, in milliseconds, plus the time constants instruments smooth with.

    Nothing in a driving HUD may animate slowly. ``base`` is the longest thing
    a driver ever waits on, and it is under a fifth of a second on purpose.
    """

    instant_ms: int = 90
    fast_ms: int = 140
    base_ms: int = 180
    slow_ms: int = 280

    #: Exponential follower time constants for painted instruments. A needle
    #: driven straight from 50 Hz telemetry looks nervous; one smoothed past
    #: ~120 ms lies about what the car is doing.
    needle_tau_s: float = 0.070
    bar_tau_s: float = 0.045

    #: Peak-hold marker behaviour on bar meters.
    peak_hold_ms: int = 900
    peak_fall_per_s: float = 0.9

    #: Over-limit blink period for the shift lights.
    blink_ms: int = 120


@dataclass(frozen=True, slots=True)
class Strokes:
    """Line weights. Hairlines are 1 device-independent pixel and stay there."""

    hairline: float = 1.0
    thin: float = 1.5
    normal: float = 2.0
    thick: float = 3.0
    #: The navigation rail's active-item marker. Referenced by name because it
    #: is a spec'd dimension, not a free parameter.
    rail_marker: float = 3.0


# --------------------------------------------------------------------------
# The theme
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Theme:
    """Everything the visual layer is allowed to know about appearance.

    Passed by reference, never copied per widget: identity comparison is a
    valid way to ask "are these two widgets themed the same".
    """

    color: Palette = field(default_factory=Palette)
    space: Spacing = field(default_factory=Spacing)
    radius: Radii = field(default_factory=Radii)
    type: TypeScale = field(default_factory=TypeScale)
    weight: Weights = field(default_factory=Weights)
    fonts: FontStacks = field(default_factory=FontStacks)
    duration: Durations = field(default_factory=Durations)
    stroke: Strokes = field(default_factory=Strokes)
    state: StateColors = field(default_factory=StateColors)
    series: SeriesColors = field(default_factory=SeriesColors)
    link: LinkColors = field(default_factory=LinkColors)

    #: QColor twins, built in __post_init__. Instruments use this in paint code.
    q: QtPalette = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", QtPalette(self.color))

    # -- lookups ------------------------------------------------------------

    def hex(self, key: str) -> str:
        """Palette key -> ``#RRGGBB``. Raises on an unknown key, deliberately.

        Fails at build/construction time so a typo in a stylesheet token or a
        state mapping surfaces the first time the app is launched, not the
        first time the car faults.
        """
        try:
            return getattr(self.color, key)  # type: ignore[no-any-return]
        except AttributeError:
            raise KeyError("unknown palette key %r" % (key,)) from None

    def rgb(self, key: str) -> QColor:
        """Palette key -> the shared ``QColor``. Do not mutate the result."""
        try:
            return getattr(self.q, key)  # type: ignore[no-any-return]
        except AttributeError:
            raise KeyError("unknown palette key %r" % (key,)) from None

    def state_color(self, state_name: str) -> QColor:
        """``VehicleState`` name (any case) -> QColor, via :class:`StateColors`.

        Takes a name rather than the enum so this module keeps its promise of
        importing nothing but Qt -- the protocol package is a dependency of the
        model layer, not of the paint layer.
        """
        key = getattr(self.state, state_name.lower(), None)
        return self.rgb(key) if key is not None else self.q.text_tertiary

    def series_color(self, role: str) -> QColor:
        key = getattr(self.series, role, None)
        return self.rgb(key) if key is not None else self.q.text_secondary

    def link_color(self, role: str) -> QColor:
        key = getattr(self.link, role, None)
        return self.rgb(key) if key is not None else self.q.text_tertiary

    def delta_color(self, delta: float, *, lower_is_better: bool = True) -> QColor:
        """The one sanctioned producer of ``good``/``bad``.

        Routing every delta through here is what makes the "green is faster"
        rule mechanically true instead of merely documented.
        """
        if delta == 0.0:
            return self.q.text_secondary
        ahead = (delta < 0.0) if lower_is_better else (delta > 0.0)
        return self.q.good if ahead else self.q.bad


# --------------------------------------------------------------------------
# Colour arithmetic
# --------------------------------------------------------------------------


def mix(a: QColor, b: QColor, t: float) -> QColor:
    """Blend ``a`` toward ``b`` by ``t`` in sRGB.

    sRGB rather than a perceptual space on purpose: these are all tints of near
    neutrals over an opaque dark ground, where the difference is invisible and
    the cost is a dependency plus a cube root per call.
    """
    if t <= 0.0:
        return QColor(a)
    if t >= 1.0:
        return QColor(b)
    inv = 1.0 - t
    return QColor(
        int(a.red() * inv + b.red() * t + 0.5),
        int(a.green() * inv + b.green() * t + 0.5),
        int(a.blue() * inv + b.blue() * t + 0.5),
        int(a.alpha() * inv + b.alpha() * t + 0.5),
    )


def with_alpha(color: QColor, alpha: float) -> QColor:
    """A copy of ``color`` at fractional alpha 0..1."""
    out = QColor(color)
    out.setAlphaF(0.0 if alpha < 0.0 else 1.0 if alpha > 1.0 else alpha)
    return out


def hex_of(color: QColor) -> str:
    return color.name(QColor.NameFormat.HexRgb).upper()


def mix_hex(a: str, b: str, t: float) -> str:
    """Blend two hex strings and return an opaque hex string.

    Stylesheet hover and pressed states use this rather than ``rgba()``: the
    ground under every control in this application is opaque, so a blend is
    pixel-identical to a translucent overlay and avoids Qt's stylesheet colour
    parser entirely, which is fussy about the alpha argument's type.
    """
    return hex_of(mix(QColor(a), QColor(b), t))


def rgba_css(color: str | QColor, alpha: float) -> str:
    """``rgba(r, g, b, a)`` with an **integer** alpha, for QSS.

    Qt's stylesheet parser reads the fourth argument as an integer 0-255; a
    float like ``0.6`` parses as ``0`` and the element vanishes. Reserved for
    the handful of genuinely translucent surfaces (tooltips, scrims) where a
    blend against a known ground is not possible.
    """
    c = QColor(color) if isinstance(color, str) else color
    a = int(round(255.0 * (0.0 if alpha < 0.0 else 1.0 if alpha > 1.0 else alpha)))
    return "rgba(%d, %d, %d, %d)" % (c.red(), c.green(), c.blue(), a)


# --------------------------------------------------------------------------
# The instance
# --------------------------------------------------------------------------

#: The application-wide theme. Widgets take an optional ``theme`` argument so
#: they can be exercised in isolation, but in the running app every one of them
#: shares this object -- and so does the stylesheet.
THEME = Theme()


def theme() -> Theme:
    """The shared theme instance."""
    return THEME


__all__ = [
    "Durations",
    "FontStacks",
    "LinkColors",
    "Palette",
    "QtPalette",
    "Radii",
    "SeriesColors",
    "Spacing",
    "StateColors",
    "Strokes",
    "THEME",
    "Theme",
    "TypeScale",
    "Weights",
    "hex_of",
    "mix",
    "mix_hex",
    "rgba_css",
    "theme",
    "with_alpha",
]
