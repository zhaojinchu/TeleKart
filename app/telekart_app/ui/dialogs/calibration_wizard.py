"""Teaching the app what the driver's hardware actually does.

The rule this dialog is built around: **the driver never sees an axis index.**
Not once, not in a tooltip, not in the summary. "Axis 2" is meaningless to
someone holding a steering wheel, and asking them to find it is asking them to
do the computer's job. So every page works the same way -- it says which control
to move, watches every axis and every button at once, and takes the one that
moved as the answer.

That inversion is also what makes the awkward hardware work without a special
case. A wheel whose pedals are two buttons, a pad whose triggers idle at -1.0, a
wheel stuck in combined-pedal mode where throttle and brake share one axis: in
each case the driver presses the pedal, exactly one input changes, and the
wizard records what it saw. There is no list of supported devices to fall off.

What is recorded is a *measurement*, never a preference. ``rest``/``lo``/``hi``
are the numbers the hardware produced; ``invert`` stays untouched and remains
the driver's separate, explicit "this is backwards" switch. Keeping the two
apart is what stops a recalibration from double-inverting an axis that was
already fine.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Callable

from PySide6.QtCore import QRectF, QSize, Qt, QTimer
from PySide6.QtGui import QPainter, QPaintEvent
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
    QWizard,
    QWizardPage,
)

from ...input.chain import AxisCalibration, AxisChainConfig, ChainConfigError
from ...input.mapping import (
    Control,
    ControlBinding,
    InputRef,
    RefKind,
    axis as axis_ref,
    button as button_ref,
)
from ...input.profile import DeviceProfile
from ...input.sources import EMPTY_SNAPSHOT, DeviceSnapshot
from ..theme import fonts
from ..theme.tokens import THEME, Theme, with_alpha
from ..widgets.base import clamp

Sampler = Callable[[], "DeviceSnapshot | None"]

#: Poll rate while a page is open. Fast enough that a flick of the wheel is
#: caught, slow enough to be free.
_POLL_MS = 16

#: Movement that counts as "the driver moved this one". SDL axes idle with a
#: few counts of noise and a cheap pot can drift several percent, so the
#: threshold has to clear that without demanding a full sweep to register.
_DETECT_THRESHOLD = 0.30

#: A pedal that never travels this far has not really been pressed, and letting
#: it through produces a calibration that saturates at a touch.
_MIN_PEDAL_SPAN = 0.20

#: Steering half-span. Below this the wheel was not turned to the stop, and the
#: resulting calibration would make a quarter turn read as full lock.
_MIN_STEER_HALF = 0.15


@dataclass
class _Capture:
    """Everything the pages accumulate, in one object the wizard owns."""

    steer_ref: InputRef | None = None
    steer_lo: float = 0.0
    steer_hi: float = 0.0
    steer_rest: float = 0.0

    pedals: dict[Control, tuple[InputRef, float, float]] = field(default_factory=dict)

    rotation_deg: float = 0.0
    lock_deg: float = 0.0
    invert_steer: bool = False

    def steer_ok(self) -> bool:
        if self.steer_ref is None:
            return False
        return (
            self.steer_rest - self.steer_lo >= _MIN_STEER_HALF
            and self.steer_hi - self.steer_rest >= _MIN_STEER_HALF
        )


class _TravelBar(QWidget):
    """Live raw position, with the captured extremes marked.

    A plain number is not enough here: the driver needs to see that the mark
    stopped moving when they hit the mechanical stop, which is the only way to
    tell a full sweep from a nearly-full one.
    """

    def __init__(self, parent: QWidget | None = None, *, theme: Theme = THEME) -> None:
        super().__init__(parent)
        self._theme = theme
        self._value = 0.0
        self._lo: float | None = None
        self._hi: float | None = None
        self._rest: float | None = None
        self._bipolar = True
        self.setMinimumHeight(58)
        self._font = fonts.ui_font(theme.type.micro, theme.weight.semibold, theme=theme)

    def configure(self, *, bipolar: bool) -> None:
        self._bipolar = bipolar
        self.update()

    def set_value(self, value: float) -> None:
        if value != self._value:
            self._value = value
            self.update()

    def set_marks(self, lo: float | None, hi: float | None, rest: float | None) -> None:
        self._lo, self._hi, self._rest = lo, hi, rest
        self.update()

    def _x(self, rect: QRectF, value: float) -> float:
        # Bipolar during capture, because raw SDL axes are -1..+1 whatever the
        # control is and there is no calibrated range yet -- that is what the
        # driver is producing. Unipolar on the summary page, where the bars
        # show the *calibrated* pedal travel.
        if self._bipolar:
            return rect.left() + rect.width() * (clamp(value, -1.0, 1.0) + 1.0) * 0.5
        return rect.left() + rect.width() * clamp(value, 0.0, 1.0)

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        q = self._theme.q
        track = QRectF(8.0, self.height() * 0.5 - 7.0, self.width() - 16.0, 14.0)

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(with_alpha(q.text_primary, 0.10))
        painter.drawRoundedRect(track, 4.0, 4.0)

        if self._lo is not None and self._hi is not None and self._hi > self._lo:
            span = QRectF(
                self._x(track, self._lo),
                track.top(),
                self._x(track, self._hi) - self._x(track, self._lo),
                track.height(),
            )
            painter.setBrush(with_alpha(q.cyan, 0.28))
            painter.drawRoundedRect(span, 4.0, 4.0)

        for value, color in (
            (self._lo, q.cyan),
            (self._hi, q.cyan),
            (self._rest, q.warn),
        ):
            if value is None:
                continue
            x = self._x(track, value)
            painter.setBrush(color)
            painter.drawRect(QRectF(x - 1.0, track.top() - 5.0, 2.0, track.height() + 10.0))

        x = self._x(track, self._value)
        painter.setBrush(q.accent)
        painter.drawEllipse(QRectF(x - 7.0, track.center().y() - 7.0, 14.0, 14.0))

        painter.setFont(self._font)
        painter.setPen(q.text_tertiary)
        painter.drawText(
            QRectF(8.0, 0.0, self.width() - 16.0, 16.0),
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            "full travel of the control you are moving",
        )
        painter.end()


class _WizardPage(QWizardPage):
    """Common plumbing: a poll timer and access to the shared capture."""

    def __init__(self, wizard: "CalibrationWizard", title: str, body: str) -> None:
        super().__init__(wizard)
        self._wizard = wizard
        self.setTitle(title)

        layout = QVBoxLayout(self)
        layout.setSpacing(wizard.theme.space.md)

        self.body = QLabel(body, self)
        self.body.setWordWrap(True)
        layout.addWidget(self.body)
        self._layout = layout

        self.status = QLabel("", self)
        self.status.setProperty("variant", "caption")
        self.status.setWordWrap(True)

    @property
    def capture(self) -> _Capture:
        return self._wizard.capture

    def add_status(self) -> None:
        self._layout.addStretch(1)
        self._layout.addWidget(self.status)

    def initializePage(self) -> None:
        self._wizard.start_polling(self)

    def cleanupPage(self) -> None:
        self._wizard.stop_polling()

    def on_sample(self, sample: DeviceSnapshot) -> None:
        """Called at the poll rate while this page is on screen."""


class _IntroPage(_WizardPage):
    def __init__(self, wizard: "CalibrationWizard") -> None:
        super().__init__(
            wizard,
            "Calibrate " + (wizard.profile.name or "this device"),
            "You will be asked to move one control at a time. Nothing is sent "
            "to the car while this window is open.\n\n"
            "Take each control all the way to its mechanical stop. The wizard "
            "records what your hardware actually produces, which is almost "
            "never the range printed on the box.",
        )
        self.add_status()

    def initializePage(self) -> None:
        super().initializePage()
        # Restarting the wizard must not inherit half of the previous run.
        self._wizard.capture = _Capture()

    def on_sample(self, sample: DeviceSnapshot) -> None:
        if sample.connected:
            self.status.setText(
                "Device connected — %d axes, %d buttons."
                % (len(sample.axes), sample.button_count)
            )
        else:
            self.status.setText(
                "No device is reporting. Plug it in; the wizard will notice."
            )


class _SteerSweepPage(_WizardPage):
    def __init__(self, wizard: "CalibrationWizard") -> None:
        super().__init__(
            wizard,
            "Turn the wheel",
            "Turn all the way to the left stop, then all the way to the right "
            "stop. Take your time — the wizard is watching every control at "
            "once and will work out which one is the wheel.",
        )
        self.bar = _TravelBar(self, theme=wizard.theme)
        self._layout.addWidget(self.bar)
        self.add_status()
        self._baseline: tuple[float, ...] = ()
        self._lo = 0.0
        self._hi = 0.0
        self._index = -1

    def initializePage(self) -> None:
        super().initializePage()
        self._baseline = ()
        self._index = -1
        self.bar.configure(bipolar=True)
        self.bar.set_marks(None, None, None)

    def on_sample(self, sample: DeviceSnapshot) -> None:
        if not sample.axes:
            self.status.setText("No axes are reporting yet.")
            return
        if not self._baseline:
            self._baseline = sample.axes
            return

        index = _most_moved(sample.axes, self._baseline, self._index)
        if index < 0:
            self.status.setText("Waiting for the wheel to move.")
            return
        if index != self._index:
            self._index = index
            value = sample.axes[index]
            self._lo = self._hi = value

        value = sample.axes[self._index]
        self._lo = min(self._lo, value)
        self._hi = max(self._hi, value)
        self.bar.set_value(value)
        self.bar.set_marks(self._lo, self._hi, None)

        self.capture.steer_ref = axis_ref(self._index)
        self.capture.steer_lo = self._lo
        self.capture.steer_hi = self._hi

        span = self._hi - self._lo
        if span < _MIN_STEER_HALF * 2.0:
            self.status.setText("Keep going — reach both stops.")
        else:
            self.status.setText(
                "Got it. Both stops recorded across %.0f%% of the device range."
                % (span * 50.0,)
            )
        self.completeChanged.emit()

    def isComplete(self) -> bool:
        return self._index >= 0 and (self._hi - self._lo) >= _MIN_STEER_HALF * 2.0


class _SteerCentrePage(_WizardPage):
    def __init__(self, wizard: "CalibrationWizard") -> None:
        super().__init__(
            wizard,
            "Let the wheel centre",
            "Let go of the wheel and let it settle. The centre is measured "
            "rather than assumed: a wheel's electrical centre is rarely its "
            "mechanical one, and that offset is the single biggest lever on "
            "odometry heading drift.",
        )
        self.bar = _TravelBar(self, theme=wizard.theme)
        self._layout.addWidget(self.bar)
        self.add_status()
        self._settled = 0.0
        self._last = 0.0

    def initializePage(self) -> None:
        super().initializePage()
        self._settled = 0.0
        self.bar.configure(bipolar=True)

    def on_sample(self, sample: DeviceSnapshot) -> None:
        ref = self.capture.steer_ref
        if ref is None or ref.index >= len(sample.axes):
            self.status.setText("Lost the steering axis; go back one page.")
            return
        value = sample.axes[ref.index]
        self.bar.set_value(value)
        self.bar.set_marks(self.capture.steer_lo, self.capture.steer_hi, value)

        if abs(value - self._last) < 0.01:
            self._settled += _POLL_MS / 1000.0
        else:
            self._settled = 0.0
        self._last = value

        if self._settled < 0.5:
            self.status.setText("Hold still…")
        else:
            self.capture.steer_rest = value
            if self.capture.steer_ok():
                self.status.setText(
                    "Centre recorded. Left half %.2f, right half %.2f of the "
                    "device range."
                    % (value - self.capture.steer_lo, self.capture.steer_hi - value)
                )
            else:
                self.status.setText(
                    "Centre recorded, but it sits too close to one stop. Go "
                    "back and sweep the wheel fully."
                )
        self.completeChanged.emit()

    def isComplete(self) -> bool:
        return self._settled >= 0.5 and self.capture.steer_ok()


class _PedalPage(_WizardPage):
    def __init__(
        self, wizard: "CalibrationWizard", control: Control, title: str, body: str
    ) -> None:
        super().__init__(wizard, title, body)
        self.control = control
        self.bar = _TravelBar(self, theme=wizard.theme)
        self._layout.addWidget(self.bar)
        self.add_status()
        self._baseline: tuple[float, ...] = ()
        self._index = -1
        self._button = -1
        self._rest = 0.0
        self._full = 0.0

    def initializePage(self) -> None:
        super().initializePage()
        self._baseline = ()
        self._index = -1
        self._button = -1
        self.bar.configure(bipolar=False)
        self.bar.set_marks(None, None, None)

    def on_sample(self, sample: DeviceSnapshot) -> None:
        if not self._baseline and sample.axes:
            self._baseline = sample.axes

        # Digital pedals: two ordinary buttons where an analog pedal should be.
        # Checked first because a wheel in that configuration usually still
        # reports a dead axis that noise could win on.
        if self._button < 0 and sample.pressed_edges:
            self._button = (sample.pressed_edges & -sample.pressed_edges).bit_length() - 1

        if self._button < 0 and self._baseline:
            index = _most_moved(sample.axes, self._baseline, self._index)
            if index >= 0:
                if index != self._index:
                    self._index = index
                    self._rest = self._baseline[index]
                    self._full = sample.axes[index]
                value = sample.axes[index]
                if abs(value - self._rest) > abs(self._full - self._rest):
                    self._full = value
                self.bar.set_value(value)
                self.bar.set_marks(
                    min(self._rest, self._full), max(self._rest, self._full), self._rest
                )

        self._publish()
        self._describe(sample)
        self.completeChanged.emit()

    def _publish(self) -> None:
        if self._button >= 0:
            self.capture.pedals[self.control] = (button_ref(self._button), 0.0, 1.0)
        elif self._index >= 0:
            self.capture.pedals[self.control] = (
                axis_ref(self._index),
                self._rest,
                self._full,
            )

    def _describe(self, sample: DeviceSnapshot) -> None:
        if self._button >= 0:
            self.status.setText(
                "Recorded as an on/off control. The rate limit becomes your "
                "pedal travel — Settings has a gentler default for that."
            )
        elif self._index < 0:
            self.status.setText("Waiting for the pedal to move.")
        elif abs(self._full - self._rest) < _MIN_PEDAL_SPAN:
            self.status.setText("Press it all the way down.")
        else:
            self.status.setText(
                "Full travel recorded: %.2f at rest, %.2f pressed."
                % (self._rest, self._full)
            )

    def isComplete(self) -> bool:
        if self._button >= 0:
            return True
        return self._index >= 0 and abs(self._full - self._rest) >= _MIN_PEDAL_SPAN


class _LockPage(_WizardPage):
    def __init__(self, wizard: "CalibrationWizard") -> None:
        super().__init__(
            wizard,
            "Steering range",
            "Most wheels turn much further than a driver wants to use. "
            "Restricting the range makes full lock reachable without taking a "
            "hand off the rim; it is applied as a gain on the calibrated "
            "steering, so the measurement above stays untouched.",
        )
        row = QHBoxLayout()
        row.setSpacing(wizard.theme.space.md)

        self.rotation = QDoubleSpinBox(self)
        self.rotation.setRange(0.0, 1440.0)
        self.rotation.setSingleStep(90.0)
        self.rotation.setDecimals(0)
        self.rotation.setSuffix(" ° total")
        self.rotation.setValue(wizard.profile.device_rotation_deg)
        self.rotation.setSpecialValueText("unknown")
        row.addWidget(self.rotation)

        self.lock = QDoubleSpinBox(self)
        self.lock.setRange(0.0, 1440.0)
        self.lock.setSingleStep(30.0)
        self.lock.setDecimals(0)
        self.lock.setSuffix(" ° used")
        self.lock.setValue(wizard.profile.rotation_lock_deg)
        self.lock.setSpecialValueText("all of it")
        row.addWidget(self.lock)

        self.invert = QCheckBox("Steering is backwards", self)
        self.invert.setToolTip(
            "Separate from the measurement on purpose, so recalibrating can "
            "never double-invert an axis that was already correct."
        )
        self.invert.setChecked(
            wizard.profile.chain.steer.calibration.invert
        )
        row.addWidget(self.invert)
        row.addStretch(1)
        self._layout.addLayout(row)
        self.add_status()

    def initializePage(self) -> None:
        super().initializePage()
        self.status.setText(
            "Leave both at their defaults if you are not sure. They change "
            "feel, not safety."
        )

    def validatePage(self) -> bool:
        self.capture.rotation_deg = self.rotation.value()
        self.capture.lock_deg = self.lock.value()
        self.capture.invert_steer = self.invert.isChecked()
        return True


class _SummaryPage(_WizardPage):
    def __init__(self, wizard: "CalibrationWizard") -> None:
        super().__init__(
            wizard,
            "Check it",
            "Move each control. The bars below use the new calibration, so "
            "full travel should now reach the ends and centre should sit at "
            "the middle.",
        )
        self.steer = _TravelBar(self, theme=wizard.theme)
        self.throttle = _TravelBar(self, theme=wizard.theme)
        self.brake = _TravelBar(self, theme=wizard.theme)
        self.throttle.configure(bipolar=False)
        self.brake.configure(bipolar=False)
        for label, bar in (
            ("Steering", self.steer),
            ("Throttle", self.throttle),
            ("Brake", self.brake),
        ):
            caption = QLabel(label, self)
            caption.setProperty("variant", "eyebrow")
            self._layout.addWidget(caption)
            self._layout.addWidget(bar)
        self.add_status()

    def initializePage(self) -> None:
        super().initializePage()
        try:
            self._wizard.build_profile()
        except ChainConfigError as exc:
            self.status.setText(
                "That calibration is not usable: %s. Go back and repeat the "
                "control it names." % (exc,)
            )
            return
        self.status.setText("Looks good. Finish to save it against this device.")

    def on_sample(self, sample: DeviceSnapshot) -> None:
        profile = self._wizard.pending_profile
        if profile is None:
            return
        for control, bar in (
            (Control.STEER, self.steer),
            (Control.THROTTLE, self.throttle),
            (Control.BRAKE, self.brake),
        ):
            bar.set_value(_calibrated_value(profile, control, sample))

    def isComplete(self) -> bool:
        return self._wizard.pending_profile is not None


class CalibrationWizard(QWizard):
    """Measures one device and returns an updated :class:`DeviceProfile`."""

    def __init__(
        self,
        profile: DeviceProfile,
        sampler: Sampler,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
    ) -> None:
        super().__init__(parent)
        if profile is None:
            raise ValueError("the calibration wizard needs a device profile to edit")
        self.profile = profile
        self.theme = theme
        self.capture = _Capture()
        self.pending_profile: DeviceProfile | None = None

        self._sampler = sampler
        self._page: _WizardPage | None = None
        self._timer = QTimer(self)
        self._timer.setInterval(_POLL_MS)
        self._timer.timeout.connect(self._poll)

        self.setWizardStyle(QWizard.WizardStyle.ModernStyle)
        self.setOption(QWizard.WizardOption.NoBackButtonOnStartPage, True)
        self.setOption(QWizard.WizardOption.NoCancelButtonOnLastPage, False)
        self.setWindowTitle("Calibrate controls")
        self.setMinimumSize(QSize(620, 480))

        self.addPage(_IntroPage(self))
        self.addPage(_SteerSweepPage(self))
        self.addPage(_SteerCentrePage(self))
        self.addPage(
            _PedalPage(
                self,
                Control.THROTTLE,
                "Press the accelerator",
                "Push the accelerator all the way down, then release it. If "
                "your pedals are buttons, just press the one you use to "
                "accelerate.",
            )
        )
        self.addPage(
            _PedalPage(
                self,
                Control.BRAKE,
                "Press the brake",
                "Same again with the brake. A load-cell brake will not reach "
                "its stop — press it as hard as you intend to press it while "
                "driving, and that becomes full braking.",
            )
        )
        self.addPage(_LockPage(self))
        self.addPage(_SummaryPage(self))

    # -- polling ------------------------------------------------------------

    def start_polling(self, page: _WizardPage) -> None:
        self._page = page
        if not self._timer.isActive():
            self._timer.start()

    def stop_polling(self) -> None:
        self._page = None

    def _poll(self) -> None:
        page = self._page
        if page is None:
            return
        sample = self._sampler()
        page.on_sample(sample if sample is not None else EMPTY_SNAPSHOT)

    def done(self, result: int) -> None:
        self._timer.stop()
        super().done(result)

    # -- result -------------------------------------------------------------

    def build_profile(self) -> DeviceProfile:
        """Assemble the calibrated profile. Raises ``ChainConfigError``.

        Raising is right here: this runs when the summary page opens, which is
        construction time for the profile, and a calibration that cannot be
        represented must be corrected rather than silently softened.
        """
        capture = self.capture
        chain = self.profile.chain
        mapping = self.profile.mapping

        pre_gain = 1.0
        if capture.rotation_deg > 0.0 and 0.0 < capture.lock_deg < capture.rotation_deg:
            pre_gain = capture.rotation_deg / capture.lock_deg

        if capture.steer_ref is None:
            raise ChainConfigError("no steering control was captured")
        steer_cal = AxisCalibration.steering(
            capture.steer_lo,
            capture.steer_rest,
            capture.steer_hi,
            invert=capture.invert_steer,
            pre_gain=pre_gain,
        )
        chain = chain.with_axis(
            Control.STEER, _replace_calibration(chain.steer, steer_cal)
        )
        mapping = mapping.with_binding(
            Control.STEER, ControlBinding(positive=capture.steer_ref)
        )

        for control in (Control.THROTTLE, Control.BRAKE):
            captured = capture.pedals.get(control)
            if captured is None:
                continue
            ref, rest, full = captured
            calibration = (
                AxisCalibration.pedal(0.0, 1.0)
                if ref.kind is RefKind.BUTTON
                else AxisCalibration.pedal(rest, full)
            )
            chain = chain.with_axis(
                control, _replace_calibration(chain.axis(control), calibration)
            )
            mapping = mapping.with_binding(control, ControlBinding(positive=ref))

        profile = replace(
            self.profile.with_chain(chain).with_mapping(mapping),
            device_rotation_deg=capture.rotation_deg,
            rotation_lock_deg=capture.lock_deg,
        )
        self.pending_profile = profile
        return profile

    def result_profile(self) -> DeviceProfile | None:
        """The calibrated profile, or ``None`` if the driver cancelled."""
        return self.pending_profile if self.result() == QWizard.DialogCode.Accepted else None


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------


def _most_moved(
    axes: tuple[float, ...], baseline: tuple[float, ...], current: int
) -> int:
    """Index of the axis that has moved furthest from its baseline.

    Sticky: once an axis has won, it keeps winning unless another beats it
    outright. Without that, a wheel with a noisy unused axis flips the answer
    every few frames and the driver watches the marker jump between controls.
    """
    best = current
    best_delta = _DETECT_THRESHOLD
    if 0 <= current < len(axes) and current < len(baseline):
        best_delta = max(best_delta, abs(axes[current] - baseline[current]) * 0.9)
    for index in range(min(len(axes), len(baseline))):
        delta = abs(axes[index] - baseline[index])
        if delta > best_delta:
            best = index
            best_delta = delta
    return best


def _replace_calibration(
    axis: AxisChainConfig, calibration: AxisCalibration
) -> AxisChainConfig:
    return AxisChainConfig(
        calibration=calibration,
        deadzone=axis.deadzone,
        saturation=axis.saturation,
        curve=axis.curve,
        rate_rise=axis.rate_rise,
        rate_fall=axis.rate_fall,
        smoothing=axis.smoothing,
    )


def _calibrated_value(
    profile: DeviceProfile, control: Control, sample: DeviceSnapshot
) -> float:
    """Raw device state through the new calibration, for the preview bars.

    Only the calibration stage, deliberately -- deadzone, curve and rate limit
    are *feel* and belong to the settings screen. Showing them here would make
    a correctly calibrated pedal look like it stopped short.
    """
    binding = profile.mapping.binding(control)
    ref = binding.positive
    calibration = profile.chain.axis(control).calibration
    if ref.kind is RefKind.BUTTON:
        pressed = bool(sample.buttons & (1 << ref.index))
        value = 1.0 if pressed else 0.0
    elif ref.is_analog and ref.index < len(sample.axes):
        raw = sample.axes[ref.index]
        if calibration.bipolar:
            span = (
                (calibration.hi - calibration.rest)
                if raw >= calibration.rest
                else (calibration.rest - calibration.lo)
            )
            value = 0.0 if span <= 0.0 else (raw - calibration.rest) / span
        else:
            span = calibration.hi - calibration.rest
            value = 0.0 if span == 0.0 else (raw - calibration.rest) / span
    else:
        return 0.0
    value *= calibration.pre_gain
    if calibration.invert:
        value = -value
    return clamp(value, -1.0, 1.0) if calibration.bipolar else clamp(value, 0.0, 1.0)


__all__ = ["CalibrationWizard", "Sampler"]
