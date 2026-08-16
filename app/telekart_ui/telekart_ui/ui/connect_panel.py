"""The connect panel: a host and a button.

An overlay rather than a screen. It covers the picture when there is no car and
gets out of the way when there is, so the app has exactly one place to be. There
is no car list: `telekart.local` resolves through the OS, which is what a person
types and what the docs already tell them to type.

There is no passphrase either. This build has no shared key -- reaching the car
is the only credential -- so the panel is a text field and two buttons.

This is the only place in the app with stock Qt widgets on screen, which is why
it is also the only place the stylesheet has anything to do.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QPainter
from PySide6.QtWidgets import QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget

from ..model.snapshots import LinkSnapshot, LinkState
from ..net.discovery import DEFAULT_HOSTNAME
from .theme import C, with_alpha


class ConnectPanel(QWidget):
    """Covers the video when there is no car. Dismisses itself when there is."""

    connectRequested = Signal(str)  # host
    disconnectRequested = Signal()
    dismissed = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("ConnectScrim")
        self.setAutoFillBackground(False)

        self._card = QWidget(self)
        self._card.setObjectName("ConnectPanel")
        self._card.setFixedWidth(400)

        title = QLabel("Connect to a car", self._card)
        title.setProperty("variant", "title")

        self._host = QLineEdit(self._card)
        self._host.setPlaceholderText(DEFAULT_HOSTNAME)
        self._host.setClearButtonEnabled(True)

        self._status = QLabel("", self._card)
        self._status.setProperty("variant", "caption")
        self._status.setWordWrap(True)

        self._connect = QPushButton("Connect", self._card)
        self._connect.setProperty("variant", "primary")
        self._connect.setDefault(True)
        self._disconnect = QPushButton("Disconnect", self._card)
        self._disconnect.setEnabled(False)

        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        buttons.addWidget(self._disconnect)
        buttons.addStretch(1)
        buttons.addWidget(self._connect)

        layout = QVBoxLayout(self._card)
        layout.setContentsMargins(24, 22, 24, 20)
        layout.setSpacing(10)
        layout.addWidget(title)
        layout.addWidget(_caption("Car", self._card))
        layout.addWidget(self._host)
        layout.addWidget(self._status)
        layout.addSpacing(4)
        layout.addLayout(buttons)

        self._connect.clicked.connect(self._emit_connect)
        self._disconnect.clicked.connect(self.disconnectRequested)
        self._host.returnPressed.connect(self._emit_connect)

    # -- state --------------------------------------------------------------

    def prefill(self, host: str) -> None:
        self._host.setText(host)
        self._host.setFocus()
        self._host.selectAll()

    def set_link(self, link: LinkSnapshot) -> None:
        state = link.state
        busy = state in (LinkState.RESOLVING, LinkState.CONNECTING)
        self._connect.setEnabled(not busy)
        self._disconnect.setEnabled(state.usable)
        self._connect.setText("Connecting…" if busy else "Connect")

        if state is LinkState.FAILED:
            self._status.setProperty("variant", "error")
            self._status.setText(link.detail or "connection failed")
        elif busy:
            self._status.setProperty("variant", "caption")
            self._status.setText(link.detail or "…")
        elif state.usable:
            self._status.setProperty("variant", "caption")
            self._status.setText(f"connected to {link.car_id or link.address}")
        else:
            self._status.setProperty("variant", "caption")
            self._status.setText("")
        # Re-polish: Qt does not restyle on a property change by itself, so
        # without this an error stays red after the next successful connect.
        self._status.style().unpolish(self._status)
        self._status.style().polish(self._status)

    def _emit_connect(self) -> None:
        self.connectRequested.emit(self._host.text().strip() or DEFAULT_HOSTNAME)

    # -- presentation -------------------------------------------------------

    def resizeEvent(self, event: object) -> None:  # noqa: D102
        self._card.move(
            (self.width() - self._card.width()) // 2,
            max(24, (self.height() - self._card.sizeHint().height()) // 2),
        )
        super().resizeEvent(event)  # type: ignore[arg-type]

    def paintEvent(self, event: object) -> None:  # noqa: D102
        painter = QPainter(self)
        # A scrim, not an opaque fill: when the panel is summoned mid-session
        # over a live picture, the driver can still see the car.
        painter.fillRect(self.rect(), with_alpha(C.bg_base, 0.86))
        painter.end()

    def keyPressEvent(self, event: object) -> None:  # noqa: D102
        if event.key() == Qt.Key.Key_Escape:  # type: ignore[attr-defined]
            self.dismissed.emit()
            event.accept()  # type: ignore[attr-defined]
            return
        super().keyPressEvent(event)  # type: ignore[arg-type]


def _caption(text: str, parent: QWidget) -> QLabel:
    label = QLabel(text, parent)
    label.setProperty("variant", "caption")
    return label


__all__ = ["ConnectPanel"]
