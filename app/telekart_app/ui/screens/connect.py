"""Finding a car, connecting to it, and proving the connection is sound.

Discovery lives here rather than in the model, and that is a deliberate layering
call: browsing the network is a *pre-connection* concern with no snapshot to
publish, and the model's job starts once there is a car to talk to. Keeping mDNS
out of the model also keeps the model importable in a headless test with no
network stack at all.

Discovery is never load-bearing. zeroconf may be missing, the browse may find
nothing, and a stale service record may point at a car that has since been
switched off -- so the hostname field is always present, always enabled, and is
the path the documentation tells people to use. A connect screen that can only
offer what it discovered is a connect screen that fails on exactly the congested
race-day network it is most needed on.

The compatibility panel exists because of one specific failure. A protocol
mismatch is refused by the handshake with a perfectly clear error message that
scrolls past in a log nobody is reading, and the visible symptom is "it just
does not connect". Showing both version numbers side by side turns ten minutes
of confusion into one glance.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

from PySide6.QtCore import QSize, Qt, QTimer, Signal
from PySide6.QtGui import QHideEvent, QShowEvent
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from telekart_protocol import PROTO_VERSION, ErrorCode

from ... import __version__
from ...model.snapshots import LinkSnapshot, LinkState, SessionSnapshot
from ...net.discovery import DEFAULT_HOSTNAME, DiscoveredCar, ZeroconfBrowser, resolve
from ..theme.qss import restyle
from ..theme.tokens import THEME, Theme
from ..widgets.card import Card
from ..widgets.link_quality import LinkQuality
from ..widgets.stat_tile import StatTile

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...model.app_model import AppModel

_COLUMNS = ("Car", "Address", "Firmware", "Found via", "Last seen")

#: How often the discovered table re-renders. Ages tick continuously, and
#: rebuilding a table at 60 Hz to advance a "3 s ago" label would be absurd.
_REFRESH_MS = 1000


class ConnectScreen(QWidget):
    """Discovered cars, a hostname field, and the evidence that the link works."""

    connectRequested = Signal(str, str, bool)  # host, shared key, remember
    disconnectRequested = Signal()
    simulatorRequested = Signal()

    #: Marshals discovery results from zeroconf's thread onto the GUI thread.
    #: Emitting a signal is the only safe way across that boundary; touching a
    #: QTableWidget from the browse thread is a crash waiting for a race.
    _carsChanged = Signal(list)

    def __init__(
        self,
        model: "AppModel",
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
    ) -> None:
        super().__init__(parent)
        self._model = model
        self._theme = theme
        self._cars: list[DiscoveredCar] = []
        self._link = model.link
        self._session = model.session
        self._closing = threading.Event()

        self._browser = ZeroconfBrowser(on_change=self._on_browser_thread)
        self._browsing = False

        self.setObjectName("Root")
        self._build(theme)

        self._refresh = QTimer(self)
        self._refresh.setInterval(_REFRESH_MS)
        self._refresh.timeout.connect(self._render_cars)

        self._carsChanged.connect(self._on_cars)
        model.linkChanged.connect(self._on_link)
        model.sessionChanged.connect(self._on_session)
        self._on_link(self._link)
        self._on_session(self._session)

    # -- construction -------------------------------------------------------

    def _build(self, theme: Theme) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        outer.addWidget(scroll)

        page = QWidget(scroll)
        scroll.setWidget(page)

        columns = QHBoxLayout(page)
        columns.setContentsMargins(
            theme.space.xl, theme.space.xl, theme.space.xl, theme.space.xl
        )
        columns.setSpacing(theme.space.lg)

        left = QVBoxLayout()
        left.setSpacing(theme.space.lg)
        right = QVBoxLayout()
        right.setSpacing(theme.space.lg)
        columns.addLayout(left, 3)
        columns.addLayout(right, 2)

        left.addWidget(self._build_discovery(theme))
        left.addWidget(self._build_manual(theme))
        left.addStretch(1)

        right.addWidget(self._build_link_card(theme))
        right.addWidget(self._build_compat_card(theme))
        right.addStretch(1)

    def _build_discovery(self, theme: Theme) -> Card:
        card = Card("Cars on this network", self, theme=theme,
                    subtitle="Double-click a car to connect")

        self._rescan = QPushButton("Rescan", card)
        self._rescan.setProperty("variant", "quiet")
        self._rescan.clicked.connect(self._rescan_now)
        card.add_action(self._rescan)

        self.table = QTableWidget(0, len(_COLUMNS), card)
        self.table.setHorizontalHeaderLabels(list(_COLUMNS))
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(False)
        self.table.setMinimumHeight(180)
        self.table.itemDoubleClicked.connect(self._connect_selected)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for index in range(1, len(_COLUMNS)):
            header.setSectionResizeMode(index, QHeaderView.ResizeMode.ResizeToContents)
        card.add_widget(self.table, 1)

        self._discovery_note = QLabel("", card)
        self._discovery_note.setProperty("variant", "caption")
        self._discovery_note.setWordWrap(True)
        card.add_widget(self._discovery_note)
        return card

    def _build_manual(self, theme: Theme) -> Card:
        card = Card("Connect by name", self, theme=theme)

        grid = QGridLayout()
        grid.setHorizontalSpacing(theme.space.md)
        grid.setVerticalSpacing(theme.space.sm)
        grid.setColumnStretch(1, 1)

        grid.addWidget(_field_label("Host", card), 0, 0)
        self.host = QLineEdit(card)
        self.host.setPlaceholderText(DEFAULT_HOSTNAME)
        self.host.returnPressed.connect(self._connect_manual)
        grid.addWidget(self.host, 0, 1)

        grid.addWidget(_field_label("Passphrase", card), 1, 0)
        self.passphrase = QLineEdit(card)
        self.passphrase.setEchoMode(QLineEdit.EchoMode.Password)
        self.passphrase.setPlaceholderText("the car's shared key")
        self.passphrase.returnPressed.connect(self._connect_manual)
        grid.addWidget(self.passphrase, 1, 1)

        self.remember = QCheckBox("Remember this passphrase for this car", card)
        self.remember.setChecked(True)
        grid.addWidget(self.remember, 2, 1)
        card.add_layout(grid)

        buttons = QHBoxLayout()
        buttons.setSpacing(theme.space.sm)
        self.connect_button = QPushButton("Connect", card)
        self.connect_button.setProperty("variant", "primary")
        self.connect_button.setDefault(True)
        self.connect_button.clicked.connect(self._connect_manual)
        buttons.addWidget(self.connect_button)

        self.disconnect_button = QPushButton("Disconnect", card)
        self.disconnect_button.clicked.connect(self.disconnectRequested.emit)
        buttons.addWidget(self.disconnect_button)
        buttons.addStretch(1)

        self.simulator_button = QPushButton("No car? Launch the simulator", card)
        self.simulator_button.setProperty("variant", "ghost")
        self.simulator_button.setToolTip(
            "Starts telekart-sim locally and connects to it. The simulator "
            "speaks the same protocol on the same ports, so everything in this "
            "application behaves exactly as it does with the real car."
        )
        self.simulator_button.clicked.connect(self.simulatorRequested.emit)
        buttons.addWidget(self.simulator_button)
        card.add_layout(buttons)

        self._error = QLabel("", card)
        self._error.setProperty("variant", "fault")
        self._error.setWordWrap(True)
        self._error.setVisible(False)
        card.add_widget(self._error)
        return card

    def _build_link_card(self, theme: Theme) -> Card:
        card = Card("Link", self, theme=theme)

        self.quality = LinkQuality(card, theme=theme)
        card.add_widget(self.quality)

        grid = QGridLayout()
        grid.setSpacing(theme.space.sm)
        self._tiles: dict[str, StatTile] = {}
        specs = (
            ("rtt", "Round trip", "ms", 0),
            ("loss", "Telemetry loss", "%", 2),
            ("tlm", "Telemetry", "Hz", 1),
            ("video", "Video", "fps", 1),
        )
        for index, (key, label, unit, decimals) in enumerate(specs):
            tile = StatTile(
                card, theme=theme, label=label, unit=unit, decimals=decimals, flat=True
            )
            tile.setMinimumHeight(64)
            self._tiles[key] = tile
            grid.addWidget(tile, index // 2, index % 2)
        card.add_layout(grid)

        self._link_detail = QLabel("", card)
        self._link_detail.setProperty("variant", "caption")
        self._link_detail.setWordWrap(True)
        card.add_widget(self._link_detail)
        return card

    def _build_compat_card(self, theme: Theme) -> Card:
        card = Card("Compatibility", self, theme=theme)
        grid = QGridLayout()
        grid.setHorizontalSpacing(theme.space.lg)
        grid.setVerticalSpacing(theme.space.xs)
        grid.setColumnStretch(1, 1)

        self._compat: dict[str, QLabel] = {}
        rows = (
            ("proto", "Protocol"),
            ("app", "This app"),
            ("fw", "Firmware"),
            ("car", "Car id"),
            ("session", "Session"),
        )
        for row, (key, label) in enumerate(rows):
            grid.addWidget(_field_label(label, card), row, 0)
            value = QLabel("--", card)
            value.setProperty("variant", "mono")
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            self._compat[key] = value
            grid.addWidget(value, row, 1)
        card.add_layout(grid)

        self._compat_note = QLabel("", card)
        self._compat_note.setWordWrap(True)
        self._compat_note.setVisible(False)
        card.add_widget(self._compat_note)
        return card

    # -- discovery ----------------------------------------------------------

    def showEvent(self, event: QShowEvent) -> None:
        self._start_browsing()
        self._refresh.start()
        super().showEvent(event)

    def hideEvent(self, event: QHideEvent) -> None:
        self._refresh.stop()
        super().hideEvent(event)

    def shutdown(self) -> None:
        """Stop the browse thread. Called by the window before it closes.

        Without this, zeroconf's own threads keep a reference to this widget
        and can deliver a callback into a half-destroyed Python object during
        interpreter teardown.
        """
        self._closing.set()
        if self._browsing:
            self._browsing = False
            self._browser.stop()

    def _start_browsing(self) -> None:
        if self._browsing or self._closing.is_set():
            return
        self._browsing = self._browser.start()
        if not self._browsing:
            self._discovery_note.setText(
                "mDNS browsing is unavailable, so this list will stay empty. "
                "Connecting by name still works -- try %s." % (DEFAULT_HOSTNAME,)
            )
        else:
            self._discovery_note.setText("")
        self._resolve_default()

    def _rescan_now(self) -> None:
        self._start_browsing()
        self._on_cars(self._browser.cars())
        self._resolve_default()

    def _resolve_default(self) -> None:
        """Look up the documented hostname alongside the browse.

        ``getaddrinfo`` cannot enumerate, but it is the resolver Finder uses and
        it answers from cache in microseconds when the car is up -- so the
        default car usually appears in this table before zeroconf has finished
        its first query. It runs on a throwaway thread because the same call
        blocks for five seconds when the car is *not* up.
        """
        host = self.host.text().strip() or DEFAULT_HOSTNAME

        def worker() -> None:
            found = resolve(host, timeout=2.0)
            if found is None or self._closing.is_set():
                return
            merged = [c for c in self._browser.cars() if c.address != found.address]
            merged.append(found)
            self._carsChanged.emit(merged)

        threading.Thread(target=worker, name="connectResolve", daemon=True).start()

    def _on_browser_thread(self, cars: list[DiscoveredCar]) -> None:
        if not self._closing.is_set():
            self._carsChanged.emit(cars)

    def _on_cars(self, cars: list[DiscoveredCar]) -> None:
        seen: dict[str, DiscoveredCar] = {}
        for car in cars:
            seen[car.address] = car
        self._cars = sorted(seen.values(), key=lambda c: (c.car_id or c.name, c.address))
        self._render_cars()

    def _render_cars(self) -> None:
        selected = self._selected_address()
        self.table.setRowCount(len(self._cars))
        now = time.time()
        for row, car in enumerate(self._cars):
            age = max(0.0, now - car.last_seen) if car.last_seen else 0.0
            values = (
                car.car_id or car.name,
                car.address,
                car.fw_version or "--",
                car.source,
                "just now" if age < 2.0 else "%.0f s ago" % (age,),
            )
            for column, text in enumerate(values):
                item = self.table.item(row, column)
                if item is None:
                    item = QTableWidgetItem()
                    self.table.setItem(row, column, item)
                if item.text() != text:
                    item.setText(text)
            if car.address == selected:
                self.table.selectRow(row)
        if self._cars and self.table.currentRow() < 0:
            self.table.selectRow(0)

    def _selected_address(self) -> str:
        row = self.table.currentRow()
        if row < 0 or row >= len(self._cars):
            return ""
        return self._cars[row].address

    # -- connect ------------------------------------------------------------

    def _connect_selected(self) -> None:
        row = self.table.currentRow()
        if 0 <= row < len(self._cars):
            car = self._cars[row]
            self._emit_connect(car.host or car.address)

    def _connect_manual(self) -> None:
        self._emit_connect(self.host.text().strip() or DEFAULT_HOSTNAME)

    def _emit_connect(self, host: str) -> None:
        self._error.setVisible(False)
        self.connectRequested.emit(
            host, self.passphrase.text(), self.remember.isChecked()
        )

    # -- model --------------------------------------------------------------

    def _on_link(self, link: LinkSnapshot) -> None:
        self._link = link
        connected = link.state.usable
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected or link.state is LinkState.FAILED)

        self.quality.set_channel(
            "ctrl", connected and link.control_hz > 0.0, 1.0 - link.loss, link.control_hz
        )
        self.quality.set_channel(
            "tlm", connected and link.telemetry_hz > 0.0, 1.0 - link.loss, link.telemetry_hz
        )
        self.quality.set_channel("vid", link.video_ok, 1.0, link.video_fps)
        self.quality.set_rtt_ms(link.rtt * 1000.0 if link.rtt > 0.0 else None)
        self.quality.set_loss(link.loss)

        self._tiles["rtt"].set_value(link.rtt * 1000.0)
        self._tiles["loss"].set_value(link.loss * 100.0)
        self._tiles["tlm"].set_value(link.telemetry_hz)
        self._tiles["video"].set_value(link.video_fps)

        self._link_detail.setText(
            _link_detail_text(link)
        )
        if link.state is LinkState.FAILED and link.detail:
            self._error.setText(link.detail)
            self._error.setVisible(True)
        elif connected:
            self._error.setVisible(False)

        self._compat["car"].setText(link.car_id or "--")
        self._compat["session"].setText(
            "--" if not link.session_id else "%d" % (link.session_id,)
        )

    def _on_session(self, session: SessionSnapshot) -> None:
        self._session = session
        self._compat["app"].setText(__version__)
        self._compat["fw"].setText(session.fw_version or "--")
        self._compat["proto"].setText(
            "%d — agreed" % (PROTO_VERSION,)
            if session.active
            else "%d — this build" % (PROTO_VERSION,)
        )
        if session.car_id:
            self._compat["car"].setText(session.car_id)

        # The handshake refuses a version mismatch outright, so if a session is
        # active the versions agree by construction. The panel's job is to make
        # the *refusal* legible, which is why the note only appears on failure.
        if session.last_error_code == ErrorCode.PROTOCOL_VERSION.value:
            self._compat_note.setProperty("variant", "fault")
            self._compat_note.setText(
                "The car speaks a different protocol version. Flash the same "
                "release on both ends -- the handshake refuses a mismatch "
                "rather than risk misreading a control packet."
            )
            self._compat_note.setVisible(True)
        elif session.last_error_code == ErrorCode.AUTH_FAILED.value:
            self._compat_note.setProperty("variant", "fault")
            self._compat_note.setText(
                "The passphrase was rejected. It is the car's shared key from "
                "its config.yaml, not the WiFi password."
            )
            self._compat_note.setVisible(True)
        elif session.active:
            self._compat_note.setProperty("variant", "caption")
            self._compat_note.setText("Handshake complete; versions agree.")
            self._compat_note.setVisible(True)
        else:
            self._compat_note.setVisible(False)
        # Qt does not re-run selector matching when a property used in one is
        # written, so the colour would not change without this.
        restyle(self._compat_note)

    def sizeHint(self) -> QSize:
        return QSize(1080, 700)


def _field_label(text: str, parent: QWidget) -> QLabel:
    label = QLabel(text, parent)
    label.setProperty("variant", "eyebrow")
    label.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Preferred)
    return label


def _link_detail_text(link: LinkSnapshot) -> str:
    if not link.state.usable:
        return link.detail or "Not connected."
    parts = ["%s via %s" % (link.state.value, link.address or link.host or "?")]
    if link.video_ok:
        parts.append("video %.0f fps, %.0f kbps" % (link.video_fps, link.video_bitrate / 1000.0))
    else:
        parts.append("no video")
    if link.rtt_p95 > 0.0:
        parts.append("rtt p95 %.0f ms" % (link.rtt_p95 * 1000.0,))
    return " · ".join(parts)


__all__ = ["ConnectScreen"]
