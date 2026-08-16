"""The standard content surface: a titled panel with a body and an action slot.

Card is a real styled ``QFrame`` rather than a hand-painted widget, and that is
a deliberate exception to how the rest of this package works. Cards contain
ordinary Qt controls -- buttons, spin boxes, tables -- and those are painted by
the stylesheet. A hand-painted parent underneath stylesheet-painted children is
precisely the seam that makes an application look assembled from two different
programs, so the container stays on the stylesheet's side of the line and reads
its spacing from the same token object the instruments use.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QLayout,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..theme.qss import restyle
from ..theme.tokens import THEME, Theme


class Card(QFrame):
    """Titled surface. ``body`` is where callers put their content."""

    def __init__(
        self,
        title: str = "",
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        subtitle: str = "",
        dense: bool = False,
    ) -> None:
        super().__init__(parent)
        self._theme = theme
        self._dense = dense
        self.setObjectName("Card")
        # QFrame paints its own background only when told to; without this the
        # stylesheet's background-color is ignored on some styles and the card
        # is invisible against the page.
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setFrameShape(QFrame.Shape.NoFrame)

        pad = theme.space.md if dense else theme.space.lg
        gap = theme.space.sm if dense else theme.space.md

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self._header = QWidget(self)
        self._header.setObjectName("CardHeader")
        self._header.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        header_layout = QHBoxLayout(self._header)
        header_layout.setContentsMargins(pad, theme.space.sm, pad, theme.space.sm)
        header_layout.setSpacing(theme.space.sm)

        titles = QVBoxLayout()
        titles.setContentsMargins(0, 0, 0, 0)
        titles.setSpacing(0)

        self._title = QLabel(title.upper(), self._header)
        self._title.setObjectName("CardTitle")
        titles.addWidget(self._title)

        self._subtitle = QLabel(subtitle, self._header)
        self._subtitle.setProperty("variant", "caption")
        self._subtitle.setVisible(bool(subtitle))
        titles.addWidget(self._subtitle)

        header_layout.addLayout(titles)
        header_layout.addStretch(1)

        self._actions = QHBoxLayout()
        self._actions.setContentsMargins(0, 0, 0, 0)
        self._actions.setSpacing(theme.space.xs)
        header_layout.addLayout(self._actions)

        self._header.setVisible(bool(title) or bool(subtitle))
        outer.addWidget(self._header)

        self._body = QWidget(self)
        self._body.setObjectName("CardBody")
        self._body_layout = QVBoxLayout(self._body)
        self._body_layout.setContentsMargins(pad, gap, pad, pad)
        self._body_layout.setSpacing(gap)
        outer.addWidget(self._body, 1)

        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)

    # -- content ------------------------------------------------------------

    @property
    def body(self) -> QVBoxLayout:
        """The body layout. Add content here."""
        return self._body_layout

    @property
    def body_widget(self) -> QWidget:
        return self._body

    def add_widget(self, widget: QWidget, stretch: int = 0) -> QWidget:
        self._body_layout.addWidget(widget, stretch)
        return widget

    def add_layout(self, layout: QLayout, stretch: int = 0) -> QLayout:
        self._body_layout.addLayout(layout, stretch)
        return layout

    def add_stretch(self, stretch: int = 1) -> None:
        self._body_layout.addStretch(stretch)

    def add_action(self, widget: QWidget) -> QWidget:
        """Put a control in the header, right-aligned."""
        self._actions.addWidget(widget)
        self._header.setVisible(True)
        return widget

    def set_body_margins(self, left: int, top: int, right: int, bottom: int) -> None:
        """For bodies that hold a single edge-to-edge child, e.g. a video view."""
        self._body_layout.setContentsMargins(left, top, right, bottom)

    # -- header -------------------------------------------------------------

    def set_title(self, title: str) -> None:
        self._title.setText(title.upper())
        self._header.setVisible(bool(title) or self._subtitle.isVisible())

    def set_subtitle(self, subtitle: str) -> None:
        self._subtitle.setText(subtitle)
        self._subtitle.setVisible(bool(subtitle))
        self._header.setVisible(bool(self._title.text()) or bool(subtitle))

    def set_highlighted(self, highlighted: bool) -> None:
        """Draw the card's border in the accent colour.

        Used for "this panel is the one that needs attention" -- an active
        calibration step, the pane holding keyboard focus. The property is read
        by the stylesheet, so it needs an explicit re-polish; Qt does not
        re-run selector matching when a dynamic property changes.
        """
        self.setProperty("highlighted", bool(highlighted))
        restyle(self)

    @property
    def theme(self) -> Theme:
        return self._theme


__all__ = ["Card"]
