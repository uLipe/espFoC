"""Dashboard view: motion control, SVM, and scope channels."""

from __future__ import annotations

from PySide6.QtWidgets import QHBoxLayout, QSizePolicy, QWidget

from ..control_rail import ControlRail
from ..states_panel import StatesPanel
from ..svm_panel import SvmPanel
from ..widgets import PageShell, horizontal_splitter, vertical_splitter


class DashboardView(QWidget):
    def __init__(
        self,
        control: ControlRail,
        svm: SvmPanel,
        states: StatesPanel,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)

        control.bind_svm_autoset(svm.autoset)

        control.setMinimumWidth(260)
        control.setMaximumWidth(360)
        control.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)

        plots = vertical_splitter(
            svm,
            states,
            stretches=(2, 3),
            sizes=(340, 500),
        )

        split = horizontal_splitter(
            control,
            plots,
            stretches=(0, 1),
            sizes=(300, 960),
        )

        body = QWidget()
        lay = QHBoxLayout(body)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(split)

        shell = PageShell(
            "Dashboard",
            "Setpoints and actions on the left; SVM vector view and scope on the right.",
            body,
            parent=self,
        )
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(shell)
