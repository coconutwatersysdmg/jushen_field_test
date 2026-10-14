# -*- coding: utf-8 -*-
from __future__ import annotations

from PySide6.QtCore import QThread, QTimer, Signal
from PySide6.QtWidgets import QMessageBox, QPushButton

from services.network_switch_service import read_network_state, switch_network


class _NetworkWorker(QThread):
    resultReady = Signal(dict, str)

    def __init__(self, target: str | None, parent=None):
        super().__init__(parent)
        self.target = target

    def run(self):
        try:
            state = switch_network(self.target) if self.target else read_network_state()
            self.resultReady.emit(state, "")
        except Exception as exc:
            # Refresh even on failure: Wi-Fi may have changed before IP setup failed.
            try:
                state = read_network_state() if self.target else {}
            except Exception:
                state = {}
            self.resultReady.emit(state, str(exc))


class NetworkSwitchButton(QPushButton):
    busyChanged = Signal(bool)

    def __init__(self, parent=None):
        super().__init__("切换到雷达工控机", parent)
        self.worker = None
        self._switching = False
        self._flow_busy = False
        self._state = {}
        self._retry_target = None
        self._pending_result = ({}, "")
        self.clicked.connect(self._switch)
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(15000)
        self._poll_timer.timeout.connect(self._inspect)
        self._poll_timer.start()
        QTimer.singleShot(0, self._inspect)

    def is_switching(self):
        return self._switching

    def is_working(self):
        return self.worker is not None

    def set_flow_busy(self, busy):
        self._flow_busy = bool(busy)
        self.setEnabled(not self._switching and not self._flow_busy and self.worker is None)

    def _inspect(self):
        if self.worker is None:
            self._start_worker(None)

    def _switch(self):
        if self.worker is not None or self._flow_busy:
            return
        target = self._target()
        self._start_worker(target)

    def _target(self):
        if self._retry_target:
            return self._retry_target
        return "hotspot" if self._state.get("network") == "radar" else "radar"

    def _start_worker(self, target):
        self._switching = target is not None
        self.setEnabled(False)
        if self._switching:
            self.setText("正在切换…")
            self.setToolTip("请允许 Windows 管理员授权，然后等待 Wi-Fi 和 IP 配置完成。")
            self.busyChanged.emit(True)
        self._pending_result = ({}, "")
        self.worker = _NetworkWorker(target, self)
        self.worker.resultReady.connect(self._receive_result)
        self.worker.finished.connect(self._worker_finished)
        self.worker.start()

    def _receive_result(self, state, error):
        self._pending_result = (state, error)

    def _worker_finished(self):
        was_switching = self._switching
        worker = self.worker
        target = worker.target
        self.worker = None
        worker.deleteLater()
        self._switching = False
        state, error = self._pending_result
        # Unknown state must not pretend the last successful state is still current.
        self._state = state
        if was_switching:
            self._retry_target = target if error else None
        if state.get("ready") and state.get("network") == self._retry_target:
            self._retry_target = None
        self.setText("切换到热点" if self._target() == "hotspot" else "切换到雷达工控机")
        current = state.get("ssid") or "未连接 / 未能读取"
        addresses = ", ".join(state.get("addresses") or []) or "未获取"
        readiness = "配置已就绪" if state.get("ready") else "配置未就绪"
        self.setToolTip(f"当前 Wi-Fi：{current}\nIPv4：{addresses}\n{readiness}" + (f"\n{error}" if error else ""))
        self.set_flow_busy(self._flow_busy)
        if was_switching:
            self.busyChanged.emit(False)
            if error:
                QMessageBox.warning(self.window(), "网络切换未完成", error + "\n\n请查看按钮提示中的当前网络状态；可以重试或使用原来的桌面 BAT。")
            else:
                self.window().statusBar().showMessage(f"已切换到 {current}，IPv4：{addresses}。", 10000)
