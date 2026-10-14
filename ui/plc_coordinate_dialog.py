"""Editable DB addresses and asynchronous, read-only live PLC coordinates."""
from __future__ import annotations

from datetime import datetime, timedelta, timezone
from pathlib import Path
from copy import deepcopy
from threading import Event

from PySide6.QtCore import QThread, Signal, Qt, QTimer
from PySide6.QtGui import QIntValidator
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QSpinBox,
    QComboBox, QCheckBox, QPushButton, QTableWidget, QHeaderView, QWidget,
    QGridLayout, QGroupBox, QTableWidgetItem, QAbstractItemView, QTabWidget,
)

from services.plc_coordinate_reader import (
    AXES, CONFIG_PATH, CoordinateReader, create_client,
    default_config, load_config, save_config, validate_config,
    default_compensation_config,
)


class CoordinatePoller(QThread):
    sample = Signal(dict, str)
    status = Signal(str)

    def __init__(self, config: dict, client_factory=create_client, probe=False, parent=None):
        super().__init__(parent)
        self.config = config
        self.client_factory = client_factory
        self.probe = probe
        self.cancelled = Event()

    def stop(self):
        self.cancelled.set()

    def run(self):
        try:
            validate_config(self.config, require_addresses=not self.probe)
        except Exception as exc:
            self.status.emit(f'配置错误：{exc}')
            return
        while not self.cancelled.is_set():
            client = None
            try:
                self.status.emit('正在建立 S7 连接…')
                client = self.client_factory()
                client.connect(self.config['ip'], self.config['rack'], self.config['slot'],
                               tcp_port=self.config['port'])
                if self.cancelled.is_set():
                    return
                if self.probe:
                    self.status.emit('S7 连接成功；DB 读取权限和地址需持续读取后验证')
                    return
                reader = CoordinateReader(self.config, client)
                while not self.cancelled.is_set():
                    values = reader.read_once(self.cancelled.is_set)
                    if self.cancelled.is_set():
                        return
                    self.sample.emit(values, datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='milliseconds'))
                    if self.cancelled.wait(self.config['interval_ms'] / 1000):
                        return
            except InterruptedError:
                return
            except Exception as exc:
                if not self.cancelled.is_set():
                    self.status.emit(f'连接/读取失败：{exc}。2 秒后自动重试；当前数据显示为无效。')
            finally:
                if client is not None:
                    try:
                        client.disconnect()
                    except Exception:
                        pass
            if self.probe or self.cancelled.wait(2):
                return


class PlcCoordinateDialog(QDialog):
    def __init__(self, parent=None, *, config_path=CONFIG_PATH, client_factory=create_client):
        super().__init__(parent)
        self.config_path = Path(config_path)
        self.client_factory = client_factory
        self.worker = None
        self._stopping = False
        self._close_pending = False
        self._editing = False
        self._restart_pending = False
        self._truck_config = {'enabled': False}
        self._compensation_config = default_compensation_config()
        self.setWindowTitle('PLC 实时数据 · 轴坐标 / 车厢信息')
        self.setWindowModality(Qt.WindowModality.NonModal)
        self.resize(1120, 660)
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        self.data_tabs = QTabWidget()
        dashboard = QWidget()
        self._build_dashboard(QVBoxLayout(dashboard))
        self.data_tabs.addTab(dashboard, '车厢实时数据')
        self.data_tabs.addTab(self._build_compensation_page(), '人工补偿')
        layout.addWidget(self.data_tabs, 1)
        self.settings_widget = QWidget()
        settings = QVBoxLayout(self.settings_widget)
        settings.setContentsMargins(0, 0, 0, 0)
        network = QHBoxLayout()
        self.ip_input = QLineEdit()
        self.ip_input.setMaximumWidth(150)
        network.addWidget(QLabel('PLC IP'))
        network.addWidget(self.ip_input)
        self.network_fields = {}
        for key, label, low, high in [('port', '端口', 1, 65535), ('rack', 'Rack', 0, 7),
                                      ('slot', 'Slot', 0, 31), ('interval_ms', '间隔 ms', 100, 60000)]:
            spin = QSpinBox()
            spin.setRange(low, high)
            self.network_fields[key] = spin
            network.addWidget(QLabel(label))
            network.addWidget(spin)
        settings.addLayout(network)
        self.table = QTableWidget(4, 6)
        self.table.setHorizontalHeaderLabels(['轴', '启用', 'DB 编号', '字节偏移', '类型', '当前值（原始单位）'])
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setMinimumHeight(185)
        self.axis_fields = {}
        for row, name in enumerate(AXES):
            enabled = QCheckBox()
            db, offset = QLineEdit(), QLineEdit()
            db.setValidator(QIntValidator(1, 65535, db))
            offset.setValidator(QIntValidator(0, 2097144, offset))
            db.setPlaceholderText('待填写')
            offset.setPlaceholderText('待填写')
            data_type = QComboBox()
            data_type.addItems(['REAL', 'LREAL', 'DINT'])
            value = QLabel('—')
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            fields = {'enabled': enabled, 'db': db, 'offset': offset, 'type': data_type, 'value': value}
            self.axis_fields[name] = fields
            for column, widget in enumerate([QLabel(name), enabled, db, offset, data_type, value]):
                self.table.setCellWidget(row, column, widget)
        settings.addWidget(self.table)
        self.settings_widget.hide()
        self.settings_toggle = QPushButton('连接与地址设置 ▸')
        self.settings_toggle.setCheckable(True)
        self.settings_toggle.toggled.connect(self._toggle_settings)
        layout.addWidget(self.settings_toggle)
        layout.addWidget(self.settings_widget)
        buttons = QHBoxLayout()
        self.save_btn = QPushButton('保存配置')
        self.probe_btn = QPushButton('测试 S7 连接', self.settings_widget)
        self.start_btn = QPushButton('开始读取', self.settings_widget)
        self.stop_btn = QPushButton('停止读取', self.settings_widget)
        self.stop_btn.setEnabled(False)
        for button in [self.probe_btn, self.start_btn, self.stop_btn]:
            button.hide()
        buttons.addWidget(self.save_btn)
        settings.addLayout(buttons)
        self.status_label = QLabel('等待自动连接…')
        self.status_label.setWordWrap(True)
        self.timestamp_label = QLabel('最近成功读取：—')
        layout.addWidget(self.status_label)
        layout.addWidget(self.timestamp_label)
        warning = QLabel('只读监视 · XYZR 为原始编码值；尺寸及角度单位以 PLC 定义为准。前板/后板按参考图对应高板/低板。')
        warning.setWordWrap(True)
        layout.addWidget(warning)
        self.save_btn.clicked.connect(self.save_settings)
        self.probe_btn.clicked.connect(self.test_connection)
        self.start_btn.clicked.connect(self.start_reading)
        self.stop_btn.clicked.connect(self.stop_reading)
        try:
            config = load_config(self.config_path)
        except ValueError as exc:
            config = default_config()
            self.status_label.setText(f'配置加载失败：{exc}。修正后保存配置。')
        self._apply_config(config)

    def _apply_config(self, config):
        self._truck_config = dict(config.get('truck', {'enabled': False}))
        self._compensation_config = deepcopy(config.get('compensation', default_compensation_config()))
        self.ip_input.setText(config['ip'])
        for name, spin in self.network_fields.items():
            spin.setValue(config[name])
        for name, fields in self.axis_fields.items():
            item = config['axes'][name]
            fields['enabled'].setChecked(item['enabled'])
            for key in ('db', 'offset'):
                fields[key].setText('' if item[key] is None else str(item[key]))
            fields['type'].setCurrentText(item['type'])

    def _config(self):
        config = {'ip': self.ip_input.text().strip(), 'truck': dict(self._truck_config),
                  'compensation': deepcopy(self._compensation_config),
                  **{key: spin.value() for key, spin in self.network_fields.items()}, 'axes': {}}
        for name, fields in self.axis_fields.items():
            item = {'enabled': fields['enabled'].isChecked(), 'type': fields['type'].currentText()}
            for key in ('db', 'offset'):
                text = fields[key].text().strip()
                try:
                    item[key] = int(text) if text else None
                except ValueError as exc:
                    raise ValueError(f'{name} {key} 必须是整数') from exc
            config['axes'][name] = item
        validate_config(config)
        return config

    def save_settings(self):
        try:
            save_config(self.config_path, self._config())
            self.status_label.setText('配置已保存；收起设置后自动连接')
        except (ValueError, OSError) as exc:
            self.status_label.setText(f'保存失败：{exc}')

    def is_running(self):
        # Retain worker ownership until its queued finished signal is handled.
        return self.worker is not None

    def _clear_values(self):
        for fields in self.axis_fields.values():
            fields['value'].setText('—')
        for label in (*self.axis_values.values(), *self.truck_values.values()):
            label.setText('—')
        for table in self.vertex_tables:
            for row in range(3):
                for column in range(4):
                    table.item(row, column).setText('—')
        for key, groups in self.compensation_values.items():
            for group, label in groups.items():
                configured = self._compensation_config.get('enabled') and self._compensation_config[group].get(key) is not None
                label.setText('—' if configured else '待配置')

    def _set_busy(self, busy):
        self.settings_widget.setEnabled(not busy)
        for button in (self.start_btn, self.save_btn, self.probe_btn):
            button.setEnabled(not busy)
        self.stop_btn.setEnabled(busy and not self._stopping)

    def _start(self, probe=False):
        if self.is_running():
            return
        try:
            config = self._config()
            validate_config(config, require_addresses=not probe)
        except (ValueError, OSError) as exc:
            self.status_label.setText(f'待配置 / 无法启动：{exc}')
            return
        self._stopping = False
        self._clear_values()
        self.timestamp_label.setText('最近成功读取：—')
        self.status_label.setText('正在建立 S7 连接…')
        self.worker = CoordinatePoller(config, self.client_factory, probe, self)
        self.worker.sample.connect(self._sample)
        self.worker.status.connect(self._status)
        self.worker.finished.connect(self._finished)
        self._set_busy(True)
        self.worker.start()

    def start_reading(self):
        self._start()

    def test_connection(self):
        self._start(probe=True)

    def _sample(self, values, timestamp):
        if self._stopping:
            return
        for name, value in values.items():
            if name in AXES:
                text = str(value) if isinstance(value, int) else f'{value:.6f}'
                self.axis_fields[name]['value'].setText(text)
                self.axis_values[name].setText(text)
        truck = values.get('truck', {})
        for key, label in self.truck_values.items():
            if key not in truck:
                label.setText('—')
            elif key == 'front_baffle':
                value = truck[key]
                label.setText({1: '有（1）', 2: '无（2）'}.get(value, f'未确认（{value}）'))
            elif key == 'dunnage':
                label.setText(f'{truck[key]}（含义待确认）')
            else:
                label.setText(str(truck[key]))
        for point in range(1, 9):
            table = self.vertex_tables[(point - 1) // 4]
            for row, axis in enumerate(('x', 'y', 'z')):
                table.item(row, (point - 1) % 4).setText(str(truck.get(f'p{point}_{axis}', '—')))
        compensation = values.get('compensation', {})
        for key, groups in self.compensation_values.items():
            for group, label in groups.items():
                value = compensation.get(group, {}).get(key)
                configured = self._compensation_config.get('enabled') and self._compensation_config[group].get(key) is not None
                label.setText(str(value) if value is not None else ('—' if configured else '待配置'))
        self.status_label.setText('读取中 · 当前值来自 PLC DB')
        self.timestamp_label.setText(f'最近成功读取：{timestamp}')

    def _status(self, text):
        if self._stopping:
            return
        self.status_label.setText(text)
        self._clear_values()

    def stop_reading(self):
        if self.worker is not None:
            self._stopping = True
            self.worker.stop()
            self.status_label.setText('正在停止，等待当前网络请求结束…')
            self._clear_values()
            self.stop_btn.setEnabled(False)

    def _finished(self):
        worker = self.worker
        self.worker = None
        if worker is not None:
            worker.deleteLater()
        if self._stopping:
            self.status_label.setText('已停止 · 当前坐标无效')
        self._set_busy(False)
        if self._close_pending:
            self.close()
        elif self._restart_pending:
            self._restart_pending = False
            self._auto_start()

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(0, self._auto_start)

    def _auto_start(self):
        if self.isVisible() and not self._editing and not self._close_pending:
            self.start_reading()

    def _toggle_settings(self, expanded):
        self._editing = expanded
        self.settings_widget.setVisible(expanded)
        self.settings_toggle.setText('连接与地址设置 ▾' if expanded else '连接与地址设置 ▸')
        if expanded:
            self.stop_reading()
        else:
            try:
                save_config(self.config_path, self._config())
            except (ValueError, OSError) as exc:
                self.status_label.setText(f'配置错误：{exc}')
                return
            if self.is_running():
                self._restart_pending = True
            else:
                self._auto_start()

    def _build_dashboard(self, layout):
        axes_box = QGroupBox('实时轴坐标 · 当前编码值')
        axes_layout = QHBoxLayout(axes_box)
        self.axis_values = {}
        for name in AXES:
            card = QGroupBox(f'{name} 轴')
            card_layout = QVBoxLayout(card)
            label = QLabel('—')
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet('font-size: 28px; font-weight: bold; color: #087a9b; padding: 12px;')
            label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            self.axis_values[name] = label
            card_layout.addWidget(label)
            axes_layout.addWidget(card)
        layout.addWidget(axes_box)
        geometry = QGroupBox('车厢信息 · 雷达处理数据')
        grid = QGridLayout(geometry)
        self.truck_values = {}
        info = QGroupBox('车厢尺寸')
        info_grid = QGridLayout(info)
        for row, (key, title) in enumerate([('length', '长度'), ('width', '宽度'),
                                            ('height', '高度'), ('angle', '偏转角度')]):
            label = self._number_label()
            self.truck_values[key] = label
            info_grid.addWidget(QLabel(title), row, 0)
            info_grid.addWidget(label, row, 1)
        grid.addWidget(info, 0, 0, 2, 1)
        self.vertex_tables = []
        for index in range(2):
            table = QTableWidget(3, 4)
            table.setHorizontalHeaderLabels([f'顶点 {point}' for point in range(index * 4 + 1, index * 4 + 5)])
            table.setVerticalHeaderLabels(['X', 'Y', 'Z'])
            table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
            table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
            table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
            table.setMinimumHeight(155)
            for row in range(3):
                for column in range(4):
                    item = QTableWidgetItem('—')
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    table.setItem(row, column, item)
            self.vertex_tables.append(table)
            grid.addWidget(table, 0, index + 1)
        boards = QGroupBox('高低板尺寸')
        board_grid = QGridLayout(boards)
        board_grid.addWidget(QLabel('高板 / 前车板'), 0, 1)
        board_grid.addWidget(QLabel('低板 / 后车板'), 0, 2)
        for row, (key, title) in enumerate([('length', '长度'), ('width', '宽度'), ('height', '高度')], 1):
            board_grid.addWidget(QLabel(title), row, 0)
            for column, prefix in enumerate(('front', 'rear'), 1):
                label = self._number_label()
                self.truck_values[f'{prefix}_{key}'] = label
                board_grid.addWidget(label, row, column)
        grid.addWidget(boards, 1, 1, 1, 2)
        flags = QHBoxLayout()
        for key, title in [('dunnage', '有无垫木'), ('front_baffle', '有无前挡板')]:
            label = self._number_label()
            label.setToolTip('DB112 偏移 548，INT；编码含义待现场确认' if key == 'dunnage'
                             else 'DB112 偏移 550，INT；按参考图 1=有、2=无')
            self.truck_values[key] = label
            flags.addWidget(QLabel(title))
            flags.addWidget(label, 1)
        grid.addLayout(flags, 2, 0, 1, 3)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 2)
        grid.setColumnStretch(2, 2)
        layout.addWidget(geometry, 1)

    @staticmethod
    def _number_label():
        label = QLabel('—')
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        label.setStyleSheet('background: #eaf1f6; color: #17364a; border: 1px solid #c6d4df; padding: 8px; font-size: 18px;')
        return label

    def _build_compensation_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        title = QLabel('人工补偿装车数据')
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet('font-size: 24px; font-weight: bold; color: #17364a; padding: 12px;')
        layout.addWidget(title)
        note = QLabel('只读监视：输入值显示 PLC 中已有的设定值。本页不修改补偿参数，不下发确认、清零或重新计算指令。')
        note.setWordWrap(True)
        layout.addWidget(note)
        grid = QGridLayout()
        for column, text in [(0, '补偿项目'), (1, '修改后值'), (2, '输入值 / 设定值'), (3, '方向与作用说明')]:
            label = QLabel(text)
            label.setStyleSheet('font-weight: bold; color: #087a9b; padding: 8px;')
            grid.addWidget(label, 0, column)
        rows = [
            ('d_AxE1', 'X轴起始坐标 Ax–E1 补偿', '参考范围：−200 ～ 200', '为正时，放置点 E1 整体靠向车头；为负时靠向车尾。'),
            ('d_AxE2', 'X轴起始坐标 Ax–E2 补偿', '参考范围：−200 ～ 200', '为正时，放置点 E2 整体靠向车头；为负时靠向车尾。'),
            ('d_Ay', 'Y轴起始坐标 Ay 补偿', '参考范围：−200 ～ 200', '为正时，放置点整体靠向驾驶室一侧；为负时靠向副驾驶一侧。'),
            ('d_dx', 'X轴托盘中间预留缝隙', '参考范围：−10 ～ 30', '数值越大，X轴前后两列托盘之间的缝隙越大。'),
            ('d_dy', 'Y轴托盘中间预留缝隙', '参考范围：−10 ～ 30', '数值越大，Y轴左右两排托盘之间的缝隙越大。'),
        ]
        self.compensation_values = {}
        for row, (key, name, value_range, description) in enumerate(rows, 1):
            name_label = QLabel(f'{name}\n{key}')
            name_label.setWordWrap(True)
            grid.addWidget(name_label, row, 0)
            self.compensation_values[key] = {}
            for column, group in [(1, 'applied'), (2, 'input')]:
                value = self._number_label()
                value.setText('待配置')
                self.compensation_values[key][group] = value
                grid.addWidget(value, row, column)
            comment = QLabel(f'{description}\n{value_range}')
            comment.setWordWrap(True)
            comment.setStyleSheet('color: #b43333; padding: 8px;')
            grid.addWidget(comment, row, 3)
        grid.setColumnStretch(0, 3)
        grid.setColumnStretch(1, 2)
        grid.setColumnStretch(2, 2)
        grid.setColumnStretch(3, 5)
        layout.addLayout(grid, 1)
        pending = QLabel('读取地址待确认：请提供 DB102 中“设定值”和“修改后值”对应五项变量的字节偏移。\n参考范围仅作说明；数值单位、实际应用与计算结果以现场 PLC 程序为准。')
        pending.setWordWrap(True)
        layout.addWidget(pending)
        return page

    def reject(self):
        # Escape follows reject(), not closeEvent(), in QDialog.
        if self.is_running():
            self._close_pending = True
            self.stop_reading()
            return
        super().reject()

    def closeEvent(self, event):
        if self.is_running():
            self._close_pending = True
            self.stop_reading()
            event.ignore()
            return
        self._close_pending = False
        super().closeEvent(event)
