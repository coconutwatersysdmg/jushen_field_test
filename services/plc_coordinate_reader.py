"""Read-only S7 DB coordinate access, independent of motion control and Qt."""
from __future__ import annotations

import ipaddress
import json
import math
import os
import struct
import tempfile
from pathlib import Path
from typing import Callable

AXES = ('X', 'Y', 'Z', 'R')
COMPENSATION_KEYS = ('d_AxE1', 'd_AxE2', 'd_Ay', 'd_dx', 'd_dy')
CONFIG_PATH = Path(__file__).resolve().parents[1] / 'config' / 'plc_coordinates.json'
TRUCK_FIELDS = ('length', 'width', 'height', 'angle', 'front_length', 'front_width',
                'front_height', 'rear_length', 'rear_width', 'rear_height') + tuple(
                    f'p{point}_{axis}' for point in range(1, 9) for axis in ('x', 'y', 'z'))


def default_config() -> dict:
    return {
        'ip': '10.11.207.100', 'port': 102, 'rack': 0, 'slot': 1,
        'interval_ms': 200,
        'axes': {name: {'enabled': True, 'db': None, 'offset': None, 'type': 'REAL'}
                 for name in AXES},
    }


def default_compensation_config() -> dict:
    return {'enabled': False, 'db': 102,
            'applied': dict.fromkeys(COMPENSATION_KEYS), 'input': dict.fromkeys(COMPENSATION_KEYS)}


def _integer(value, low: int, high: int, label: str) -> None:
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{label} 必须是 {low}～{high} 的整数')


def validate_config(config: dict, *, require_addresses: bool = False) -> None:
    if not isinstance(config, dict):
        raise ValueError('配置必须是 JSON 对象')
    if not isinstance(config.get('ip'), str):
        raise ValueError('PLC IP 必须是 IPv4 地址字符串')
    try:
        ipaddress.IPv4Address(config.get('ip', ''))
    except (ValueError, TypeError, ipaddress.AddressValueError) as exc:
        raise ValueError('PLC IP 必须是有效的 IPv4 地址') from exc
    for key, low, high in [('port', 1, 65535), ('rack', 0, 7), ('slot', 0, 31),
                           ('interval_ms', 100, 60000)]:
        _integer(config.get(key), low, high, key)
    axes = config.get('axes')
    if not isinstance(axes, dict) or set(axes) != set(AXES):
        raise ValueError('配置必须包含 X/Y/Z/R 四个轴')
    enabled = 0
    for name in AXES:
        item = axes[name]
        if not isinstance(item, dict) or type(item.get('enabled')) is not bool:
            raise ValueError(f'{name} 轴启用状态无效')
        if item.get('type') not in ('REAL', 'LREAL', 'DINT'):
            raise ValueError(f'{name} 类型只能是 REAL、LREAL 或 DINT')
        enabled += int(item['enabled'])
        for key, low, high, label in [('db', 1, 65535, 'DB 编号'),
                                       ('offset', 0, 2097144, '字节偏移')]:
            if key not in item:
                raise ValueError(f'{name} 轴配置缺少{label}字段；未知地址应填 null')
            if item.get(key) is None:
                if item['enabled'] and require_addresses:
                    raise ValueError(f'{name} 轴待配置：请填写{label}')
            else:
                _integer(item[key], low, high, f'{name} {label}')
    if require_addresses and not enabled:
        raise ValueError('请至少启用一个坐标轴')
    truck = config.get('truck', {'enabled': False})
    if not isinstance(truck, dict) or type(truck.get('enabled')) is not bool:
        raise ValueError('车厢读取配置无效')
    if truck['enabled']:
        _integer(truck.get('db'), 1, 65535, '车厢 DB 编号')
        _integer(truck.get('offset'), 0, 2096996, '车厢起始字节偏移')
    compensation = config.get('compensation', default_compensation_config())
    if not isinstance(compensation, dict) or type(compensation.get('enabled')) is not bool:
        raise ValueError('补偿读取配置无效')
    _integer(compensation.get('db'), 1, 65535, '补偿 DB 编号')
    for group in ('applied', 'input'):
        fields = compensation.get(group)
        if not isinstance(fields, dict) or set(fields) - set(COMPENSATION_KEYS):
            raise ValueError(f'补偿 {group} 字段配置无效')
        for key, offset in fields.items():
            if offset is not None:
                _integer(offset, 0, 2097144, f'{group}.{key} 字节偏移')


def load_config(path: Path = CONFIG_PATH) -> dict:
    path = Path(path)
    if not path.exists():
        return default_config()
    try:
        config = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise ValueError(f'无法读取配置 {path.name}：{exc}') from exc
    validate_config(config)
    return config


def save_config(path: Path, config: dict) -> None:
    validate_config(config)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(config, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def create_client():
    try:
        from s7 import Client
    except ImportError as exc:
        raise RuntimeError('缺少 python-snap7，请运行“安装环境.bat”安装项目依赖') from exc
    return Client(auto_reconnect=False)


class CoordinateReader:
    def __init__(self, config: dict, client):
        validate_config(config, require_addresses=True)
        self.config = config
        self.client = client

    def read_once(self, cancelled: Callable[[], bool] = lambda: False) -> dict:
        values = {}
        for name in AXES:
            item = self.config['axes'][name]
            if not item['enabled']:
                continue
            if cancelled():
                raise InterruptedError('读取已停止')
            size, format_code = {'REAL': (4, '>f'), 'LREAL': (8, '>d'), 'DINT': (4, '>i')}[item['type']]
            data = self.client.db_read(item['db'], item['offset'], size)
            if data is None or len(data) != size:
                raise ValueError(f'{name} DB{item["db"]} 偏移 {item["offset"]} 返回长度不足 {size} 字节')
            value = struct.unpack(format_code, data)[0]
            if not math.isfinite(value):
                raise ValueError(f'{name} 返回 NaN/无穷大，请检查数据类型和偏移')
            values[name] = value
        truck = self.config.get('truck', {})
        if truck.get('enabled'):
            if cancelled():
                raise InterruptedError('读取已停止')
            data = self.client.db_read(truck['db'], truck['offset'], 156)
            if data is None or len(data) != 156:
                raise ValueError(f'车厢 DB{truck["db"]} 偏移 {truck["offset"]} 读取长度不足 156 字节')
            values['truck'] = dict(zip(TRUCK_FIELDS, struct.unpack('>34i', data[:136])))
            values['truck']['dunnage'], values['truck']['front_baffle'] = struct.unpack('>2h', data[152:156])
        compensation = self.config.get('compensation', {})
        if compensation.get('enabled'):
            groups = {'applied': {}, 'input': {}}
            for group, fields in groups.items():
                for key, offset in compensation[group].items():
                    if offset is None:
                        continue
                    if cancelled():
                        raise InterruptedError('读取已停止')
                    data = self.client.db_read(compensation['db'], offset, 4)
                    if data is None or len(data) != 4:
                        raise ValueError(f'补偿 {group}.{key} 返回长度不足 4 字节')
                    fields[key] = struct.unpack('>i', data)[0]
            values['compensation'] = groups
        return values
