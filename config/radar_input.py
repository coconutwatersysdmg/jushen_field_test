"""User-selected local PCD input, retained across offline configuration reloads."""
from __future__ import annotations

import json
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PCD_DIRECTORY = PROJECT_ROOT / 'pcd'
SELECTION_STATE_FILE = PROJECT_ROOT / 'workdir' / 'radar_input.json'


def load_selected_radar_pcd() -> str:
    try:
        value = json.loads(SELECTION_STATE_FILE.read_text(encoding='utf-8')).get('selected_radar_pcd', '')
        return str(value or '')
    except (OSError, ValueError, TypeError, AttributeError):
        return ''


def save_selected_radar_pcd(path: str) -> str:
    selected = Path(path).expanduser().resolve()
    if selected.suffix.lower() != '.pcd' or not selected.is_file():
        raise ValueError(f'请选择可读的本地 .pcd 文件：{selected}')
    SELECTION_STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    SELECTION_STATE_FILE.write_text(json.dumps({'selected_radar_pcd': str(selected)}, ensure_ascii=False, indent=2), encoding='utf-8')
    return str(selected)
