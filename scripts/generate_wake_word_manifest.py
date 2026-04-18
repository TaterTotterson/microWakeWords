#!/usr/bin/env python3
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_PATH = REPO_ROOT / 'wake_word_manifest.json'
REPO_OWNER = 'TaterTotterson'
REPO_NAME = 'microWakeWords'
REPO_REF = 'main'

SOURCE_SPECS = (
    ('microWakeWords', 'microWakeWords'),
    ('microWakeWordsV2', 'microWakeWordsV2'),
    ('microWakeWordsV3', 'microWakeWordsV3'),
)


def slug_to_label(value: str) -> str:
    token = str(value or '').strip()
    if not token:
        return 'Wake Word'
    parts = [part for part in re.split(r'[_\-\s]+', token) if part]
    if not parts:
        return token
    return ' '.join(part.capitalize() for part in parts)


def raw_url(path: Path) -> str:
    rel = path.relative_to(REPO_ROOT).as_posix()
    return f'https://raw.githubusercontent.com/{REPO_OWNER}/{REPO_NAME}/{REPO_REF}/{rel}'


def load_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def build_entries() -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for source_key, source_label in SOURCE_SPECS:
        source_dir = REPO_ROOT / source_key
        if not source_dir.is_dir():
            continue
        for json_path in sorted(source_dir.glob('*.json')):
            model_path = json_path.with_suffix('.tflite')
            if not model_path.is_file():
                continue
            payload = load_json(json_path)
            slug = str(payload.get('wake_word') or json_path.stem).strip() or json_path.stem
            entry: dict[str, Any] = {
                'id': f'{source_key}:{slug}',
                'source': source_key,
                'source_label': source_label,
                'slug': slug,
                'name': slug,
                'label': str(payload.get('display_name') or payload.get('title') or payload.get('label') or slug_to_label(slug)),
                'path': json_path.relative_to(REPO_ROOT).as_posix(),
                'model_path': model_path.relative_to(REPO_ROOT).as_posix(),
                'url': raw_url(json_path),
                'download_url': raw_url(json_path),
            }
            if payload.get('author'):
                entry['author'] = payload['author']
            languages = payload.get('trained_languages')
            if isinstance(languages, list) and languages:
                entry['trained_languages'] = [str(item) for item in languages if str(item).strip()]
            if payload.get('version') is not None:
                entry['version'] = payload.get('version')
            micro = payload.get('micro')
            if isinstance(micro, dict) and micro.get('minimum_esphome_version'):
                entry['minimum_esphome_version'] = str(micro.get('minimum_esphome_version'))
            entries.append(entry)
    entries.sort(key=lambda item: (str(item.get('source_label') or ''), str(item.get('label') or ''), str(item.get('slug') or '')))
    return entries


def build_manifest() -> dict[str, Any]:
    entries = build_entries()
    counts = {
        source_key: sum(1 for entry in entries if entry.get('source') == source_key)
        for source_key, _source_label in SOURCE_SPECS
        if any(entry.get('source') == source_key for entry in entries)
    }
    return {
        'repository': f'{REPO_OWNER}/{REPO_NAME}',
        'ref': REPO_REF,
        'sources': [
            {'key': source_key, 'label': source_label, 'count': counts.get(source_key, 0)}
            for source_key, source_label in SOURCE_SPECS
            if counts.get(source_key, 0)
        ],
        'count': len(entries),
        'entries': entries,
    }


def main() -> None:
    manifest = build_manifest()
    OUTPUT_PATH.write_text(json.dumps(manifest, indent=2, sort_keys=False) + chr(10), encoding='utf-8')
    print(f'Wrote {OUTPUT_PATH} with {manifest.get("count", 0)} entries.')


if __name__ == '__main__':
    main()
