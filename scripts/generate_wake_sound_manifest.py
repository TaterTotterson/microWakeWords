#!/usr/bin/env python3
from __future__ import annotations

import json
import mimetypes
import re
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_PATH = REPO_ROOT / 'wake_sound_manifest.json'
REPO_OWNER = 'TaterTotterson'
REPO_NAME = 'microWakeWords'
REPO_REF = 'main'

SOURCE_SPECS = (
    ('wakeSounds', 'wakeSounds'),
)

AUDIO_EXTENSIONS = {
    '.flac',
}

MEDIA_TYPES = {
    '.flac': 'audio/flac',
}


def slug_to_label(value: str) -> str:
    token = str(value or '').strip()
    if not token:
        return 'Wake Sound'
    parts = [part for part in re.split(r'[_\-\.\s]+', token) if part]
    if not parts:
        return token
    return ' '.join(part.capitalize() for part in parts)


def raw_url(path: Path) -> str:
    rel = path.relative_to(REPO_ROOT).as_posix()
    return f'https://raw.githubusercontent.com/{REPO_OWNER}/{REPO_NAME}/{REPO_REF}/{rel}'


def media_type_for(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix in MEDIA_TYPES:
        return MEDIA_TYPES[suffix]
    guessed, _encoding = mimetypes.guess_type(path.name)
    return str(guessed or 'application/octet-stream')


def build_entries() -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for source_key, source_label in SOURCE_SPECS:
        source_dir = REPO_ROOT / source_key
        if not source_dir.is_dir():
            continue
        for audio_path in sorted(source_dir.iterdir()):
            if not audio_path.is_file():
                continue
            if audio_path.name.startswith('.'):
                continue
            suffix = audio_path.suffix.lower()
            if suffix not in AUDIO_EXTENSIONS:
                continue
            slug = audio_path.stem.strip() or audio_path.name
            entry: dict[str, Any] = {
                'id': f'{source_key}:{slug}',
                'source': source_key,
                'source_label': source_label,
                'slug': slug,
                'name': slug,
                'label': slug_to_label(slug),
                'path': audio_path.relative_to(REPO_ROOT).as_posix(),
                'url': raw_url(audio_path),
                'download_url': raw_url(audio_path),
                'extension': suffix.lstrip('.'),
                'media_type': media_type_for(audio_path),
                'size_bytes': audio_path.stat().st_size,
            }
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
