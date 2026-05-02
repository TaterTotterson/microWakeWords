#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIR = REPO_ROOT / 'wakeSounds'
SOURCE_EXTENSIONS = {
    '.flac',
    '.mp3',
    '.ogg',
    '.wav',
}
TARGET_EXTENSION = '.flac'
TARGET_SAMPLE_RATE = '48000'
TARGET_CHANNELS = 1
TARGET_SAMPLE_FMT = 's16'


def audio_files() -> list[Path]:
    if not SOURCE_DIR.is_dir():
        return []
    return sorted(
        path
        for path in SOURCE_DIR.iterdir()
        if path.is_file()
        and not path.name.startswith('.')
        and path.suffix.lower() in SOURCE_EXTENSIONS
    )


def probe(path: Path) -> dict[str, str]:
    result = subprocess.run(
        [
            'ffprobe',
            '-v',
            'error',
            '-select_streams',
            'a:0',
            '-print_format',
            'json',
            '-show_entries',
            'stream=codec_name,sample_rate,channels,sample_fmt',
            str(path),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    payload = json.loads(result.stdout or '{}')
    streams = payload.get('streams') or []
    if not streams:
        raise RuntimeError(f'No audio stream found in {path}')
    return {str(key): str(value) for key, value in streams[0].items()}


def is_target_format(path: Path) -> bool:
    if path.suffix.lower() != TARGET_EXTENSION:
        return False
    stream = probe(path)
    return (
        stream.get('codec_name') == 'flac'
        and stream.get('sample_rate') == TARGET_SAMPLE_RATE
        and stream.get('channels') == str(TARGET_CHANNELS)
        and stream.get('sample_fmt') == TARGET_SAMPLE_FMT
    )


def target_path(path: Path, duplicate_stems: set[str]) -> Path:
    suffix = path.suffix.lower()
    if suffix == TARGET_EXTENSION:
        return path
    if path.stem in duplicate_stems:
        return path.with_name(f'{path.stem}-{suffix.lstrip(".")}{TARGET_EXTENSION}')
    return path.with_suffix(TARGET_EXTENSION)


def convert(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        dir=target.parent,
        prefix=f'.{target.stem}.',
        suffix=TARGET_EXTENSION,
        delete=False,
    ) as tmp:
        tmp_path = Path(tmp.name)
    try:
        subprocess.run(
            [
                'ffmpeg',
                '-hide_banner',
                '-loglevel',
                'error',
                '-y',
                '-i',
                str(source),
                '-map',
                '0:a:0',
                '-vn',
                '-ac',
                str(TARGET_CHANNELS),
                '-ar',
                TARGET_SAMPLE_RATE,
                '-sample_fmt',
                TARGET_SAMPLE_FMT,
                '-compression_level',
                '5',
                str(tmp_path),
            ],
            check=True,
        )
        tmp_path.replace(target)
    finally:
        if tmp_path.exists():
            tmp_path.unlink()


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Normalize wakeSounds audio to 48 kHz mono 16-bit FLAC for satellite firmware.'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print what would be converted without writing files.',
    )
    parser.add_argument(
        '--force',
        action='store_true',
        help='Regenerate converted FLAC files even when an up-to-date target already exists.',
    )
    args = parser.parse_args()

    sources = audio_files()
    duplicate_stems = {
        stem
        for stem, count in Counter(
            path.stem for path in sources if path.suffix.lower() != TARGET_EXTENSION
        ).items()
        if count > 1
    }

    converted = 0
    skipped = 0
    for source in sources:
        target = target_path(source, duplicate_stems)
        if source == target and is_target_format(source):
            skipped += 1
            print(f'skip      {source.relative_to(REPO_ROOT)}')
            continue
        if (
            not args.force
            and source != target
            and target.exists()
            and target.stat().st_mtime >= source.stat().st_mtime
            and is_target_format(target)
        ):
            skipped += 1
            print(
                f'skip      {source.relative_to(REPO_ROOT)} -> '
                f'{target.relative_to(REPO_ROOT)}'
            )
            continue
        print(f'convert   {source.relative_to(REPO_ROOT)} -> {target.relative_to(REPO_ROOT)}')
        if not args.dry_run:
            convert(source, target)
        converted += 1

    mode = 'Would convert' if args.dry_run else 'Converted'
    print(f'{mode} {converted} wake sound(s); skipped {skipped} up-to-date file(s).')


if __name__ == '__main__':
    main()
