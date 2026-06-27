#!/usr/bin/env python3
"""Collect ESPHome build outputs into the prebuilt firmware manifest layout."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import zipfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_PREBUILT_PREFIX = "prebuilt_firmware"
VERSION_RE = re.compile(r"^[0-9]+[.][0-9]+[.][0-9]+(?:[-+][0-9A-Za-z.-]+)?$")


@dataclass(frozen=True)
class FirmwareDevice:
    key: str
    name: str
    config: str
    build_name: str
    artifact_basename: str


DEVICES: tuple[FirmwareDevice, ...] = (
    FirmwareDevice("satellite1", "Satellite1 / Sat1", "satellite1-TaterTimer.yaml", "tatersat1", "tater-satellite1"),
    FirmwareDevice("voicepe", "Voice PE", "voicePE-TaterTimer.yaml", "tatervpe", "tater-voicepe"),
    FirmwareDevice(
        "respeaker_lite",
        "Respeaker Lite",
        "respeakerLite-TaterTimer.yaml",
        "tater-respeaker-lite",
        "tater-respeaker-lite",
    ),
    FirmwareDevice("koala", "Koala", "koala-TaterTimer.yaml", "tater-koala", "tater-koala"),
    FirmwareDevice(
        "respeaker_xvf3800",
        "Respeaker XVF3800",
        "respeakerXVF3800-TaterTimer.yaml",
        "tater-respeaker-xvf3800",
        "tater-respeaker-xvf3800",
    ),
    FirmwareDevice(
        "s3box_display",
        "Tater ESP32-S3-BOX-3 Display",
        "esp32-s3-box-3.yaml",
        "taters3box",
        "tater-s3box-display",
    ),
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def copy_artifact(
    *,
    source: Path,
    destination: Path,
    repo_path: str,
    kind: str,
) -> dict[str, Any]:
    if not source.exists():
        raise FileNotFoundError(f"Missing {kind} firmware artifact: {source}")

    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return {
        "kind": kind,
        "path": repo_path,
        "size_bytes": destination.stat().st_size,
        "sha256": sha256_file(destination),
    }


def build_manifest(version: str, output_root: Path, generated_date: str) -> dict[str, Any]:
    devices: list[dict[str, Any]] = []
    for device in DEVICES:
        device_dir = output_root / version / device.key
        build_dir = Path(".esphome") / "build" / device.build_name / ".pioenvs" / device.build_name

        ota_name = f"{device.artifact_basename}-{version}.ota.bin"
        factory_name = f"{device.artifact_basename}-{version}.factory.bin"
        ota_repo_path = f"{REPO_PREBUILT_PREFIX}/{version}/{device.key}/{ota_name}"
        factory_repo_path = f"{REPO_PREBUILT_PREFIX}/{version}/{device.key}/{factory_name}"

        artifacts = {
            "ota": copy_artifact(
                source=build_dir / "firmware.ota.bin",
                destination=device_dir / ota_name,
                repo_path=ota_repo_path,
                kind="ota",
            ),
            "factory": copy_artifact(
                source=build_dir / "firmware.factory.bin",
                destination=device_dir / factory_name,
                repo_path=factory_repo_path,
                kind="factory",
            ),
        }
        devices.append(
            {
                "key": device.key,
                "name": device.name,
                "config": device.config,
                "artifacts": artifacts,
            }
        )

    return {
        "schema_version": 1,
        "version": version,
        "generated_date": generated_date,
        "default_install_artifact": "ota",
        "notes": [
            "Use ota artifacts for ESPHome OTA updates from Tater app.",
            "Use factory artifacts for first USB/serial flash or recovery flashes at offset 0x0.",
        ],
        "devices": devices,
    }


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def write_checksums(path: Path, manifest: dict[str, Any], output_root: Path) -> None:
    lines: list[str] = []
    for device in manifest["devices"]:
        for artifact in device["artifacts"].values():
            artifact_path = output_root.parent / artifact["path"]
            lines.append(f"{artifact['sha256']}  {artifact_path.as_posix()}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_release_notes(path: Path, version: str, manifest: dict[str, Any]) -> None:
    device_lines = "\n".join(
        f"- {device['name']}: OTA and factory images" for device in manifest["devices"]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(
            [
                f"Automated Tater firmware build {version}.",
                "",
                "Includes:",
                device_lines,
                "",
                "Use OTA images for app-driven updates. Use factory images for first USB/serial flash or recovery.",
                "",
            ]
        ),
        encoding="utf-8",
    )


def write_zip(path: Path, output_root: Path, version: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        version_root = output_root / version
        for item in sorted(version_root.rglob("*")):
            if item.is_file():
                archive.write(item, Path(REPO_PREBUILT_PREFIX) / version / item.relative_to(version_root))
        latest_path = output_root / "latest.json"
        if latest_path.exists():
            archive.write(latest_path, Path(REPO_PREBUILT_PREFIX) / "latest.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", required=True, help="Firmware version, usually from tag vX.Y.Z")
    parser.add_argument("--output-root", default=REPO_PREBUILT_PREFIX, help="Directory to write prebuilt firmware into")
    parser.add_argument("--release-dir", default="dist/release", help="Directory for release helper assets")
    parser.add_argument("--generated-date", help="Manifest generated date, defaults to current UTC date")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    version = args.version.strip().removeprefix("v")
    if not VERSION_RE.fullmatch(version):
        raise SystemExit(f"Invalid version {args.version!r}; expected something like 3.0.12")

    output_root = Path(args.output_root)
    release_dir = Path(args.release_dir)
    generated_date = args.generated_date or datetime.now(timezone.utc).date().isoformat()

    manifest = build_manifest(version, output_root, generated_date)
    write_json(output_root / version / "manifest.json", manifest)
    write_json(
        output_root / "latest.json",
        {
            "version": version,
            "manifest": f"{REPO_PREBUILT_PREFIX}/{version}/manifest.json",
        },
    )
    write_checksums(release_dir / f"tater-firmware-{version}-checksums.txt", manifest, output_root)
    write_release_notes(release_dir / "release-notes.md", version, manifest)
    write_zip(release_dir / f"tater-firmware-{version}.zip", output_root, version)


if __name__ == "__main__":
    main()
