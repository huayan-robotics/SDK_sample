#!/usr/bin/env python3
"""Verify and configure the bundled SDK archive for a sample project."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
import tempfile
import zipfile
from pathlib import Path, PurePosixPath


ROOT = Path(__file__).resolve().parent
PROJECTS_FILE = ROOT / "projects.json"
CHECKSUM_SUFFIX = ".sha256"
LIBRARY_SUFFIXES = (".dll", ".lib", ".a", ".so")


class ConfigureError(RuntimeError):
    pass


def load_projects() -> dict:
    try:
        data = json.loads(PROJECTS_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigureError(f"Cannot read projects.json: {exc}") from exc
    projects = data.get("projects")
    if not isinstance(projects, dict):
        raise ConfigureError("projects.json does not contain a projects object")
    return projects


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_checksum(path: Path, archive_name: str) -> str:
    try:
        text = path.read_text(encoding="utf-8").strip()
    except OSError as exc:
        raise ConfigureError(f"Cannot read SDK checksum file {path}: {exc}") from exc
    match = re.fullmatch(r"([0-9a-fA-F]{64})(?:\s+\*?(.+))?", text)
    if not match:
        raise ConfigureError(f"Invalid SDK checksum file: {path}")
    recorded_name = match.group(2)
    if recorded_name and recorded_name != archive_name:
        raise ConfigureError(
            f"SDK checksum filename mismatch: {recorded_name} != {archive_name}"
        )
    return match.group(1).lower()


def find_sdk_archive(language: str) -> tuple[Path, str]:
    language_dir = ROOT / "sdk" / language
    if not language_dir.is_dir():
        raise ConfigureError(f"SDK language directory does not exist: {language_dir}")
    archives = sorted(language_dir.glob("*.zip"))
    if len(archives) != 1:
        raise ConfigureError(
            f"Expected exactly one SDK ZIP under {language_dir}, found {len(archives)}"
        )
    archive = archives[0]
    checksum_path = archive.with_name(archive.name + CHECKSUM_SUFFIX)
    if not checksum_path.is_file():
        raise ConfigureError(f"SDK checksum file does not exist: {checksum_path}")
    expected = read_checksum(checksum_path, archive.name)
    actual = sha256_of(archive)
    if actual != expected:
        raise ConfigureError(
            f"SDK SHA-256 mismatch for {archive.name}\n"
            f"  expected: {expected}\n  actual:   {actual}"
        )
    return archive, actual


def validated_members(archive: zipfile.ZipFile) -> list[zipfile.ZipInfo]:
    members: list[zipfile.ZipInfo] = []
    names: set[str] = set()
    for info in archive.infolist():
        if info.is_dir():
            continue
        if "\\" in info.filename:
            raise ConfigureError(f"Unsafe SDK ZIP path: {info.filename}")
        path = PurePosixPath(info.filename)
        if path.is_absolute() or ".." in path.parts:
            raise ConfigureError(f"Unsafe SDK ZIP path: {info.filename}")
        if info.flag_bits & 1:
            raise ConfigureError(f"Encrypted SDK ZIP entry is not supported: {info.filename}")
        if info.filename in names:
            raise ConfigureError(f"Duplicate SDK ZIP entry: {info.filename}")
        names.add(info.filename)
        members.append(info)
    try:
        bad_member = archive.testzip()
    except (NotImplementedError, RuntimeError, zipfile.BadZipFile) as exc:
        raise ConfigureError(f"Cannot verify SDK ZIP: {exc}") from exc
    if bad_member is not None:
        raise ConfigureError(f"SDK ZIP CRC check failed: {bad_member}")
    return members


def select_one(members: list[zipfile.ZipInfo], basename: str) -> zipfile.ZipInfo:
    matches = [info for info in members if PurePosixPath(info.filename).name == basename]
    if len(matches) != 1:
        raise ConfigureError(
            f"Expected exactly one {basename} in SDK ZIP, found {len(matches)}"
        )
    return matches[0]


def write_member(
    archive: zipfile.ZipFile, member: zipfile.ZipInfo, target: Path
) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    with archive.open(member) as source, target.open("wb") as destination:
        shutil.copyfileobj(source, destination)


def stage_cpp(
    archive: zipfile.ZipFile, members: list[zipfile.ZipInfo], staging: Path
) -> list[str]:
    header = select_one(members, "HR_Pro.h")
    write_member(archive, header, staging / "samples" / "HR_Pro.h")
    libraries = [
        info for info in members
        if PurePosixPath(info.filename).name.lower().endswith(LIBRARY_SUFFIXES)
    ]
    if not libraries:
        raise ConfigureError("C++ SDK ZIP does not contain any library files")
    basenames = [PurePosixPath(info.filename).name for info in libraries]
    if len(basenames) != len(set(basenames)):
        raise ConfigureError("C++ SDK ZIP contains duplicate library filenames")
    for info in libraries:
        write_member(archive, info, staging / "lib" / PurePosixPath(info.filename).name)
    return ["samples/HR_Pro.h", "lib"]


def stage_csharp(
    archive: zipfile.ZipFile, members: list[zipfile.ZipInfo], staging: Path
) -> list[str]:
    libraries = [
        info for info in members
        if PurePosixPath(info.filename).name.lower().endswith(".dll")
    ]
    names = [PurePosixPath(info.filename).name for info in libraries]
    if "RobotLibrarys.dll" not in names:
        raise ConfigureError("C# SDK ZIP does not contain RobotLibrarys.dll")
    if len(names) != len(set(names)):
        raise ConfigureError("C# SDK ZIP contains duplicate DLL filenames")
    for info in libraries:
        write_member(archive, info, staging / "lib" / PurePosixPath(info.filename).name)
    return ["lib"]


def stage_python(
    archive: zipfile.ZipFile, members: list[zipfile.ZipInfo], staging: Path
) -> list[str]:
    module = select_one(members, "CPS.py")
    write_member(archive, module, staging / "samples" / "CPS.py")
    return ["samples/CPS.py"]


def remove_path(path: Path) -> None:
    if path.is_dir():
        shutil.rmtree(path)
    elif path.exists():
        path.unlink()


def install_staged(project_dir: Path, staging: Path, relatives: list[str]) -> None:
    applied: list[tuple[Path, Path | None]] = []
    try:
        for relative in relatives:
            source = staging / relative
            destination = project_dir / relative
            if not source.exists():
                raise ConfigureError(f"Staged SDK output is missing: {relative}")
            destination.parent.mkdir(parents=True, exist_ok=True)
            backup = destination.with_name(destination.name + ".sdk-old")
            if backup.exists():
                remove_path(backup)
            saved: Path | None = None
            if destination.exists():
                destination.rename(backup)
                saved = backup
            applied.append((destination, saved))
            source.rename(destination)
    except (OSError, ConfigureError) as exc:
        for destination, backup in reversed(applied):
            try:
                if destination.exists():
                    remove_path(destination)
                if backup is not None and backup.exists():
                    backup.rename(destination)
            except OSError:
                pass
        if isinstance(exc, ConfigureError):
            raise
        raise ConfigureError(f"Cannot install SDK files: {exc}") from exc
    for _destination, backup in applied:
        if backup is not None and backup.exists():
            remove_path(backup)


def configure_project(project_id: str, project: dict) -> None:
    project_dir = ROOT / project_id
    if not project_dir.is_dir():
        raise ConfigureError(f"Project directory does not exist: {project_dir}")
    language = project["language"]
    archive_path, digest = find_sdk_archive(language)
    staging = Path(tempfile.mkdtemp(prefix=".sdk-config-", dir=project_dir))
    relatives: list[str] = []
    try:
        try:
            with zipfile.ZipFile(archive_path) as archive:
                members = validated_members(archive)
                if language == "cpp":
                    relatives = stage_cpp(archive, members, staging)
                elif language == "csharp":
                    relatives = stage_csharp(archive, members, staging)
                elif language == "python":
                    relatives = stage_python(archive, members, staging)
                else:
                    raise ConfigureError(f"Unsupported language: {language}")
        except (OSError, zipfile.BadZipFile) as exc:
            raise ConfigureError(f"Cannot read SDK ZIP {archive_path}: {exc}") from exc
        install_staged(project_dir, staging, relatives)
    finally:
        if staging.exists():
            shutil.rmtree(staging, ignore_errors=True)
    print(f"Verified {archive_path.name}: sha256 {digest}")
    print(f"Configured {project_id}: {len(relatives)} managed SDK target(s)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", help="Project id, for example s-series-cpp")
    parser.add_argument("--list", action="store_true", help="List available projects")
    parser.add_argument("--all", action="store_true", help="Configure every project")
    args = parser.parse_args()
    try:
        projects = load_projects()
        if args.list:
            for project_id in projects:
                item = projects[project_id]
                print(f"{project_id}: {item['model']} / {item['language']}")
            return 0
        if args.all:
            for project_id, project in projects.items():
                configure_project(project_id, project)
            return 0
        if not args.project:
            parser.error("one of --project, --list, or --all is required")
        if args.project not in projects:
            raise ConfigureError(f"Unknown project: {args.project}")
        configure_project(args.project, projects[args.project])
        return 0
    except (ConfigureError, OSError) as exc:
        print(f"Configuration failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
