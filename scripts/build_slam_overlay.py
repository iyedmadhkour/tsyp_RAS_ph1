#!/usr/bin/env python3
"""Build the pinned Team 4 SLAM fix outside the project and system ROS."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import time
import urllib.request

REVISION = "ec8f7635dea317b531c419f798f87d90a336f32e"  # upstream 2.8.5
ARCHIVE_SHA256 = "c480942995404009cc833b32e10ca2538c1a322b1cda94cf2744c947271fe52a"
HEADER_SHA256 = "277fd3ddbb4c655408660e7bab8ff39f49875af709bd7a83ff3d029190d44052"
URL = f"https://codeload.github.com/SteveMacenski/slam_toolbox/tar.gz/{REVISION}"
ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / "patches/slam_toolbox-2.8.5-grid-extents.patch"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def source_manifest(source):
    return {str(p.relative_to(source)): digest(p)
            for p in sorted(source.rglob("*")) if p.is_file()}


def download(cache):
    archive = cache / f"slam_toolbox-{REVISION}.tar.gz"
    if archive.exists():
        return archive
    for attempt in range(3):
        # Failed downloads remain available for diagnosis; no existing file is erased.
        with tempfile.NamedTemporaryFile(dir=cache, prefix="download-", delete=False) as stream:
            try:
                with urllib.request.urlopen(URL, timeout=30) as response:
                    shutil.copyfileobj(response, stream)
                stream.flush()
                if digest(Path(stream.name)) != ARCHIVE_SHA256:
                    raise RuntimeError("Upstream archive checksum mismatch")
                os.link(stream.name, archive)  # Atomic publication; never overwrite.
                return archive
            except (OSError, RuntimeError) as exc:
                print(f"Download attempt {attempt + 1} failed: {exc}", flush=True)
                if attempt == 2:
                    raise
                time.sleep(2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspace", type=Path, help="Separate user-owned overlay directory")
    parser.add_argument("--archive", type=Path, help="Reuse an existing verified source archive")
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    ws = args.workspace.expanduser().resolve()
    home = Path.home().resolve()
    if (ws == home or not ws.is_relative_to(home) or ws.is_relative_to(ROOT)
            or ROOT.is_relative_to(ws)):
        parser.error("Use a dedicated directory under your Linux home, outside the checkout")
    if not 1 <= args.jobs <= 8:
        parser.error("--jobs must be between 1 and 8")
    if os.environ.get("ROS_DISTRO") != "jazzy":
        parser.error("Source /opt/ros/jazzy/setup.bash in this shell first")
    for tool in ("colcon", "cmake", "g++", "patch"):
        if not shutil.which(tool):
            parser.error(f"Required existing tool missing: {tool}; nothing will be installed")
    marker = ws / "livingmap-source-manifest.json"
    source = ws / "src/slam_toolbox"
    identity = {"revision": REVISION, "archive_sha256": ARCHIVE_SHA256,
                "patch_sha256": digest(PATCH)}
    if marker.exists():
        saved = json.loads(marker.read_text())
        if any(saved[k] != value for k, value in identity.items()):
            parser.error("Workspace belongs to a different patch/revision; use a new directory")
        if saved["source_files"] != source_manifest(source):
            parser.error("Prepared source has changed; preserve it and use a new directory")
    else:
        if ws.exists() and any(ws.iterdir()):
            parser.error("Unrecognized nonempty workspace; choose a new directory")
        ws.mkdir(parents=True, exist_ok=True)
        cache = ws / "cache"
        cache.mkdir()
        archive = args.archive.expanduser().resolve() if args.archive else download(cache)
        if digest(archive) != ARCHIVE_SHA256:
            parser.error("Archive checksum mismatch; source was not extracted")
        with tarfile.open(archive) as tar:
            tar.extractall(cache, filter="data")
        extracted = cache / f"slam_toolbox-{REVISION}"
        if digest(extracted / "lib/karto_sdk/include/karto_sdk/Karto.h") != HEADER_SHA256:
            parser.error("Unexpected upstream header; source was not patched")
        source.parent.mkdir()
        extracted.rename(source)
        subprocess.run(["patch", "--batch", "--fuzz=0", "-p1", "-i", str(PATCH)],
                       cwd=source, check=True)
        marker.write_text(json.dumps({**identity, "source_files": source_manifest(source)}, indent=2))
    env = os.environ.copy()
    env["CMAKE_BUILD_PARALLEL_LEVEL"] = str(args.jobs)
    env["MAKEFLAGS"] = f"-j{args.jobs}"
    log = ws / "log" / time.strftime("build-%Y%m%d-%H%M%S")
    subprocess.run([
        "colcon", "--log-base", str(log), "build", "--base-paths", str(ws / "src"),
        "--build-base", str(ws / "build"), "--install-base", str(ws / "install"),
        "--merge-install", "--packages-select", "slam_toolbox",
        "--allow-overriding", "slam_toolbox", "--executor", "sequential",
        "--cmake-args", "-DBUILD_TESTING=OFF"], cwd=ws, env=env, check=True)
    subprocess.run([str(ROOT / "scripts/test_slam_grid_extents.sh"),
                    str(ws / "install")], env=env, check=True)
    print(f"Validated overlay: {ws / 'install'}")
    print("Source the project environment first, then this overlay's local_setup.bash.")
    print("Nothing in /opt/ros/jazzy or your shell startup files was changed.")


if __name__ == "__main__":
    main()
