# -*- mode: python ; coding: utf-8 -*-
"""Build the sim-control Policy Node executable."""

from pathlib import Path

from PyInstaller.utils.hooks import collect_all


PROJECT_DIR = Path(SPEC).resolve().parent.parent
SRC_DIR = PROJECT_DIR / "src"


def collect_runtime():
    datas = []
    binaries = []
    hiddenimports = ["pyarrow", "yaml"]

    for package in (
        "dora",
        "forge_tool",
        "forge_msgs",
    ):
        package_datas, package_binaries, package_hiddenimports = collect_all(
            package,
            include_py_files=True,
        )
        datas.extend(package_datas)
        binaries.extend(package_binaries)
        hiddenimports.extend(package_hiddenimports)

    return (
        list(dict.fromkeys(datas)),
        list(dict.fromkeys(binaries)),
        list(dict.fromkeys(hiddenimports)),
    )


DATAS, BINARIES, HIDDENIMPORTS = collect_runtime()


def analysis(entry, extra_paths=()):
    return Analysis(
        [str(entry)],
        pathex=[str(SRC_DIR), *(str(path) for path in extra_paths)],
        binaries=BINARIES,
        datas=DATAS,
        hiddenimports=HIDDENIMPORTS,
        hookspath=[],
        hooksconfig={},
        runtime_hooks=[],
        excludes=[],
        noarchive=False,
        optimize=0,
    )


policy = analysis(PROJECT_DIR / "scripts" / "sim_control_policy_entry.py")
policy_pyz = PYZ(policy.pure)
policy_exe = EXE(
    policy_pyz,
    policy.scripts,
    policy.binaries,
    policy.datas,
    [],
    name="sim_control_policy",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=True,
)
