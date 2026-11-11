# PlatformIO pre-script: defines FW_GIT_REV (git describe) for the host link HELLO.
import subprocess

Import("env")  # noqa: F821  (provided by PlatformIO / SCons)

try:
    rev = subprocess.check_output(
        ["git", "describe", "--always", "--dirty"], cwd=env["PROJECT_DIR"], stderr=subprocess.DEVNULL  # noqa: F821
    ).decode().strip()
except Exception:
    rev = "unknown"

env.Append(CPPDEFINES=[("FW_GIT_REV", env.StringifyMacro(rev))])  # noqa: F821
