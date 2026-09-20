#!/usr/bin/env python3
"""Prepare an idle serial port, then open the NoMachine page; no serial data IO."""

import os
from pathlib import Path
import subprocess
import time
import shutil
from prepare_browser_serial import prepare_serial


def main():
    env = os.environ.copy()
    if not env.get("DISPLAY"):
        sessions = []
        for proc in Path("/proc").iterdir():
            if not proc.name.isdigit():
                continue
            try:
                if proc.stat().st_uid != os.getuid() or (proc / "comm").read_text().strip() != "gnome-shell":
                    continue
                entries = (proc / "environ").read_bytes().split(b"\0")
                selected = {}
                for entry in entries:
                    key, sep, value = entry.partition(b"=")
                    if sep and key in (b"DISPLAY", b"XAUTHORITY", b"DBUS_SESSION_BUS_ADDRESS",
                                       b"XDG_RUNTIME_DIR", b"WAYLAND_DISPLAY"):
                        selected[key.decode()] = os.fsdecode(value)
                if selected.get("DISPLAY"):
                    sessions.append(selected)
            except (OSError, ValueError):
                continue
        if len(sessions) != 1:
            raise RuntimeError("No unique graphical desktop found; open Big Yaw PID Tuner from the desktop")
        env.update(sessions[0])
    page = Path(__file__).resolve().with_name("big-yaw.html")
    if not page.is_file():
        raise RuntimeError("big-yaw.html is missing")
    try:
        print(prepare_serial())
    except (OSError, RuntimeError) as error:
        if shutil.which("zenity"):
            subprocess.run(["zenity", "--error", "--title=Big Yaw serial preflight",
                            "--text=" + str(error)], env=env, check=False)
        raise
    with page.with_name("big-yaw-browser.log").open("ab") as log:
        child = subprocess.Popen(
            ["/usr/bin/google-chrome", "--no-first-run", "--no-default-browser-check", "--app=" + page.as_uri()],
            env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
    time.sleep(1)
    if child.poll() not in (None, 0):
        raise RuntimeError("Chrome did not start; see big-yaw-browser.log")
    print("Opened big-yaw.html on DISPLAY=" + env["DISPLAY"] + "; serial remains disconnected.")


if __name__ == "__main__":
    main()
