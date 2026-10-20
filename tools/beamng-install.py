#!/usr/bin/env python3
"""Installs engine-sim's BeamNG plugin and mod, or puts the game back how it was.

  install    back up BinLinux/libBeamNGBlendLayerDSP.so as .orig, copy ours in, copy the mod zip
  uninstall  restore the original plugin and remove the mod zip
  status     show what's installed

Steam's "Verify integrity of game files" and game updates restore the stock plugin; run
install again afterwards. The game must not be running.

Normally run through the build:  ninja -C build beamng-install
"""

import argparse
import filecmp
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

PLUGIN_NAME = "libBeamNGBlendLayerDSP.so"
MOD_NAME = "engineSimBridge.zip"
OUR_PLUGIN_MARKER = b"engine-sim BeamNG plugin"
HOME = Path.home()


def steam_libraries():
    """Library folders listed in Steam's libraryfolders.vdf (native and Flatpak Steam)."""
    roots = [
        HOME / ".local/share/Steam",
        HOME / ".steam/steam",
        HOME / ".var/app/com.valvesoftware.Steam/.local/share/Steam",
    ]
    libraries = []
    for root in roots:
        vdf = root / "steamapps/libraryfolders.vdf"
        if not vdf.exists():
            continue
        libraries.append(root)
        for match in re.finditer(r'"path"\s+"([^"]+)"', vdf.read_text(errors="replace")):
            libraries.append(Path(match.group(1)))
    seen, unique = set(), []
    for library in libraries:
        key = library.resolve() if library.exists() else library
        if key not in seen:
            seen.add(key)
            unique.append(library)
    return unique


def find_game(override):
    if override:
        return Path(override)
    for library in steam_libraries():
        game = library / "steamapps/common/BeamNG.drive"
        if (game / "BinLinux" / PLUGIN_NAME).exists() or (game / "BinLinux").is_dir():
            return game
    return None


def find_user_folder(override):
    if override:
        return Path(override)
    current = HOME / ".local/share/BeamNG/BeamNG.drive/current"
    return current if current.is_dir() else None


def is_ours(path):
    return path.exists() and OUR_PLUGIN_MARKER in path.read_bytes()


def game_running():
    result = subprocess.run(["pgrep", "-f", "BeamNG.drive.x64"], capture_output=True)
    return result.returncode == 0


def copy_atomically(source, destination):
    temporary = destination.with_name(destination.name + ".engine-sim-tmp")
    shutil.copy2(source, temporary)
    os.replace(temporary, destination)


def describe_plugin(plugin):
    if not plugin.exists():
        return "missing"
    return "engine-sim" if is_ours(plugin) else "stock"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=["install", "uninstall", "status"])
    parser.add_argument("--plugin", help="built plugin (build/beamng/fmod-plugin/%s)" % PLUGIN_NAME)
    parser.add_argument("--mod", help="built mod zip (build/beamng/%s)" % MOD_NAME)
    parser.add_argument("--game", help="BeamNG.drive install folder (default: found through Steam)")
    parser.add_argument("--user", help="BeamNG user folder (default: ~/.local/share/BeamNG/BeamNG.drive/current)")
    parser.add_argument("--force", action="store_true", help="skip the check that the game isn't running")
    args = parser.parse_args()

    game = find_game(args.game)
    user = find_user_folder(args.user)
    if game is None:
        sys.exit("BeamNG.drive not found in any Steam library; pass --game")

    target = game / "BinLinux" / PLUGIN_NAME
    backup = target.with_name(PLUGIN_NAME + ".orig")
    mod_target = user / "mods" / MOD_NAME if user else None

    if args.action == "status":
        print("game:         %s" % game)
        print("plugin:       %s (%s)" % (target, describe_plugin(target)))
        print("backup:       %s" % (backup if backup.exists() else "none"))
        print("user folder:  %s" % (user or "not found (pass --user)"))
        if mod_target:
            print("mod:          %s" % ("installed" if mod_target.exists() else "not installed"))
        print("game running: %s" % ("yes" if game_running() else "no"))
        return

    if game_running() and not args.force:
        sys.exit("BeamNG.drive is running; close it first")

    if args.action == "install":
        if not args.plugin or not args.mod:
            sys.exit("install needs --plugin and --mod")
        plugin, mod = Path(args.plugin), Path(args.mod)
        if not is_ours(plugin):
            sys.exit("%s doesn't look like the engine-sim plugin" % plugin)
        if user is None:
            sys.exit("BeamNG user folder not found; start the game once, or pass --user")

        # Keep the newest stock plugin: after a game update the file is stock again
        if target.exists() and not is_ours(target):
            if not backup.exists() or not filecmp.cmp(target, backup, shallow=False):
                shutil.copy2(target, backup)
                print("backed up the stock plugin to %s" % backup)
        elif not backup.exists():
            print("warning: no stock plugin to back up; uninstall won't be able to restore it")

        copy_atomically(plugin, target)
        print("installed the plugin to %s" % target)

        mod_target.parent.mkdir(parents=True, exist_ok=True)
        copy_atomically(mod, mod_target)
        print("installed the mod to %s" % mod_target)
        print("done: start engine-sim (GUI or headless), then BeamNG.drive")

    elif args.action == "uninstall":
        if backup.exists():
            os.replace(backup, target)
            print("restored the stock plugin")
        elif is_ours(target):
            print("warning: no backup found; use Steam's \"Verify integrity of game files\" to restore the stock plugin")
        else:
            print("the stock plugin is already in place")

        if mod_target and mod_target.exists():
            mod_target.unlink()
            print("removed %s" % mod_target)


if __name__ == "__main__":
    main()
