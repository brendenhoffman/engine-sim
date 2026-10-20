#!/usr/bin/env python3
"""Stand-in for BeamNG + the FMOD plugin, for testing the bridge without the game.

Sends vehicle packets ("BES1" rpm, throttle, ignition, volume, pan) to engine-sim like the
mod's Lua does, prints the torque replies, and listens on the plugin's audio port to check
that engine sound arrives.

  tools/fake-beamng.py                       # rev between 1500 and 5000 rpm for 20 s
  tools/fake-beamng.py --rpm 3000 --throttle 0.6 --seconds 10
"""

import argparse
import math
import socket
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4570, help="engine-sim bridge port")
    parser.add_argument("--audio-port", type=int, default=4571, help="port the FMOD plugin would listen on")
    parser.add_argument("--rpm", type=float, help="hold this rpm instead of sweeping")
    parser.add_argument("--throttle", type=float, default=0.5)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=60.0, help="packets per second (BeamNG sends one per graphics frame)")
    args = parser.parse_args()

    link = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    link.connect((args.host, args.port))
    link.setblocking(False)

    audio = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    audio.bind((args.host, args.audio_port))
    audio.setblocking(False)

    start = time.monotonic()
    next_send = start
    next_report = start + 1.0
    torques, keeps, profiles = [], 0, 0
    sound_config = None
    audio_samples, audio_sq, audio_peak = 0, 0.0, 0.0

    while time.monotonic() - start < args.seconds:
        now = time.monotonic()
        t = now - start

        if now >= next_send:
            rpm = args.rpm if args.rpm is not None else 3250 + 1750 * math.sin(t * 0.6)
            link.send(b"BES1" + struct.pack("<5f", rpm, args.throttle, 1.0, 1.0, 0.0))
            next_send += 1.0 / args.rate

        while True:
            try:
                data = link.recv(2048)
            except BlockingIOError:
                break
            except ConnectionRefusedError:
                break  # engine-sim not listening (yet)
            tag = data[:4]
            if tag == b"BES1" and len(data) == 12:
                torques.append(struct.unpack("<2f", data[4:])[0])
            elif tag == b"BESN":
                keeps += 1
            elif tag == b"BESP":
                profiles += 1
            elif tag == b"BESC" and len(data) >= 20:
                sound_config = struct.unpack("<4f", data[4:20])

        while True:
            try:
                data = audio.recv(8192)
            except BlockingIOError:
                break
            if data[:4] != b"BESF":
                continue
            samples = struct.unpack("<%df" % ((len(data) - 4) // 4), data[4:])
            audio_samples += len(samples)
            audio_sq += sum(s * s for s in samples)
            audio_peak = max([audio_peak] + [abs(s) for s in samples])

        if now >= next_report:
            rpm = args.rpm if args.rpm is not None else 3250 + 1750 * math.sin(t * 0.6)
            torque = "%7.1f Nm" % (sum(torques) / len(torques)) if torques else "     -    "
            rms = math.sqrt(audio_sq / audio_samples) if audio_samples else 0.0
            config = ("flat-eq %s intake %s exhaust %s" % tuple("on" if v else "off" for v in sound_config[1:])) if sound_config else "-"
            print("%5.1fs  rpm %5.0f  torque %s  keep-beam %3d  profile %d  sound [%s]  audio %5d samples/s rms %.3f peak %.2f"
                  % (t, rpm, torque, keeps, profiles, config, audio_samples, rms, audio_peak))
            torques, keeps, profiles = [], 0, 0
            audio_samples, audio_sq, audio_peak = 0, 0.0, 0.0
            next_report += 1.0

        time.sleep(0.001)


if __name__ == "__main__":
    main()
