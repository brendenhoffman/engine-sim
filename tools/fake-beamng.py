#!/usr/bin/env python3
"""Stand-in for BeamNG + the FMOD plugin, for testing the bridge without the game.

Sends vehicle packets ("BES1" rpm, throttle, ignition, volume, pan) to engine-sim like the
mod's Lua does, prints the torque replies, and listens on the plugin's audio port to check
that engine sound arrives.

  tools/fake-beamng.py                       # rev between 1500 and 5000 rpm for 20 s
  tools/fake-beamng.py --rpm 3000 --throttle 0.6 --seconds 10
  tools/fake-beamng.py --physics --throttle 0.7  # spin a toy car with the torque engine-sim sends

--physics runs a crude drivetrain: the engine's flywheel (the car's inertia, or engine-sim's once
tuned) on a springy driveshaft to a car in a fixed gear. It's for spotting torque feedback that
makes the rpm hunt, not for realistic driving.
"""

import argparse
import json
import math
import socket
import struct
import time


# What the mod's game-engine extension sends for a stock car (values are illustrative)
SAMPLE_VEHICLE = {
    "v": 1,
    "vehicle": {"id": 12345, "directory": "fullsize", "model": "Gavril Grand Marshal"},
    "engine": {
        "part": "fullsize_engine_v8_5.5", "name": "5.5L V8 Engine", "cylinders": 8, "fuel": "gasoline",
        "idle_rpm": 650, "max_rpm": 5500, "rev_limiter_rpm": 5400, "inertia": 0.18,
        "friction": 18, "dynamic_friction": 0.025, "engine_brake_torque": 46,
        "torque": [[0, 0], [500, 260], [1000, 360], [2000, 420], [3000, 440], [4000, 410], [5000, 350], [6000, 240]],
        "placement": "outside", "engine_sample": "v8_crossplane_intake", "exhaust_sample": "v8_crossplane_exhaust",
    },
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4570, help="engine-sim bridge port")
    parser.add_argument("--audio-port", type=int, default=4571, help="port the FMOD plugin would listen on")
    parser.add_argument("--rpm", type=float, help="hold this rpm instead of sweeping")
    parser.add_argument("--throttle", type=float, default=0.5)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=60.0, help="packets per second (BeamNG sends one per graphics frame)")
    parser.add_argument("--no-vehicle", action="store_true", help="don't send the sample car description")
    parser.add_argument("--vehicle", help="send this car description (a JSON object, or a line of beamng-vehicles.jsonl) instead of the sample")
    parser.add_argument("--physics", action="store_true", help="drive the rpm with the torque engine-sim sends")
    parser.add_argument("--gear", type=float, default=4.0, help="--physics: overall gear ratio")
    parser.add_argument("--damping", type=float, default=0.3, help="--physics: driveshaft damping ratio")
    args = parser.parse_args()

    vehicle = SAMPLE_VEHICLE
    if args.vehicle:
        with open(args.vehicle) as f:
            vehicle = json.loads(f.readline())
        vehicle = vehicle.get("vehicle_info", vehicle)
    car_engine = vehicle.get("engine") or {}
    table = sorted(car_engine.get("torque") or [[0, 0], [7000, 0]])

    def table_torque(rpm):
        for (r0, t0), (r1, t1) in zip(table, table[1:]):
            if rpm <= r1:
                return t0 + (t1 - t0) * (rpm - r0) / max(r1 - r0, 1)
        return table[-1][1]

    # --physics state: engine and car speeds (rad/s), driveshaft twist (rad)
    rpm_to_av = math.pi / 30
    engine_av = 2000 * rpm_to_av
    car_av = engine_av
    twist = 0.0
    engine_inertia = float(car_engine.get("inertia") or 0.15)
    car_inertia = 1500 * 0.32 ** 2 / args.gear ** 2   # the car's mass seen at the crank
    shaft_k = engine_inertia * (2 * math.pi * 12) ** 2  # ~12 Hz shuffle, like a real driveline
    shaft_c = 2 * args.damping * math.sqrt(shaft_k * engine_inertia)
    limiter = float(car_engine.get("rev_limiter_rpm") or 6500)
    sent_torque = float("nan")
    last_physics = time.monotonic()
    rpms = []

    link = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    link.connect((args.host, args.port))
    link.setblocking(False)

    audio = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    audio.bind((args.host, args.audio_port))
    audio.setblocking(False)

    start = time.monotonic()
    next_send = start
    next_report = start + 1.0
    next_vehicle = start
    torques, keeps, profiles = [], 0, 0
    sound_config = None
    audio_samples, audio_sq, audio_peak = 0, 0.0, 0.0

    while time.monotonic() - start < args.seconds:
        now = time.monotonic()
        t = now - start

        if args.physics:
            # Small fixed steps up to now
            while last_physics < now:
                h = 0.0005
                last_physics += h
                rpm_now = engine_av / rpm_to_av
                if math.isnan(sent_torque):
                    drive = table_torque(rpm_now) * args.throttle - (1 - args.throttle) * 40
                else:
                    drive = sent_torque
                if rpm_now > limiter:
                    drive = min(drive, 0)
                shaft = shaft_k * twist + shaft_c * (engine_av - car_av)
                engine_av += (drive - shaft) / engine_inertia * h
                car_av += (shaft - 1.4e-6 * car_av ** 3) / car_inertia * h  # drag: ~200 Nm at 5000 rpm
                twist += (engine_av - car_av) * h
                engine_av = max(engine_av, 600 * rpm_to_av)
            rpms.append(engine_av / rpm_to_av)

        if now >= next_send:
            rpm = engine_av / rpm_to_av if args.physics else args.rpm if args.rpm is not None else 3250 + 1750 * math.sin(t * 0.6)
            link.send(b"BES1" + struct.pack("<5f", rpm, args.throttle, 1.0, 1.0, 0.0))
            next_send += 1.0 / args.rate

        if not args.no_vehicle and now >= next_vehicle:
            link.send(b"BESV" + json.dumps(vehicle).encode())
            next_vehicle += 3.0

        while True:
            try:
                data = link.recv(2048)
            except BlockingIOError:
                break
            except ConnectionRefusedError:
                break  # engine-sim not listening (yet)
            tag = data[:4]
            if tag == b"BES1" and len(data) >= 12:
                sent_torque = struct.unpack("<2f", data[4:12])[0]
                torques.append(sent_torque)
            elif tag == b"BESN":
                keeps += 1
                sent_torque = float("nan")
            elif tag == b"BESP":
                profiles += 1
                fields = struct.unpack("<%df" % ((len(data) - 4) // 4), data[4:])
                if args.physics and len(fields) > 4 and fields[0] == 1 and fields[4] > 0 and fields[4] != engine_inertia:
                    # Tuned: the car takes engine-sim's flywheel and limiter
                    engine_inertia = fields[4]
                    limiter = fields[3]
                    shaft_k = engine_inertia * (2 * math.pi * 12) ** 2
                    shaft_c = 2 * args.damping * math.sqrt(shaft_k * engine_inertia)
            elif tag == b"BESC" and len(data) >= 20:
                sound_config = struct.unpack("<4f", data[4:20])  # version, flat EQ, intake level, exhaust level

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
            rpm = engine_av / rpm_to_av if args.physics else args.rpm if args.rpm is not None else 3250 + 1750 * math.sin(t * 0.6)
            if args.physics and rpms:
                print("%5.1fs  rpm %5.0f (%5.0f-%5.0f)  inertia %.3f  torque %s" % (t, rpm, min(rpms), max(rpms), engine_inertia,
                      ("%7.1f Nm (%6.1f-%6.1f)" % (sum(torques) / len(torques), min(torques), max(torques))) if torques and not any(map(math.isnan, torques)) else "car's own"))
                rpms = []
            torque = "%7.1f Nm" % (sum(torques) / len(torques)) if torques else "     -    "
            rms = math.sqrt(audio_sq / audio_samples) if audio_samples else 0.0
            config = ("flat-eq %s intake %.0f%% exhaust %.0f%%" % ("on" if sound_config[1] else "off", sound_config[2] * 100, sound_config[3] * 100)) if sound_config else "-"
            if not args.physics:
                print("%5.1fs  rpm %5.0f  torque %s  keep-beam %3d  profile %d  sound [%s]  audio %5d samples/s rms %.3f peak %.2f"
                      % (t, rpm, torque, keeps, profiles, config, audio_samples, rms, audio_peak))
            torques, keeps, profiles = [], 0, 0
            audio_samples, audio_sq, audio_peak = 0, 0.0, 0.0
            next_report += 1.0

        time.sleep(0.001)


if __name__ == "__main__":
    main()
