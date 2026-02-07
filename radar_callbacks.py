#!/usr/bin/env python3
"""
Example script showing how to consume Tesla radar tracks via callbacks.

Mirrors toyota-radar's radar_callbacks.py but with Tesla-specific fields
(probability, classification, lateral speed).
"""

import argparse
import time
from typing import Dict

from tesla_radar_driver import RadarTrack, TeslaRadarConfig, TeslaRadarDriver


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Log Tesla radar tracks using the modular radar driver."
    )
    parser.add_argument("--channel", default="can0", help="CAN channel.")
    parser.add_argument(
        "--interface",
        default="socketcan",
        help="python-can interface (socketcan, canalystii).",
    )
    parser.add_argument("--bitrate", type=int, default=500000, help="CAN bitrate.")
    parser.add_argument(
        "--radar-dbc",
        default="opendbc/tesla_radar.dbc",
        help="DBC used to decode radar tracks.",
    )
    parser.add_argument(
        "--vin",
        default=None,
        help="VIN to send (default: auto-read from radar via UDS).",
    )
    parser.add_argument(
        "--no-auto-vin",
        action="store_true",
        help="Skip auto-reading VIN from radar at startup.",
    )
    parser.add_argument(
        "--no-setup",
        action="store_true",
        help="Skip bringing interfaces up with ip link.",
    )
    parser.add_argument(
        "--use-sudo",
        action="store_true",
        help="Run interface setup commands with sudo.",
    )
    parser.add_argument(
        "--setup-extra",
        action="append",
        default=[],
        metavar="TOKEN",
        help="Extra tokens to prefix ip link commands (repeatable).",
    )
    parser.add_argument(
        "--keepalive-rate-hz",
        type=float,
        default=100.0,
        help="Frequency for radar keep-alive loop.",
    )
    parser.add_argument(
        "--track-timeout",
        type=float,
        default=0.5,
        help="Seconds before removing stale tracks from the driver cache.",
    )
    parser.add_argument(
        "--print-interval",
        type=float,
        default=0.25,
        help="Minimum seconds between prints for the same track.",
    )
    parser.add_argument(
        "--summary-interval",
        type=float,
        default=5.0,
        help="Seconds between summary statistics prints.",
    )
    parser.add_argument("--debug", action="store_true", help="Enable debug output.")
    return parser.parse_args()


class TrackLogger:
    def __init__(self, min_interval: float) -> None:
        self._last_print: Dict[int, float] = {}
        self._min_interval = max(min_interval, 0.0)

    def __call__(self, track: RadarTrack) -> None:
        now = time.time()
        last = self._last_print.get(track.track_id, 0.0)
        if now - last >= self._min_interval:
            self._last_print[track.track_id] = now
            # Show Tesla-specific fields if available
            extra = ""
            if hasattr(track, "prob_exist"):
                extra = f" prob={track.prob_exist:5.1f}% cls={track.object_class}"
            print(
                f"{time.strftime('%H:%M:%S')} track {track.track_id:2d} "
                f"long={track.long_dist:6.2f}m lat={track.lat_dist:6.2f}m "
                f"rel_speed={track.rel_speed:6.2f}m/s new={track.new_track}"
                f"{extra}"
            )


def main() -> None:
    args = parse_args()

    config = TeslaRadarConfig(
        channel=args.channel,
        interface=args.interface,
        bitrate=args.bitrate,
        radar_dbc=args.radar_dbc,
        vin=args.vin,
        keepalive_rate_hz=args.keepalive_rate_hz,
        track_timeout=args.track_timeout,
        auto_vin=not args.no_auto_vin,
        auto_setup=not args.no_setup,
        use_sudo=args.use_sudo,
        setup_extra_args=args.setup_extra,
        debug=args.debug,
    )

    driver = TeslaRadarDriver(config)
    driver.register_track_callback(TrackLogger(args.print_interval))

    try:
        driver.start()
    except Exception as exc:
        driver.stop()
        raise SystemExit(f"Failed to start radar driver: {exc}")

    print("Tesla radar driver started. Waiting for track callbacks (Ctrl+C to exit).")

    try:
        next_summary = time.time() + args.summary_interval
        while True:
            time.sleep(0.1)
            if time.time() >= next_summary:
                tracks = driver.get_tracks()
                count = len(tracks)
                status = driver.keepalive_status()
                parts = [
                    f"Tracks cached: {count}",
                    f"RX messages: {driver.message_count()}",
                ]
                if status:
                    parts.append(f"KA TX: {int(status['tx_count'])}")
                    radar_st = {0: "Not Present", 1: "Initializing", 2: "Active"}
                    parts.append(f"Radar: {radar_st.get(status.get('radar_status', 0), '?')}")
                    if status.get("last_error"):
                        parts.append(f"ERR: {status['last_error']}")
                print(" | ".join(parts))
                next_summary = time.time() + args.summary_interval
    except KeyboardInterrupt:
        print("\nStopping driver...")
    finally:
        driver.stop()


if __name__ == "__main__":
    main()
