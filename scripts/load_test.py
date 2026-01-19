#!/usr/bin/env python3
import argparse
import datetime as dt
import json
import random
import threading
import time
import urllib.request


def post_json(url, payload, headers=None):
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Content-Type", "application/json")
    if headers:
        for key, value in headers.items():
            req.add_header(key, value)
    with urllib.request.urlopen(req, timeout=10) as resp:
        body = resp.read().decode("utf-8")
    return body


def register_drone(base_url, drone_id, registration_token=None, owner_id=None):
    url = f"{base_url}/v1/drones/register"
    payload = {"drone_id": drone_id, "owner_id": owner_id}
    headers = {}
    if registration_token:
        headers["X-Registration-Token"] = registration_token
    body = post_json(url, payload, headers=headers)
    return json.loads(body)


def telemetry_loop(base_url, drone_id, token, duration_s, interval_s, center_lat, center_lon):
    end_time = time.time() + duration_s
    url = f"{base_url}/v1/telemetry"
    headers = {"Authorization": f"Bearer {token}"}
    while time.time() < end_time:
        lat = center_lat + random.uniform(-0.001, 0.001)
        lon = center_lon + random.uniform(-0.001, 0.001)
        payload = {
            "drone_id": drone_id,
            "lat": lat,
            "lon": lon,
            "altitude_m": random.uniform(60, 120),
            "heading_deg": random.uniform(0, 360),
            "speed_mps": random.uniform(5, 15),
            "timestamp": dt.datetime.utcnow().replace(microsecond=0).isoformat() + "Z",
        }
        try:
            post_json(url, payload, headers=headers)
        except Exception as exc:
            print(f"[{drone_id}] telemetry failed: {exc}")
        time.sleep(interval_s)


def main():
    parser = argparse.ArgumentParser(description="ATC-Drone telemetry load test.")
    parser.add_argument("--base-url", default="http://localhost:3000")
    parser.add_argument("--drones", type=int, default=5)
    parser.add_argument("--duration", type=int, default=30)
    parser.add_argument("--interval-ms", type=int, default=1000)
    parser.add_argument("--registration-token", default=None)
    parser.add_argument("--owner-id", default=None)
    parser.add_argument("--center-lat", type=float, default=33.6846)
    parser.add_argument("--center-lon", type=float, default=-117.8265)
    args = parser.parse_args()

    interval_s = max(args.interval_ms, 50) / 1000.0

    print(f"Registering {args.drones} drones...")
    tokens = []
    for i in range(args.drones):
        drone_id = f"LOAD-{i:03d}"
        response = register_drone(
            args.base_url,
            drone_id,
            registration_token=args.registration_token,
            owner_id=args.owner_id,
        )
        tokens.append((drone_id, response["session_token"]))

    threads = []
    for drone_id, token in tokens:
        thread = threading.Thread(
            target=telemetry_loop,
            args=(
                args.base_url,
                drone_id,
                token,
                args.duration,
                interval_s,
                args.center_lat,
                args.center_lon,
            ),
            daemon=True,
        )
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    print("Load test complete.")


if __name__ == "__main__":
    main()
