#!/usr/bin/env python3
import argparse
import datetime as dt
import json
import urllib.request
import urllib.error


def request_json(url, method="GET", payload=None, headers=None):
    data = None
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("Content-Type", "application/json")
    if headers:
        for key, value in headers.items():
            req.add_header(key, value)
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            body = resp.read().decode("utf-8")
        return resp.status, body
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read().decode("utf-8")


def main():
    parser = argparse.ArgumentParser(description="ATC-Drone failure-mode smoke tests.")
    parser.add_argument("--base-url", default="http://localhost:3000")
    parser.add_argument("--registration-token", default=None)
    parser.add_argument("--admin-token", default=None)
    args = parser.parse_args()

    print("== Register without token (should fail if registration token is required) ==")
    status, body = request_json(
        f"{args.base_url}/v1/drones/register",
        method="POST",
        payload={"drone_id": "FAIL-001"},
    )
    print(f"Status: {status} Body: {body[:160]}")

    print("== Register with token (expected success) ==")
    headers = {}
    if args.registration_token:
        headers["X-Registration-Token"] = args.registration_token
    status, body = request_json(
        f"{args.base_url}/v1/drones/register",
        method="POST",
        payload={"drone_id": "FAIL-001"},
        headers=headers,
    )
    print(f"Status: {status} Body: {body[:160]}")
    if status not in (200, 201):
        print("Registration failed; aborting remaining tests.")
        return
    token = json.loads(body)["session_token"]

    print("== Telemetry with invalid latitude (should fail) ==")
    bad_payload = {
        "drone_id": "FAIL-001",
        "lat": 999,
        "lon": -117.8265,
        "altitude_m": 90,
        "heading_deg": 90,
        "speed_mps": 10,
        "timestamp": dt.datetime.utcnow().replace(microsecond=0).isoformat() + "Z",
    }
    status, body = request_json(
        f"{args.base_url}/v1/telemetry",
        method="POST",
        payload=bad_payload,
        headers={"Authorization": f"Bearer {token}"},
    )
    print(f"Status: {status} Body: {body[:160]}")

    print("== Telemetry with valid payload (should accept) ==")
    good_payload = dict(bad_payload)
    good_payload["lat"] = 33.6846
    status, body = request_json(
        f"{args.base_url}/v1/telemetry",
        method="POST",
        payload=good_payload,
        headers={"Authorization": f"Bearer {token}"},
    )
    print(f"Status: {status} Body: {body[:160]}")

    if args.admin_token:
        print("== Admin reset without confirm (should fail) ==")
        status, body = request_json(
            f"{args.base_url}/v1/admin/reset",
            method="POST",
            payload={},
            headers={"Authorization": f"Bearer {args.admin_token}"},
        )
        print(f"Status: {status} Body: {body[:160]}")

        print("== Admin reset with confirm (expected success) ==")
        status, body = request_json(
            f"{args.base_url}/v1/admin/reset",
            method="POST",
            payload={"confirm": "RESET"},
            headers={"Authorization": f"Bearer {args.admin_token}"},
        )
        print(f"Status: {status} Body: {body[:160]}")

    print("Failure-mode smoke tests complete.")


if __name__ == "__main__":
    main()
