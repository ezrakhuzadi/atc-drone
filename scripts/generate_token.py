#!/usr/bin/env python3
"""
Generate a dummy JWT token for Flight Blender API access.

With BYPASS_AUTH_TOKEN_VERIFICATION=1 set in Blender's .env,
the token signature is not verified, but the token structure,
issuer, audience, and scopes are still checked.
"""

import jwt
import time
import argparse


def generate_dummy_token(
    scopes: str = "flightblender.read flightblender.write",
    audience: str = "testflight.flightblender.com",
    expiry_hours: int = 24
) -> str:
    """Generate a dummy JWT token for Flight Blender API access.
    
    Args:
        scopes: Space-separated list of scopes
        audience: Token audience (must match PASSPORT_AUDIENCE in Blender)
        expiry_hours: Token validity in hours
        
    Returns:
        JWT token string
    """
    now = int(time.time())
    payload = {
        "iss": "dummy",
        "sub": "atc-drone-system",
        "aud": audience,
        "scope": scopes,
        "exp": now + (expiry_hours * 3600),
        "iat": now
    }
    # With BYPASS_AUTH_TOKEN_VERIFICATION=1, signature isn't verified
    # but we still need a valid JWT structure
    token = jwt.encode(payload, "dummy-secret", algorithm="HS256")
    return token


def main():
    parser = argparse.ArgumentParser(description="Generate dummy JWT for Flight Blender")
    parser.add_argument("--scopes", default="flightblender.read flightblender.write",
                        help="Space-separated scopes (default: flightblender.read flightblender.write)")
    parser.add_argument("--audience", default="testflight.flightblender.com",
                        help="Token audience (default: testflight.flightblender.com)")
    parser.add_argument("--expiry", type=int, default=24,
                        help="Token expiry in hours (default: 24)")
    
    args = parser.parse_args()
    
    token = generate_dummy_token(
        scopes=args.scopes,
        audience=args.audience,
        expiry_hours=args.expiry
    )
    print(token)


if __name__ == "__main__":
    main()
