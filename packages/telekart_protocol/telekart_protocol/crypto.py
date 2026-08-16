"""Packet authentication.

Threat model, stated plainly so nobody over-engineers this later: the risk is a
*stale or second* controller injecting commands into a moving car -- a laptop
that was disconnected and reconnected, a leftover process, another student on
the same LAN. It is not a determined attacker with a packet capture. A
truncated HMAC over a strictly-increasing sequence number covers that
completely, at roughly 15 microseconds per packet.

The key is derived from a session token issued during the TCP handshake rather
than being a constant passed on the command line, so a key never sits in shell
history or a screenshot.
"""

from __future__ import annotations

import hashlib
import hmac
import os

from .constants import MAC_TAG_LEN

_KDF_INFO = b"telekart-udp-v2"


def make_session_token() -> bytes:
    """Generate a fresh 16-byte session token (car side, per connection)."""
    return os.urandom(16)


def derive_udp_key(shared_key: bytes, session_token: bytes) -> bytes:
    """Derive the per-session UDP tagging key.

    HMAC-SHA256 used as a one-step KDF: the shared secret keys the MAC, and the
    session token plus a domain-separation label form the message. Both ends run
    this identically after the handshake.
    """
    if not shared_key:
        raise ValueError("shared_key must not be empty")
    if len(session_token) < 8:
        raise ValueError("session_token must be at least 8 bytes")
    return hmac.new(shared_key, _KDF_INFO + session_token, hashlib.sha256).digest()


def compute_tag(key: bytes, payload: bytes) -> bytes:
    """Compute the truncated HMAC tag over a packet body.

    `payload` must be the full packet with the tag field already zeroed --
    see `control.pack` / `telemetry.pack`, which handle that for you.
    """
    return hmac.new(key, payload, hashlib.sha256).digest()[:MAC_TAG_LEN]


def verify_tag(key: bytes, payload: bytes, tag: bytes) -> bool:
    """Constant-time tag verification."""
    return hmac.compare_digest(compute_tag(key, payload), tag)


def normalize_shared_key(value: str | bytes) -> bytes:
    """Accept a human-typed passphrase or raw bytes and return key material.

    A passphrase is hashed rather than used directly so that short, memorable
    keys still produce full-width key material.
    """
    if isinstance(value, str):
        value = value.encode("utf-8")
    if not value:
        raise ValueError("shared key must not be empty")
    return hashlib.sha256(value).digest()
