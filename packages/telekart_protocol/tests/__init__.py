"""Helpers shared by every protocol test module.

There is no key material here any more. Earlier revisions authenticated every
UDP packet with a truncated HMAC keyed per session, and the golden vectors in
``test_control`` and ``test_telemetry`` were byte-exact only under one pinned
key. That whole layer was removed: the packets now carry no tag, so a golden
vector depends on nothing but the field layout.

What that costs is written up in ``control.py``. In test terms specifically:
the suite can no longer assert that a packet from one session is rejected by
another, because nothing distinguishes them but the session id -- so those
tests are gone rather than rewritten into something weaker that looks like it
still proves something.
"""

from __future__ import annotations


def flip_bit(data: bytes, byte_index: int, bit: int = 0) -> bytes:
    """Return `data` with a single bit inverted. For tamper tests."""
    mutable = bytearray(data)
    mutable[byte_index] ^= 1 << bit
    return bytes(mutable)


def splice(data: bytes, offset: int, replacement: bytes) -> bytes:
    """Overwrite `replacement` into `data` at `offset`, keeping the length."""
    mutable = bytearray(data)
    mutable[offset : offset + len(replacement)] = replacement
    return bytes(mutable)
