"""Golden key material shared by every protocol test module.

These live in the package ``__init__`` rather than in a ``conftest.py`` because
they are *constants*, not fixtures. The golden packet vectors in
``test_control`` and ``test_telemetry`` are byte-exact only under one specific
key, so that key must be impossible to redefine per-module -- a second copy that
drifts by one character would turn every golden assertion into a tautology that
passes while proving nothing.

The hex literals below are pinned deliberately. ``GOLDEN_UDP_KEY`` is *also*
recomputed from the KDF in ``test_control`` and compared against this constant,
which is what catches a change to ``derive_udp_key`` itself -- otherwise a KDF
change would silently regenerate every "golden" byte string on both sides at
once and the two halves of the system would agree on being wrong together.
"""

from __future__ import annotations

#: sha256(b"telekart-golden-passphrase") -- what `normalize_shared_key` returns.
GOLDEN_PASSPHRASE = "telekart-golden-passphrase"
GOLDEN_SHARED_KEY = bytes.fromhex(
    "55a36e026da668f77c222a6864fa92329809d2c8b9e1fff16b9e7b3e98903b3d"
)

#: A fixed stand-in for `make_session_token()`, which is random by design.
GOLDEN_SESSION_TOKEN = bytes(range(16))

#: derive_udp_key(GOLDEN_SHARED_KEY, GOLDEN_SESSION_TOKEN)
GOLDEN_UDP_KEY = bytes.fromhex(
    "e0d17c268b95843487cc6c6d30fee5b076e8a8c581c4d12a5e684f0a59fd2cd2"
)

#: A second, unrelated key. Used to prove that a packet signed by one session
#: is rejected by another -- the actual threat model (a stale second laptop),
#: not a forged tag.
OTHER_UDP_KEY = bytes.fromhex(
    "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff"
)


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
