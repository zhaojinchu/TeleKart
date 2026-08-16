#!/bin/sh
#
# Layer 2 of the four-layer panic stop: THE ONLY LAYER THAT COVERS SIGKILL.
#
# Every in-process hook -- atexit, the signal handlers, sys.excepthook,
# threading.excepthook, the context manager around main() -- needs Python to
# still be running to fire. SIGKILL does not ask, a segfault does not return,
# and the OOM killer does not negotiate. pigpiod retains GPIO state after its
# client dies, so any of those leaves the motors turning at whatever duty they
# were last given, indefinitely.
#
# systemd runs this from ExecStopPost= on EVERY stop path -- clean stop, crash,
# watchdog expiry, crash-restart, kill -9, OOM kill.
#
# Deliberately POSIX shell with no Python, no venv, and no imports, because the
# situation it exists for is the one where Python is the thing that died. It
# talks to pigpiod through `pigs`, which is a small C binary.
#
# Install:  sudo install -m 0755 panic_stop.sh /usr/local/sbin/telekart-panic-stop
# Verify:   sudo /usr/local/sbin/telekart-panic-stop && echo ok
#
set -u

# Build order matters: pi-setup.md purges the Debian package precisely so there
# is only one pigs on the machine, but check both prefixes anyway -- a panic
# stop that failed because of $PATH would be a poor epitaph.
PIGS=""
for candidate in /usr/local/bin/pigs /usr/bin/pigs; do
    if [ -x "$candidate" ]; then
        PIGS="$candidate"
        break
    fi
done

if [ -z "$PIGS" ]; then
    echo "telekart-panic-stop: pigs not found; cannot safe the outputs" >&2
    # Exit 0 regardless: see the note at the bottom.
    exit 0
fi

# Pin map from docs/wiring.md. Hard-coded rather than read from config.yaml
# because parsing YAML needs a parser, and a parser needs a runtime, and the
# runtime is what we are assuming is gone.
ENA=12
ENB=13
IN1=5
IN2=6
IN3=20
IN4=21
SERVO=18
LED=25

# ---------------------------------------------------------------------------
# ORDER MATTERS, AND IT IS THE OPPOSITE OF INTUITIVE.
#
# The enables come down FIRST so the bridge coasts. On an L298 with EN high,
# IN1 == IN2 -- both high OR both low -- is a BRAKE, not a coast. Clearing the
# direction pins while an enable is still high therefore walks the outputs
# through a hard short across a spinning motor, at the exact moment you were
# trying to make things safe. Coast requires EN LOW, and nothing else does.
# ---------------------------------------------------------------------------

# `hp <pin> 0 0` stops the hardware PWM channel; the following `w` drives the
# pin low outright, so it leaves PWM mode and holds a level that survives this
# script exiting.
"$PIGS" hp "$ENA" 0 0  2>/dev/null
"$PIGS" hp "$ENB" 0 0  2>/dev/null
"$PIGS" w  "$ENA" 0    2>/dev/null
"$PIGS" w  "$ENB" 0    2>/dev/null

# Only now are the direction pins safe to clear.
"$PIGS" w  "$IN1" 0    2>/dev/null
"$PIGS" w  "$IN2" 0    2>/dev/null
"$PIGS" w  "$IN3" 0    2>/dev/null
"$PIGS" w  "$IN4" 0    2>/dev/null

# Stop the servo pulse train: the HS-311 goes limp and stops drawing holding
# current from the Pi's own 5 V rail, which is also the SoC's.
"$PIGS" s  "$SERVO" 0  2>/dev/null

# Status LED off, so a dark LED means "not running" without ambiguity.
"$PIGS" w  "$LED" 0    2>/dev/null

# ---------------------------------------------------------------------------
# Always exit 0.
#
# A non-zero ExecStopPost= puts the unit into a failed state, and a failed unit
# does not restart. The outputs have already been driven low by this point; the
# only thing a failure exit could still achieve is preventing the car from
# coming back. If pigpiod itself is gone, layer 3 -- the external 10k
# pull-downs on ENA/ENB/IN1-IN4 -- is what holds the bridge off.
# ---------------------------------------------------------------------------
exit 0
