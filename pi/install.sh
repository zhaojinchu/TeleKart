#!/usr/bin/env bash
#
# Install the TeleKart control and video services onto a Raspberry Pi.
#
# Idempotent by construction: every step either overwrites a file with the same
# content it would have written the first time, or checks before creating. Run
# it as often as you like -- after a `git pull`, after editing a unit, after
# wondering whether you ever ran it at all.
#
# It deliberately does NOT start telekart-control or telekart-video. Bring-up
# runs the subsystems by hand, in order, with gates (docs/bringup.md); starting
# the full stack against untested wiring is how you find out what your master
# switch is for.
#
#   sudo ./pi/install.sh              # install and enable
#   sudo ./pi/install.sh --dry-run    # print what it would do
#   sudo ./pi/install.sh --no-enable  # install files only
#
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
PI_DIR="$(dirname "$SCRIPT_PATH")"
REPO_ROOT="$(dirname "$PI_DIR")"

SYSTEMD_DIR=/etc/systemd/system
PANIC_TARGET=/usr/local/sbin/telekart-panic-stop
ENV_DIR=/etc/telekart
ENV_FILE="$ENV_DIR/telekart.env"
NM_CONF=/etc/NetworkManager/conf.d/99-wifi-powersave-off.conf

# The units are written against this prefix and rewritten to wherever the
# checkout actually lives.
TEMPLATE_ROOT=/home/pi/telekart

DRY_RUN=0
DO_ENABLE=1
WARNINGS=0

for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        --no-enable) DO_ENABLE=0 ;;
        -h|--help)
            sed -n '2,20p' "$SCRIPT_PATH" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "unknown option: $arg" >&2
            exit 2
            ;;
    esac
done

say()  { printf '  %s\n' "$*"; }
# Silent under --dry-run, where `run` has already printed the intent and a
# second line claiming the work happened would be a lie.
done_say() { [ "$DRY_RUN" -eq 1 ] || printf '  %s\n' "$*"; }
step() { printf '\n== %s\n' "$*"; }
warn() { printf '  !! %s\n' "$*" >&2; WARNINGS=$((WARNINGS + 1)); }

run() {
    if [ "$DRY_RUN" -eq 1 ]; then
        printf '  [dry-run] %s\n' "$*"
        return 0
    fi
    "$@"
}

if [ "$DRY_RUN" -eq 0 ] && [ "$(id -u)" -ne 0 ]; then
    echo "install.sh must run as root (try: sudo $0)" >&2
    exit 1
fi

step "TeleKart install"
say "repository : $REPO_ROOT"

# ---------------------------------------------------------------------------
# Where Python lives
# ---------------------------------------------------------------------------
VENV_PY="$REPO_ROOT/.venv/bin/python"
if [ ! -x "$VENV_PY" ]; then
    warn "no venv at $REPO_ROOT/.venv -- falling back to $(command -v python3)"
    warn "see docs/pi-setup.md section 4: the venv needs --system-site-packages"
    VENV_PY="$(command -v python3 || echo /usr/bin/python3)"
fi
say "interpreter: $VENV_PY"

# The unit runs as whoever owns the checkout, unless that is nobody sensible.
# Root is the documented answer for this build (sched_setscheduler and mlock),
# and the capability lines in the unit are there so a future non-root user works.
SERVICE_USER=root
say "service user: $SERVICE_USER"

# ---------------------------------------------------------------------------
# Preflight. Warnings, never failures: an installer that refuses to copy a file
# because a kernel module is loaded helps nobody.
# ---------------------------------------------------------------------------
step "Preflight"

BOOT_CONFIG=/boot/firmware/config.txt
[ -f "$BOOT_CONFIG" ] || BOOT_CONFIG=/boot/config.txt

if [ -f "$BOOT_CONFIG" ]; then
    if grep -qE '^\s*dtparam=audio=off' "$BOOT_CONFIG"; then
        say "dtparam=audio=off is set"
    else
        warn "dtparam=audio=off is MISSING from $BOOT_CONFIG"
        warn "  snd_bcm2835 claims BOTH hardware PWM channels -- which are ENA"
        warn "  (GPIO12) and ENB (GPIO13). Both motors will be dead and pigpio"
        warn "  will report success. Fix this before bring-up."
    fi
    if grep -qE 'i2s|hifiberry|iqaudio|audioinjector|googlevoicehat|allo-|rpi-dac' \
            "$BOOT_CONFIG"; then
        warn "an I2S overlay is present in $BOOT_CONFIG"
        warn "  it reassigns GPIO18 (servo), 20 (IN3) and 21 (IN4) to the PCM"
        warn "  peripheral at boot. See docs/wiring.md section 7."
    fi
else
    warn "no config.txt found; this does not look like a Raspberry Pi"
fi

if lsmod 2>/dev/null | grep -q '^snd_bcm2835'; then
    warn "snd_bcm2835 is loaded right now"
    # Observed on a Desktop image: dtparam=audio=off stops the dtoverlay path,
    # but vc4's KMS driver still pulls snd_bcm2835 in through snd_soc_hdmi_codec.
    # The dtparam alone is therefore NOT sufficient on any image with the
    # desktop/KMS stack, which is exactly the image most people flash first.
    if [ ! -f /etc/modprobe.d/telekart-no-audio.conf ]; then
        warn "  dtparam=audio=off alone does not stop this on a KMS/desktop image;"
        warn "  the HDMI audio codec drags it back in. Blacklist it as well:"
        warn "    echo 'blacklist snd_bcm2835' | sudo tee /etc/modprobe.d/telekart-no-audio.conf"
        warn "    sudo rmmod snd_bcm2835"
    else
        warn "  blacklisted already -- it will stay out after the next reboot"
    fi
fi

if [ -x /usr/local/bin/pigpiod ]; then
    say "pigpiod: $(/usr/local/bin/pigpiod -v 2>/dev/null || echo 'version unknown')"
elif command -v pigpiod >/dev/null 2>&1; then
    warn "pigpiod is $(command -v pigpiod), not /usr/local/bin/pigpiod"
    warn "  the drop-in unit uses the absolute path; purge the Debian package"
    warn "  and build v79 from source (docs/pi-setup.md section 3)"
else
    warn "pigpiod is not installed; the control process cannot touch a pin"
fi

if [ -n "$(swapon --show 2>/dev/null)" ]; then
    warn "swap is enabled -- a major fault mid-tick is unbounded latency"
    warn "  sudo systemctl disable --now dphys-swapfile"
fi

if ! systemctl is-active --quiet avahi-daemon 2>/dev/null; then
    warn "avahi-daemon is not active; mDNS discovery will not work"
    warn "  the app can still be pointed at an IP address by hand"
fi

# ---------------------------------------------------------------------------
# Layer 2 of the panic-stop chain
# ---------------------------------------------------------------------------
step "Panic-stop script"
run install -D -m 0755 "$PI_DIR/scripts/panic_stop.sh" "$PANIC_TARGET"
done_say "installed $PANIC_TARGET"
if [ "$DRY_RUN" -eq 0 ] && [ ! -x "$PANIC_TARGET" ]; then
    echo "panic-stop script is not executable at $PANIC_TARGET" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Shared key
# ---------------------------------------------------------------------------
step "Shared key"
if [ -f "$ENV_FILE" ]; then
    say "$ENV_FILE already exists; leaving it alone"
else
    run install -d -m 0750 "$ENV_DIR"
    if [ "$DRY_RUN" -eq 0 ]; then
        # Generated rather than templated: a key that ships in a repository is
        # not a key. This one is 32 hex characters from the kernel's CSPRNG.
        GENERATED="$(head -c 16 /dev/urandom | od -An -tx1 | tr -d ' \n')"
        umask 077
        cat >"$ENV_FILE" <<EOF
# TeleKart secrets. Root-only, 0600, and NOT in git.
# The desktop app must be given the same key to connect.
TELEKART_SHARED_KEY=$GENERATED
# TELEKART_CAR_ID=telekart-01
EOF
        chmod 0600 "$ENV_FILE"
        say "generated a fresh shared key in $ENV_FILE"
        say "the desktop app needs it: sudo grep SHARED_KEY $ENV_FILE"
    else
        printf '  [dry-run] generate %s with a random shared key\n' "$ENV_FILE"
    fi
fi

# ---------------------------------------------------------------------------
# Units
# ---------------------------------------------------------------------------
step "systemd units"

install_unit() {
    local source="$1" target="$2"
    if [ "$DRY_RUN" -eq 1 ]; then
        printf '  [dry-run] render %s -> %s\n' "$(basename "$source")" "$target"
        return 0
    fi
    local tmp
    tmp="$(mktemp)"
    # Longest path first: the interpreter line contains the repo prefix, so
    # rewriting the prefix first would leave a stale .venv path behind.
    sed -e "s|$TEMPLATE_ROOT/.venv/bin/python|$VENV_PY|g" \
        -e "s|$TEMPLATE_ROOT|$REPO_ROOT|g" \
        -e "s|^User=root$|User=$SERVICE_USER|" \
        "$source" >"$tmp"
    install -m 0644 "$tmp" "$target"
    rm -f "$tmp"
    printf '  installed %s\n' "$target"
}

install_unit "$PI_DIR/systemd/telekart-control.service" \
             "$SYSTEMD_DIR/telekart-control.service"
install_unit "$PI_DIR/systemd/telekart-video.service" \
             "$SYSTEMD_DIR/telekart-video.service"
install_unit "$PI_DIR/systemd/telekart-wifi-nopowersave.service" \
             "$SYSTEMD_DIR/telekart-wifi-nopowersave.service"

step "pigpiod drop-in"
# The template names /usr/local/bin/pigpiod because that is where a source
# build lands, but Bookworm's own package installs v79 to /usr/bin -- the same
# version, so building from source there buys nothing. Point the unit at
# whichever binary actually exists rather than making the operator notice a
# unit that fails with a bare "No such file or directory".
PIGPIOD_BIN=""
for candidate in /usr/local/bin/pigpiod /usr/bin/pigpiod; do
    [ -x "$candidate" ] && PIGPIOD_BIN="$candidate" && break
done
if [ -n "$PIGPIOD_BIN" ]; then
    run install -D -m 0644 /dev/stdin \
        "$SYSTEMD_DIR/pigpiod.service.d/override.conf" \
        < <(sed "s#^ExecStart=/usr/local/bin/pigpiod#ExecStart=$PIGPIOD_BIN#" \
            "$PI_DIR/systemd/pigpiod.service.d/override.conf")
    done_say "installed $SYSTEMD_DIR/pigpiod.service.d/override.conf (ExecStart=$PIGPIOD_BIN)"
else
    run install -D -m 0644 "$PI_DIR/systemd/pigpiod.service.d/override.conf" \
        "$SYSTEMD_DIR/pigpiod.service.d/override.conf"
    done_say "installed $SYSTEMD_DIR/pigpiod.service.d/override.conf"
fi
if [ ! -f "$SYSTEMD_DIR/pigpiod.service" ] && \
   [ ! -f /lib/systemd/system/pigpiod.service ] && \
   [ ! -f /usr/lib/systemd/system/pigpiod.service ]; then
    warn "there is no pigpiod.service for the drop-in to extend"
    warn "  create it as shown in docs/pi-setup.md section 3"
fi

# ---------------------------------------------------------------------------
# WiFi power save
# ---------------------------------------------------------------------------
step "WiFi power save"
if [ -d /etc/NetworkManager/conf.d ]; then
    if [ "$DRY_RUN" -eq 0 ]; then
        cat >"$NM_CONF" <<'EOF'
# WiFi power save buffers downlink frames at the access point until the next
# beacon: up to 100 ms of added latency, in bursts, landing inside the 200 ms
# CONTROL_TIMEOUT_MS window. It presents as random failsafe trips.
# 2 = disabled. (3 = enabled; 0 = "use the default", which is not what you want.)
[connection]
wifi.powersave = 2
EOF
        chmod 0644 "$NM_CONF"
    fi
    done_say "wrote $NM_CONF"
else
    warn "NetworkManager conf.d not found; relying on the oneshot unit alone"
fi

# ---------------------------------------------------------------------------
# Enable
# ---------------------------------------------------------------------------
step "Enable"
run systemctl daemon-reload

if [ "$DRY_RUN" -eq 0 ] && command -v systemd-analyze >/dev/null 2>&1; then
    if systemd-analyze verify "$SYSTEMD_DIR/telekart-control.service" 2>&1 |
            grep -v 'Unknown key\|Failed to prepare' | grep -q .; then
        warn "systemd-analyze reported problems with telekart-control.service"
    else
        say "unit files verify clean"
    fi
fi

if [ "$DO_ENABLE" -eq 1 ]; then
    run systemctl enable telekart-wifi-nopowersave.service
    run systemctl restart telekart-wifi-nopowersave.service
    run systemctl enable telekart-control.service
    run systemctl enable telekart-video.service
    done_say "enabled telekart-control, telekart-video, telekart-wifi-nopowersave"
    if systemctl list-unit-files 2>/dev/null | grep -q '^pigpiod\.service'; then
        run systemctl enable pigpiod.service
        run systemctl restart pigpiod.service
        done_say "pigpiod enabled and restarted with the TeleKart flags"
    fi
else
    say "skipped (--no-enable)"
fi

# ---------------------------------------------------------------------------
step "Done"
if [ "$WARNINGS" -gt 0 ]; then
    say "$WARNINGS warning(s) above -- read them before bring-up"
fi
cat <<EOF

  Nothing was started. That is deliberate: work through docs/bringup.md, which
  runs each subsystem by hand with a gate in front of it.

  When you are ready:
      sudo systemctl start telekart-control
      journalctl -fu telekart-control

  Quick checks:
      pgrep -a pigpiod                        # the flags from the drop-in
      test -x $PANIC_TARGET && echo panic-stop ok
      $VENV_PY -m telekart --backend mock --duration 3 --no-mdns

EOF
