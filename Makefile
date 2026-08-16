# TeleKart v2 task runner.
#
# Two environments, deliberately separate rather than one shared venv:
#
#   app/.venv              Python 3.12  -- PySide6, PyAV, SDL. macOS side.
#   pi/.venv               Python 3.11  -- firmware. Created with
#                                          --system-site-packages so the
#                                          apt-installed picamera2/libcamera
#                                          bindings stay visible.
#
# There is no simulator. To develop with no hardware, run the real firmware on
# this machine -- `make run-car` and `make run-camera` -- which exercises the
# genuine handshake, failsafe schedule and telemetry rather than a second
# implementation of them.
#
# Written for GNU Make 3.81 -- the version macOS ships. No .ONESHELL, no
# $(file ...), no `!=`. Recipes are POSIX sh so they also run under dash on
# Raspberry Pi OS.

.DEFAULT_GOAL := help

REPO      := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
PROTO_DIR := $(REPO)/packages/telekart_protocol
APP_DIR   := $(REPO)/app
UI_DIR    := $(APP_DIR)/telekart_ui
PI_DIR    := $(REPO)/pi

APP_VENV := $(APP_DIR)/.venv
PI_VENV  := $(PI_DIR)/.venv

APP_PY := $(APP_VENV)/bin/python
PI_PY  := $(PI_VENV)/bin/python

# Pass-through knobs:  make run-ui ARGS="--host 192.168.1.50 --fullscreen"
ARGS        ?=
PYTEST_ARGS ?=

# --------------------------------------------------------------------------
# Interpreter discovery
# --------------------------------------------------------------------------
# Recursively expanded (`=`, not `:=`) so that `make setup-ui test` sees the
# environment the earlier target just created instead of a snapshot taken
# before make started.

APP_BOOTSTRAP = $(shell \
	if command -v pyenv >/dev/null 2>&1 && pyenv prefix 3.12 >/dev/null 2>&1; then \
		echo "$$(pyenv prefix 3.12)/bin/python"; \
	elif command -v python3.12 >/dev/null 2>&1; then \
		command -v python3.12; \
	fi)

PI_BOOTSTRAP = $(shell \
	if command -v python3.11 >/dev/null 2>&1; then command -v python3.11; \
	else command -v python3; fi)

# The protocol package is pure stdlib and its suite is meant to be run by BOTH
# halves of the system, so it runs under whichever interpreter already has
# pytest rather than demanding one specific environment.
PYTEST_PY = $(shell \
	for p in "$(APP_PY)" "$(PI_PY)" "$$(command -v python3)"; do \
		if [ -x "$$p" ] && "$$p" -c "import pytest" >/dev/null 2>&1; then \
			echo "$$p"; break; \
		fi; \
	done)

RUFF = $(shell \
	for r in "$(APP_VENV)/bin/ruff" "$(PI_VENV)/bin/ruff" "$$(command -v ruff)"; do \
		if [ -x "$$r" ]; then echo "$$r"; break; fi; \
	done)

UNAME_S := $(shell uname -s)

.PHONY: help setup-ui setup-pi test test-protocol test-pi test-ui \
        lint run-ui run-car run-camera clean clean-venvs

# --------------------------------------------------------------------------

help: ## Show this help
	@echo "TeleKart v2 -- make targets"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[1m%-14s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "  Knobs:  ARGS=\"...\"   extra arguments for run-* targets"
	@echo "          PYTEST_ARGS=\"-k name -x\"   extra arguments for test-* targets"
	@echo ""
	@echo "  No hardware? Run the real firmware locally, in two terminals:"
	@echo "    make run-car      (the 100 Hz loop, on a mock GPIO backend)"
	@echo "    make run-camera   (synthetic H.264 through the real framing)"
	@echo "  then 'make run-ui' in a third. See README.md."

# ------------------------------------------------------------------ setup

setup-ui: ## Create app/.venv (Python 3.12) and install the driving station
	@if [ -z "$(APP_BOOTSTRAP)" ]; then \
		echo "ERROR: no Python 3.12 found."; \
		echo "  The station pins >=3.12,<3.14: PySide6-Essentials 6.11 and"; \
		echo "  pygame-ce need it, and 3.14 wheels are not all published yet."; \
		echo ""; \
		echo "  Install one:   pyenv install 3.12.2"; \
		exit 1; \
	fi
	@echo "==> ui: $(APP_BOOTSTRAP)"
	@"$(APP_BOOTSTRAP)" -m venv "$(APP_VENV)"
	@"$(APP_PY)" -m pip install --quiet --upgrade pip
	@echo "==> ui: installing telekart-protocol (editable, from this checkout)"
	@"$(APP_PY)" -m pip install --quiet -e "$(PROTO_DIR)"
	@echo "==> ui: installing telekart-ui[dev] -- this pulls Qt, expect a minute"
	@"$(APP_PY)" -m pip install -e "$(UI_DIR)[dev]"
	@echo ""
	@echo "Done. 'make run-ui', or 'make run-ui ARGS=\"--host 192.168.1.50\"'."

setup-pi: ## Create pi/.venv and install the firmware (run this ON the Pi)
	@echo "==> pi: $(PI_BOOTSTRAP)"
	@"$(PI_BOOTSTRAP)" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)' \
		|| { echo "ERROR: the firmware needs Python 3.11+ (Bookworm ships it)."; exit 1; }
	@echo "==> pi: creating venv with --system-site-packages"
	@echo "    (picamera2 and libcamera come from apt; a sealed venv cannot see them)"
	@"$(PI_BOOTSTRAP)" -m venv --system-site-packages "$(PI_VENV)"
	@"$(PI_PY)" -m pip install --quiet --upgrade pip
	@"$(PI_PY)" -m pip install --quiet -e "$(PROTO_DIR)"
	@if [ "$(UNAME_S)" = "Linux" ]; then \
		echo "==> pi: installing telekart[rpi,dev]"; \
		"$(PI_PY)" -m pip install -e "$(PI_DIR)[rpi,dev]"; \
	else \
		echo "==> pi: $(UNAME_S) detected, skipping the [rpi] extra"; \
		echo "    (pigpio is a Linux client library; MockBackend covers the rest)"; \
		"$(PI_PY)" -m pip install -e "$(PI_DIR)[dev]"; \
	fi
	@echo ""
	@echo "Done. Before the first real drive, on the Pi:"
	@echo "  sudo apt install -y pigpio python3-picamera2"
	@echo "  sudo systemctl enable --now pigpiod"
	@echo "  echo 'dtparam=audio=off' | sudo tee -a /boot/firmware/config.txt"
	@echo ""
	@echo "That last line is not optional: snd_bcm2835 claims BOTH hardware PWM"
	@echo "channels, and the motors get neither."

# ------------------------------------------------------------------- test

test: test-protocol test-pi test-ui ## Run every suite that has an environment
	@echo "==> all available suites passed"

test-protocol: ## Golden-byte tests for the shared wire protocol
	@if [ -z "$(PYTEST_PY)" ]; then \
		echo "ERROR: no interpreter with pytest. Run 'make setup-ui' or 'make setup-pi'."; \
		exit 1; \
	fi
	@echo "==> protocol: $(PYTEST_PY)"
	@"$(PYTEST_PY)" -m pytest "$(PROTO_DIR)" $(PYTEST_ARGS)

test-pi: ## Firmware tests (MockBackend + FakeClock, no hardware needed)
	@if [ ! -x "$(PI_PY)" ]; then \
		echo "==> pi: no environment yet ('make setup-pi'), skipping"; \
	elif [ ! -d "$(PI_DIR)/tests" ]; then \
		echo "==> pi: no tests/ directory yet, skipping"; \
	else \
		echo "==> pi: $(PI_PY)"; \
		"$(PI_PY)" -m pytest "$(PI_DIR)" $(PYTEST_ARGS); \
	fi

test-ui: ## Driving station tests (pytest-qt; needs a display or offscreen)
	@if [ ! -x "$(APP_PY)" ]; then \
		echo "==> ui: no environment yet ('make setup-ui'), skipping"; \
	elif [ ! -d "$(UI_DIR)/tests" ]; then \
		echo "==> ui: no tests/ directory yet, skipping"; \
	else \
		echo "==> ui: $(APP_PY)"; \
		QT_QPA_PLATFORM=$${QT_QPA_PLATFORM:-offscreen} \
			"$(APP_PY)" -m pytest "$(UI_DIR)" $(PYTEST_ARGS); \
	fi

# ------------------------------------------------------------------- lint

lint: ## ruff if it is installed; a syntax check otherwise
	@if [ -n "$(RUFF)" ]; then \
		echo "==> ruff: $(RUFF)"; \
		"$(RUFF)" check "$(PROTO_DIR)" "$(PI_DIR)" "$(UI_DIR)"; \
		"$(RUFF)" format --check "$(PROTO_DIR)" "$(PI_DIR)" "$(UI_DIR)"; \
	elif [ -n "$(PYTEST_PY)" ]; then \
		echo "==> ruff not installed; falling back to a compile check"; \
		echo "    (pip install ruff in any of the venvs for the real thing)"; \
		"$(PYTEST_PY)" -m compileall -q "$(PROTO_DIR)" "$(PI_DIR)" "$(UI_DIR)"; \
	else \
		echo "ERROR: no interpreter available. Run a setup target first."; \
		exit 1; \
	fi

# -------------------------------------------------------------------- run

run-ui: ## Launch the driving station
	@if [ ! -x "$(APP_PY)" ]; then \
		echo "ERROR: station not installed. Run 'make setup-ui'."; \
		exit 1; \
	fi
	@echo "==> telekart_ui $(ARGS)"
	@"$(APP_PY)" -m telekart_ui $(ARGS)

# The two targets below replace the old simulator. They run the *real* firmware
# on this machine: the same 100 Hz loop, the same handshake, the same failsafe
# ladder and the same telemetry packet, against a mock GPIO backend that carries
# the plant model. Strictly more faithful than a second implementation of the
# protocol -- what it does not give you is fault injection or a seeded, exactly
# repeatable run.

run-car: ## Run the firmware locally on a mock GPIO backend (no hardware)
	@if [ ! -x "$(PI_PY)" ]; then \
		echo "ERROR: firmware not installed. Run 'make setup-pi'."; \
		exit 1; \
	fi
	@echo "==> telekart-control --backend mock $(ARGS)"
	@echo "    UDP 4210/4211 control+telemetry, TCP 4212 session"
	@echo "    passphrase: whatever TELEKART_SHARED_KEY says, else 'change-me'"
	@"$(PI_PY)" -m telekart --backend mock --defaults $(ARGS)

run-camera: ## Serve synthetic H.264 through the real framing (no hardware)
	@if [ ! -x "$(PI_PY)" ]; then \
		echo "ERROR: firmware not installed. Run 'make setup-pi'."; \
		exit 1; \
	fi
	@echo "==> telekart-video --synthetic $(ARGS)"
	@"$(PI_PY)" -m telekart_video --synthetic $(ARGS)

# ------------------------------------------------------------------ clean

clean: ## Remove caches and build artifacts (leaves the venvs alone)
	@echo "==> removing caches and build artifacts"
	@find "$(REPO)" -type d -name '__pycache__' -not -path '*/.venv/*' -prune -exec rm -rf {} + 2>/dev/null || true
	@find "$(REPO)" -type d -name '*.egg-info' -not -path '*/.venv/*' -prune -exec rm -rf {} + 2>/dev/null || true
	@find "$(REPO)" -type d \( -name '.pytest_cache' -o -name '.ruff_cache' -o -name '.mypy_cache' \) \
		-not -path '*/.venv/*' -prune -exec rm -rf {} + 2>/dev/null || true
	@rm -rf "$(PROTO_DIR)/build" "$(PROTO_DIR)/dist" "$(UI_DIR)/build" "$(UI_DIR)/dist" \
		"$(PI_DIR)/build" "$(PI_DIR)/dist"
	@echo "    venvs kept -- 'make clean-venvs' if you really want them gone"

clean-venvs: ## Delete both virtual environments
	@echo "==> removing $(APP_VENV) $(PI_VENV)"
	@rm -rf "$(APP_VENV)" "$(PI_VENV)"
