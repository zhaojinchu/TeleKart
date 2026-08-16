# TeleKart v2 task runner.
#
# Three environments, deliberately separate rather than one shared venv:
#
#   app/.venv              Python 3.12  -- PySide6, PyAV, SDL. macOS side.
#   pi/.venv               Python 3.11  -- firmware. Created with
#                                          --system-site-packages so the
#                                          apt-installed picamera2/libcamera
#                                          bindings stay visible.
#   tools/telekart_sim/.venv             -- simulator, so it can be run from a
#                                          terminal next to a live app without
#                                          either one's dependency resolution
#                                          disturbing the other.
#
# Written for GNU Make 3.81 -- the version macOS ships. No .ONESHELL, no
# $(file ...), no `!=`. Recipes are POSIX sh so they also run under dash on
# Raspberry Pi OS.

.DEFAULT_GOAL := help

REPO      := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
PROTO_DIR := $(REPO)/packages/telekart_protocol
APP_DIR   := $(REPO)/app
PI_DIR    := $(REPO)/pi
SIM_DIR   := $(REPO)/tools/telekart_sim

APP_VENV := $(APP_DIR)/.venv
PI_VENV  := $(PI_DIR)/.venv
SIM_VENV := $(SIM_DIR)/.venv

APP_PY := $(APP_VENV)/bin/python
PI_PY  := $(PI_VENV)/bin/python
SIM_PY := $(SIM_VENV)/bin/python

# Pass-through knobs:  make run-sim ARGS="--seed 1 --packet-loss 0.02"
ARGS        ?=
PYTEST_ARGS ?=

# The headless client's module path. Named as a variable because it is the one
# entry point in this file that is a convention rather than a wire-level fact.
CLI_MODULE ?= telekart_app.cli

# --------------------------------------------------------------------------
# Interpreter discovery
# --------------------------------------------------------------------------
# Recursively expanded (`=`, not `:=`) so that `make setup-app test` sees the
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

# The simulator needs a PyAV wheel, so on macOS it wants the same pinned
# interpreter as the app rather than whatever `python3` happens to be.
SIM_BOOTSTRAP = $(if $(APP_BOOTSTRAP),$(APP_BOOTSTRAP),$(PI_BOOTSTRAP))

# The protocol package is pure stdlib and its suite is meant to be run by BOTH
# halves of the system, so it runs under whichever interpreter already has
# pytest rather than demanding one specific environment.
PYTEST_PY = $(shell \
	for p in "$(APP_PY)" "$(PI_PY)" "$(SIM_PY)" "$$(command -v python3)"; do \
		if [ -x "$$p" ] && "$$p" -c "import pytest" >/dev/null 2>&1; then \
			echo "$$p"; break; \
		fi; \
	done)

RUFF = $(shell \
	for r in "$(APP_VENV)/bin/ruff" "$(PI_VENV)/bin/ruff" "$$(command -v ruff)"; do \
		if [ -x "$$r" ]; then echo "$$r"; break; fi; \
	done)

UNAME_S := $(shell uname -s)

.PHONY: help setup-app setup-pi setup-sim test test-protocol test-pi test-app \
        lint run-sim run-app run-cli clean clean-venvs

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
	@echo "  No hardware? 'make setup-app setup-sim', then 'make run-sim' in one"
	@echo "  terminal and 'make run-app' in another. See README.md."

# ------------------------------------------------------------------ setup

setup-app: ## Create app/.venv (Python 3.12) and install the desktop app
	@if [ -z "$(APP_BOOTSTRAP)" ]; then \
		echo "ERROR: no Python 3.12 found."; \
		echo "  The desktop app pins >=3.12,<3.14: PySide6-Essentials 6.11 and"; \
		echo "  pygame-ce need it, and 3.14 wheels are not all published yet."; \
		echo ""; \
		echo "  Install one:   pyenv install 3.12.2"; \
		exit 1; \
	fi
	@echo "==> app: $(APP_BOOTSTRAP)"
	@"$(APP_BOOTSTRAP)" -m venv "$(APP_VENV)"
	@"$(APP_PY)" -m pip install --quiet --upgrade pip
	@echo "==> app: installing telekart-protocol (editable, from this checkout)"
	@"$(APP_PY)" -m pip install --quiet -e "$(PROTO_DIR)"
	@echo "==> app: installing telekart-app[dev] -- this pulls Qt, expect a minute"
	@"$(APP_PY)" -m pip install -e "$(APP_DIR)[dev]"
	@echo ""
	@echo "Done. 'make run-app', or 'make run-app ARGS=--connect 192.168.1.50'."
	@echo "For the track-marker features:  $(APP_PY) -m pip install -e '$(APP_DIR)[vision]'"
	@echo "(that extra must stay the *headless* OpenCV build -- the GUI wheels"
	@echo " link their own Qt, and two Qt stacks in one process crash on macOS.)"

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

setup-sim: ## Create tools/telekart_sim/.venv and install the simulator
	@echo "==> sim: $(SIM_BOOTSTRAP)"
	@"$(SIM_BOOTSTRAP)" -m venv "$(SIM_VENV)"
	@"$(SIM_PY)" -m pip install --quiet --upgrade pip
	@"$(SIM_PY)" -m pip install --quiet -e "$(PROTO_DIR)"
	@echo "==> sim: installing telekart-sim[dev]"
	@"$(SIM_PY)" -m pip install -e "$(SIM_DIR)[dev]"
	@echo ""
	@echo "Done. 'make run-sim' serves a fake car on the real ports."

# ------------------------------------------------------------------- test

test: test-protocol test-pi test-app ## Run every suite that has an environment
	@echo "==> all available suites passed"

test-protocol: ## Golden-byte tests for the shared wire protocol
	@if [ -z "$(PYTEST_PY)" ]; then \
		echo "ERROR: no interpreter with pytest. Run 'make setup-app' or 'make setup-pi'."; \
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

test-app: ## Desktop app tests (pytest-qt; needs a display or offscreen)
	@if [ ! -x "$(APP_PY)" ]; then \
		echo "==> app: no environment yet ('make setup-app'), skipping"; \
	elif [ ! -d "$(APP_DIR)/tests" ]; then \
		echo "==> app: no tests/ directory yet, skipping"; \
	else \
		echo "==> app: $(APP_PY)"; \
		QT_QPA_PLATFORM=$${QT_QPA_PLATFORM:-offscreen} \
			"$(APP_PY)" -m pytest "$(APP_DIR)" $(PYTEST_ARGS); \
	fi

# ------------------------------------------------------------------- lint

lint: ## ruff if it is installed; a syntax check otherwise
	@if [ -n "$(RUFF)" ]; then \
		echo "==> ruff: $(RUFF)"; \
		"$(RUFF)" check "$(PROTO_DIR)" "$(PI_DIR)" "$(APP_DIR)" "$(SIM_DIR)"; \
		"$(RUFF)" format --check "$(PROTO_DIR)" "$(PI_DIR)" "$(APP_DIR)" "$(SIM_DIR)"; \
	elif [ -n "$(PYTEST_PY)" ]; then \
		echo "==> ruff not installed; falling back to a compile check"; \
		echo "    (pip install ruff in any of the venvs for the real thing)"; \
		"$(PYTEST_PY)" -m compileall -q "$(PROTO_DIR)" "$(PI_DIR)" "$(APP_DIR)" "$(SIM_DIR)"; \
	else \
		echo "ERROR: no interpreter available. Run a setup target first."; \
		exit 1; \
	fi

# -------------------------------------------------------------------- run

run-sim: ## Serve a fake car on the real ports (no hardware)
	@if [ ! -x "$(SIM_VENV)/bin/telekart-sim" ]; then \
		echo "ERROR: simulator not installed. Run 'make setup-sim'."; \
		exit 1; \
	fi
	@echo "==> telekart-sim $(ARGS)"
	@echo "    UDP 4210/4211, TCP 4212/4213, advertised over mDNS as _telekart._tcp"
	@"$(SIM_VENV)/bin/telekart-sim" $(ARGS)

run-app: ## Launch the desktop driving station
	@if [ ! -x "$(APP_PY)" ]; then \
		echo "ERROR: app not installed. Run 'make setup-app'."; \
		exit 1; \
	fi
	@echo "==> telekart_app $(ARGS)"
	@"$(APP_PY)" -m telekart_app $(ARGS)

run-cli: ## Headless client -- connect, arm, drive, dump telemetry
	@if [ ! -x "$(APP_PY)" ]; then \
		echo "ERROR: app not installed. Run 'make setup-app'."; \
		exit 1; \
	fi
	@"$(APP_PY)" -c "import importlib.util,sys; sys.exit(0 if importlib.util.find_spec('$(CLI_MODULE)') else 1)" \
		|| { echo "ERROR: module '$(CLI_MODULE)' not found."; \
		     echo "  Override with:  make run-cli CLI_MODULE=some.other.module"; exit 1; }
	@echo "==> $(CLI_MODULE) $(ARGS)"
	@"$(APP_PY)" -m $(CLI_MODULE) $(ARGS)

# ------------------------------------------------------------------ clean

clean: ## Remove caches and build artifacts (leaves the venvs alone)
	@echo "==> removing caches and build artifacts"
	@find "$(REPO)" -type d -name '__pycache__' -not -path '*/.venv/*' -prune -exec rm -rf {} + 2>/dev/null || true
	@find "$(REPO)" -type d -name '*.egg-info' -not -path '*/.venv/*' -prune -exec rm -rf {} + 2>/dev/null || true
	@find "$(REPO)" -type d \( -name '.pytest_cache' -o -name '.ruff_cache' -o -name '.mypy_cache' \) \
		-not -path '*/.venv/*' -prune -exec rm -rf {} + 2>/dev/null || true
	@rm -rf "$(PROTO_DIR)/build" "$(PROTO_DIR)/dist" "$(APP_DIR)/build" "$(APP_DIR)/dist" \
		"$(PI_DIR)/build" "$(PI_DIR)/dist" "$(SIM_DIR)/build" "$(SIM_DIR)/dist"
	@echo "    venvs kept -- 'make clean-venvs' if you really want them gone"

clean-venvs: ## Delete the three virtual environments
	@echo "==> removing $(APP_VENV) $(PI_VENV) $(SIM_VENV)"
	@rm -rf "$(APP_VENV)" "$(PI_VENV)" "$(SIM_VENV)"
