PY      := .venv/bin/python
PIP     := .venv/bin/pip
PI_HOST := telekart@telekart.local
PI_DIR  := ~/telekart2

.DEFAULT_GOAL := help

.PHONY: help venv sim drive check-protocol clean deploy pull-pi

help:
	@echo "TeleKart2 v1"
	@echo ""
	@echo "  make venv            create the Mac venv and install deps"
	@echo "  make sim             run the simulated Pi (camera + protocol, on this Mac)"
	@echo "  make drive           run the pygame driving app  (targets the simulator)"
	@echo "  make check-protocol  verify pi/ copies match common/"
	@echo "  make clean           remove logs and __pycache__"
	@echo ""
	@echo "  The car (runs as the telekart2 service, enabled at boot):"
	@echo "  make drive-car       drive the real car at telekart.local"
	@echo "  make deploy          rsync pi/ to the Pi and restart the service"
	@echo "  make pi-logs         follow the car's journal"
	@echo "  make pi-status       service state + camera health"
	@echo "  make pi-restart      restart the service"
	@echo "  make pull-pi         snapshot the Pi's copy back into this repo"

venv:
	python3 -m venv .venv
	$(PIP) install --quiet --upgrade pip
	$(PIP) install --quiet -r mac/requirements.txt
	@echo "venv ready"

sim: venv-check
	$(PY) sim/fake_pi.py

# Targets the simulator on localhost. Reaching the actual car requires typing its name,
# because the Pi runs other software and is off-limits until an approved bring-up.
drive: venv-check
	$(PY) mac/main.py

# Bring-up only. Deliberately not a bare `make drive` away.
drive-car: venv-check
	@echo "This addresses the REAL CAR at telekart.local, not the simulator."
	@echo "The Pi runs other software -- read docs/PI_SETUP.md. Ctrl-C to abort."; sleep 5
	TELEKART_HOST=telekart.local $(PY) mac/main.py

venv-check:
	@test -x $(PY) || { echo "No venv. Run: make venv"; exit 1; }

# pi/ holds copies rather than imports, because the Pi is a separate machine with its
# own install. This target is what stops them drifting apart unnoticed.
check-protocol:
	@diff -q common/protocol.py pi/protocol.py >/dev/null \
	  && diff -q common/vehicle.py  pi/vehicle.py  >/dev/null \
	  && echo "protocol.py and vehicle.py match" \
	  || { echo "DRIFT: pi/ copies differ from common/. Re-copy:"; \
	       echo "  cp common/protocol.py pi/protocol.py"; \
	       echo "  cp common/vehicle.py  pi/vehicle.py"; exit 1; }

clean:
	rm -rf logs/*.jsonl
	find . -name __pycache__ -type d -prune -exec rm -rf {} +

# --- Bring-up only ----------------------------------------------------------
# Read docs/PI_SETUP.md before either of these. The Pi runs other software; nothing
# here is safe to run until that has been deliberately stopped.

deploy: check-protocol
	rsync -av --exclude='__pycache__' --exclude='logs' --exclude='.venv' pi/ $(PI_HOST):$(PI_DIR)/
	ssh $(PI_HOST) 'sudo systemctl restart telekart2'
	@echo "deployed and restarted"

# The car runs as a service now; these are the everyday handles.
pi-logs:
	ssh -t $(PI_HOST) 'journalctl -u telekart2 -f'

pi-status:
	ssh $(PI_HOST) 'systemctl status telekart2 --no-pager -n 5; curl -s localhost:8090/health'

pi-restart:
	ssh $(PI_HOST) 'sudo systemctl restart telekart2 && sleep 3 && systemctl is-active telekart2'

pull-pi:
	rsync -av --exclude='__pycache__' --exclude='logs' --exclude='.venv' $(PI_HOST):$(PI_DIR)/ pi/
