SHELL := /bin/bash

ROOT := ../..
PYTHON ?= $(PYO3_PYTHON)
ifeq ($(PYTHON),)
PYTHON := python3
endif
CARGO_BIN ?= $(MAPTRAX_CARGO)
RUSTC_BIN ?= $(MAPTRAX_RUSTC)
ifeq ($(CARGO_BIN),)
CARGO_BIN := cargo
endif
ifeq ($(RUSTC_BIN),)
RUSTC_BIN := rustc
endif
VENV := .venv
VENV_PYTHON := $(CURDIR)/$(VENV)/bin/python

.PHONY: all develop basic turners rerun main main-no-obstacle main-extended clean

all: basic

$(VENV_PYTHON):
	@$(PYTHON) -m venv $(VENV)

develop: $(VENV_PYTHON)
	@cd $(ROOT) && VIRTUAL_ENV=$(CURDIR)/$(VENV) PATH=$(CURDIR)/$(VENV)/bin:$$PATH PYO3_PYTHON=$(VENV_PYTHON) CARGO=$(CARGO_BIN) RUSTC=$(RUSTC_BIN) maturin develop --features python

basic: develop
	@$(VENV_PYTHON) basic.py

turners: develop
	@$(VENV_PYTHON) turners.py

rerun: develop
	@$(VENV_PYTHON) -m pip install rerun-sdk
	@$(VENV_PYTHON) rerun_demo.py

main: develop
	@$(VENV_PYTHON) -m pip install rerun-sdk
	@$(VENV_PYTHON) main.py

main-no-obstacle: develop
	@$(VENV_PYTHON) -m pip install rerun-sdk
	@$(VENV_PYTHON) main_no_obstacle.py

main-extended: develop
	@$(VENV_PYTHON) -m pip install rerun-sdk
	@$(VENV_PYTHON) main_extended.py

clean:
	@rm -rf $(VENV)
	@cd $(ROOT) && cargo clean
