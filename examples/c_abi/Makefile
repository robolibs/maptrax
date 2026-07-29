SHELL := /bin/bash

ROOT := ../..
C_DEMO_BIN := $(ROOT)/target/debug/c_api_demo

# Set GEOJSON=1 to build the library with the optional GeoJSON surface and
# compile the demo's export section against it.
GEOJSON ?= 0
ifeq ($(GEOJSON),1)
CARGO_FEATURES := --features geojson
C_DEFINES := -DMAPTRAX_GEOJSON
else
CARGO_FEATURES :=
C_DEFINES :=
endif

.PHONY: all build run clean

all: run

build:
	@cd $(ROOT) && cargo build $(CARGO_FEATURES)
	@cc $(C_DEFINES) -I$(ROOT)/include demo.c -L$(ROOT)/target/debug -lmaptrax -Wl,-rpath,$$(cd $(ROOT) && pwd)/target/debug -lm -o $(C_DEMO_BIN)

# Run from the repo root so relative output paths (target/...) resolve.
run: build
	@cd $(ROOT) && target/debug/c_api_demo

clean:
	@rm -f $(C_DEMO_BIN)
