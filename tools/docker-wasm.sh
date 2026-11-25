#!/bin/sh
# Build web/wasm/esp32sim.wasm with tools/wasm-build.sh inside the official Rust image, for hosts
# without a Rust toolchain. rust:1 is the latest stable 1.x, as upstream CI uses `stable`; --pull
# keeps it current, and the wasm32 target lives in a cached layer instead of being fetched per run.
# target/ stays in the checkout and the cargo registry in ~/.cache, so rebuilds are incremental.
#   tools/docker-wasm.sh
set -e
cd "$(dirname "$0")/.."
printf 'FROM rust:1\nRUN rustup target add wasm32-unknown-unknown\n' | docker build -q --pull -t esp32sim-rust - >/dev/null
cache="${XDG_CACHE_HOME:-$HOME/.cache}/esp32sim-cargo"
mkdir -p "$cache"
docker run --rm -u "$(id -u):$(id -g)" -e CARGO_HOME=/cargo -v "$cache":/cargo -v "$PWD":/work -w /work \
  esp32sim-rust sh tools/wasm-build.sh
