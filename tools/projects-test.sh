#!/bin/bash
# Saved projects on the nginx image (default.conf, location /wasm/fw/projects/): runs the image with a
# throwaway projects directory and checks what may and may not be written, listed and deleted.
#   tools/projects-test.sh [image]     (default: builds esp32sim:projects-test from this checkout)
set -euo pipefail
cd "$(dirname "$0")/.."
image=${1:-}
if [ -z "$image" ]; then image=esp32sim:projects-test; docker build -q -t "$image" . >/dev/null; fi
dir=$(mktemp -d)
chmod 777 "$dir"   # nginx runs as uid 101 in the container
name=esp32sim-projects-test-$$
docker run -d --rm --name "$name" -p 127.0.0.1::80 -v "$dir":/usr/share/nginx/html/wasm/fw/projects "$image" >/dev/null
cleanup() { docker rm -f "$name" >/dev/null 2>&1 || true; docker run --rm -v "$dir":/d alpine rm -rf /d/demo /d/x >/dev/null 2>&1 || true; rm -rf "$dir"; }
trap cleanup EXIT
base="http://$(docker port "$name" 80/tcp)"
P=$base/wasm/fw/projects
for _ in $(seq 50); do curl -fs -o /dev/null "$base/" && break; sleep 0.1; done

fails=0
expect() {  # WANT DESCRIPTION CURL-ARGS...
    local want=$1 what=$2; shift 2
    local got; got=$(curl -s -o /dev/null -w '%{http_code}' "$@")
    if [ "$got" = "$want" ]; then echo "ok   $what ($got)"; else echo "FAIL $what: want $want, got $got"; fails=$((fails + 1)); fi
}
bin=$(mktemp); head -c 4096 /dev/urandom > "$bin"
big=$(mktemp); head -c $((34 * 1024 * 1024)) /dev/zero > "$big"

expect 200 "empty list" "$P/"
expect 201 "PUT a bin" -T "$bin" "$P/demo/flash.bin"
expect 201 "PUT the manifest" -T - "$P/demo/manifest.json" <<< '{"board":"s31-devkit","files":{"flash":"projects/demo/flash.bin"}}'
expect 204 "PUT over an existing file" -T "$bin" "$P/demo/flash.bin"
list=$(curl -fs "$P/")
[[ $list == *'"name":"demo"'* ]] && echo "ok   list shows the project" || { echo "FAIL list: $list"; fails=$((fails + 1)); }
type=$(curl -fs -o /dev/null -w '%{content_type}' "$P/demo/manifest.json")
[[ $type == application/json* ]] && echo "ok   manifest served as JSON" || { echo "FAIL manifest type $type"; fails=$((fails + 1)); }
nosniff=$(curl -fsI "$P/demo/flash.bin" | tr -d '\r' | grep -ci '^x-content-type-options: nosniff' || true)
[ "$nosniff" = 1 ] && echo "ok   nosniff on stored files" || { echo "FAIL no nosniff header"; fails=$((fails + 1)); }

expect 405 "PUT outside projects (a demo manifest)" -T "$bin" "$base/wasm/fw/hello.json"
expect 405 "PUT traversal ../" --path-as-is -T "$bin" "$P/../hello.json"
expect 405 "PUT traversal %2e%2e" --path-as-is -T "$bin" "$P/%2e%2e/hello.json"
expect 405 "PUT traversal ..%2f" --path-as-is -T "$bin" "$P/demo/..%2f..%2fhello.json"
expect 405 "PUT an unknown file name" -T "$bin" "$P/demo/index.html"
expect 405 "PUT an upper-case project name" -T "$bin" "$P/Demo/flash.bin"
expect 405 "PUT a nested path" -T "$bin" "$P/demo/x/flash.bin"
expect 405 "MOVE" -X MOVE -H "Destination: $P/x/flash.bin" "$P/demo/flash.bin"
expect 405 "MKCOL" -X MKCOL "$P/x/"
expect 405 "OPTIONS preflight gets no CORS answer" -X OPTIONS -H "Origin: http://evil.example" -H "Access-Control-Request-Method: PUT" "$P/demo/flash.bin"
expect 413 "PUT over 33 MB" -T "$big" "$P/demo/flash.bin"

expect 204 "DELETE the project" -X DELETE "$P/demo/"
expect 404 "deleted manifest is gone" "$P/demo/manifest.json"
expect 405 "DELETE the projects root" -X DELETE "$P/"
expect 200 "demo manifests are untouched" "$base/wasm/fw/hello.json"

rm -f "$bin" "$big"
[ "$fails" = 0 ] && echo "projects: all checks passed" || { echo "projects: $fails check(s) failed"; exit 1; }
