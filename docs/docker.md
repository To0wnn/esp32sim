# Self-hosting the browser build with Docker

The image is nginx serving `web/`, exactly as GitHub Pages serves it, plus an optional store for
saved firmware projects. Emulation still runs entirely in the visitor's browser.

## Build and run

```sh
tools/wasm-build.sh            # or tools/docker-wasm.sh on a host without a Rust toolchain
tools/fetch-demo-assets.sh     # mask ROMs, xterm.js, MicroPython, the Linux image (pinned, hash-checked)
mkdir -p projects && sudo chown 101:101 projects   # nginx in the image runs as uid 101
docker compose up -d --build   # http://localhost:8080/run.html?wasm
```

`tools/docker-wasm.sh` builds with `rust:1` (latest stable) and keeps the cargo registry in
`~/.cache/esp32sim-cargo`, so rebuilds are incremental. The Dockerfile only copies `default.conf`
and `web/` (`.dockerignore`), so the image holds exactly the files you built and tested.

## Saved projects

With the `projects` volume mounted, the firmware panel in `run.html?wasm` shows **Saved projects**:
pick a board and files, enter a name and press **Save as project**. A project is
`projects/<name>/manifest.json` (the same format as a demo manifest) plus one `<kind>.bin` per file.
**▶ name** opens it through the usual `?fw=` loader, at `run.html?wasm&fw=projects/<name>/manifest`,
and 🗑 deletes it.

nginx's built-in WebDAV stores them (`default.conf`, `location ^~ /wasm/fw/projects/`). It allows
PUT and DELETE only on fixed names, at most 33 MB per file and 20 writes a minute per client. It
sends no CORS headers, so another site cannot write through a visitor's browser. The page detects
the feature with a GET of `wasm/fw/projects/`. A static host (GitHub Pages, `python3 -m http.server`)
answers 404 there and the row stays hidden.

There is no login: anyone who can reach the server can save and delete projects. Put
authentication in front (a reverse proxy) if that matters. `tools/projects-test.sh` checks the write
rules against the image, including path traversal and the size limit.
