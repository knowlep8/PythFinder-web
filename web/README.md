# The planner

A page that plans an FLL run and hands back the file the hub runs. All the
motion planning is the `pythfinder` library itself, running in the browser
through Pyodide -- there is no second implementation to keep in step.

See `docs/web-planner.md` for the plan this is being built against.

## Running it

The container is the real thing, and the one the team will use:

```bash
docker compose up -d --build      # from the repository root
open http://127.0.0.1:8080
```

## Working on it

Vite's dev server is much quicker to iterate with, but it needs the two build
artefacts the container would otherwise provide:

```bash
cd web
npm install                       # also copies Pyodide into public/
npm run dev
```

and, from the repository root, the library as a wheel:

```bash
uv build --wheel --out-dir dist
python tools/slim_wheel.py dist/pythfinder-*.whl --out-dir web/public
mv web/public/pythfinder-*.whl web/public/pythfinder.whl
```

Rebuild that wheel whenever the Python side changes -- the page fetches it
once at start-up.

## How the page gets the library

A wheel is a zip, and this one has no dependencies, so the page fetches
`/pythfinder.whl` and unpacks it straight into Pyodide's filesystem. That
avoids serving a package installer as well, which is the only reason the
container needs nothing from a CDN.
