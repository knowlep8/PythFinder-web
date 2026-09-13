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

If that reports `unknown shorthand flag: 'd'`, the Docker CLI has no compose
plugin and is reading `-d` as its own flag. Either use the standalone binary:

```bash
docker-compose up -d --build
```

or point the CLI at the plugin Homebrew already installed, once:

```bash
mkdir -p ~/.docker/cli-plugins
ln -sfn /opt/homebrew/lib/docker/cli-plugins/docker-compose ~/.docker/cli-plugins/
```

Both need a running engine — Docker Desktop, colima, or the daemon on whichever
host is serving this. A CLI on its own cannot build anything.

A related trap on macOS: if the build stops with

```
error getting credentials - err: exec: "docker-credential-desktop":
executable file not found in $PATH
```

then `~/.docker/config.json` asks for Docker Desktop's credential helper while
the CLI in use is Homebrew's, which cannot see it. Desktop keeps its binaries
outside the normal path, so add them to it:

```fish
fish_add_path /Applications/Docker.app/Contents/Resources/bin
```

That supplies the helper *and* the compose plugin, so `docker compose` starts
working too. The alternative is to delete the `"credsStore": "desktop"` line
from `~/.docker/config.json`: these images are all public, so nothing here
needs a credential helper at all.

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

## Deploying it

The container above is for local dev and testing. The real deploy is
Cloudflare Pages: `.github/workflows/deploy-pages.yaml` builds the wheel and
the site and runs `wrangler pages deploy` on every push to `main`. See the
"Run self-hosted, or as a static Pages deploy?" row in `docs/web-planner.md`
for why, and the workflow file for the two repository secrets it needs.

Two things only have to be done once, by hand, in the Cloudflare dashboard,
because nothing here can create them on your behalf:

- the Pages project itself, named `pythfinder-planner`;
- the KV namespace `wrangler.toml` binds as `RUNS` -- run
  `npx wrangler kv namespace create RUNS` from this directory and paste the
  id it prints into `wrangler.toml`. Until that id is real, saving and
  opening runs from another laptop fails closed: the planner still works,
  it just falls back to browser-only storage (see `src/store.ts`).

## Saved runs across laptops

`functions/api/runs/` is a small set of Cloudflare Pages Functions --
`GET`/`PUT`/`DELETE /api/runs/<owner>/<name>`, plus `GET /api/runs/all` for
every owner's runs at once. `<owner>` is whatever name someone typed into the
page, not an authenticated identity: this site has no login (see
`docs/web-planner.md`), so the point is keeping people from typing over each
other's runs by accident, not keeping anyone out.

These Functions do not run under plain `vite dev` -- Vite only serves the
static site. To exercise the API locally, build the site once and run it
through Wrangler instead:

```bash
npm run build
npx wrangler pages dev dist
```
