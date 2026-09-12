/**
 * Making the planner work with no route to the host.
 *
 * A competition gym is where the chain breaks: the venue's wifi, Cloudflare,
 * the tunnel, the container. A page served from a CDN survives that almost for
 * free; a self-hosted one has to earn it, which is what this file does.
 *
 * Two caches on purpose:
 *
 *   SHELL    the page and its bundle -- small, replaced on every deploy
 *   HEAVY    Pyodide, the wheel, the mat and the robot -- ~13MB, and unchanged
 *            between releases
 *
 * Bumping VERSION throws the shell away and keeps the heavy things, so a
 * deploy costs a few kilobytes rather than another 13MB over school wifi.
 *
 * The rule that matters most here: never store a redirect. With Cloudflare
 * Access in front, an expired session answers a fetch with a login page, and a
 * worker that files that away as if it were the app will serve it back for
 * ever. Only same-origin, non-redirected 200s are kept.
 */

// Bumped when this file changes, so a browser holding the previous worker
// replaces it rather than going on serving the old app.
const VERSION = "v3";
const SHELL = `planner-shell-${VERSION}`;
const HEAVY = "planner-heavy";

/** The page itself. Its bundle is hashed by the build, so it is found by
 *  reading the page rather than named here -- see scriptsIn(). */
const SHELL_FILES = ["/", "/index.html"];

/**
 * The hashed files the page loads, dug out of the HTML during install.
 *
 * They cannot be listed above, because the build renames them on every change.
 * Waiting to cache them when they are requested does not work either: a worker
 * takes control only after the load that started it, so the very first visit
 * never passes its bundle through the worker. Cache the heavy runtime and miss
 * the 21KB of script, and the planner still will not open in a gym.
 */
async function scriptsIn(html) {
  const found = new Set();

  // <script type="module" src="/assets/index-HASH.js">
  for (const [, src] of html.matchAll(/<script[^>]+src="([^"]+)"/g)) {
    found.add(src);
  }

  // the worker chunk is imported by the bundle, not named in the HTML
  for (const [, src] of html.matchAll(/["'](\/assets\/[^"']+\.js)["']/g)) {
    found.add(src);
  }

  return [...found].filter((src) => src.startsWith("/"));
}

/**
 * The big fixed things, fetched during install rather than waited for.
 *
 * Caching these only when they are requested does not work: the page asks for
 * Pyodide, the wheel and the pictures while it is starting, which is before
 * the worker has taken control of the page. Nothing passes through it, the
 * heavy cache stays empty, and the first visitor to lose the host finds the
 * planner will not open -- while testing appears to pass, because the
 * browser's own HTTP cache still holds them.
 */
const HEAVY_FILES = [
  "/pythfinder.whl",
  "/field/mat.png",
  "/field/robot.png",
  "/pyodide/pyodide.mjs",
  "/pyodide/pyodide.asm.js",
  "/pyodide/pyodide.asm.wasm",
  "/pyodide/python_stdlib.zip",
  "/pyodide/pyodide-lock.json",
];

/** Big, and identical for the life of a release. */
const isHeavy = (url) =>
  url.pathname.startsWith("/pyodide/") ||
  url.pathname.startsWith("/field/") ||
  url.pathname === "/pythfinder.whl";

self.addEventListener("install", (event) => {
  event.waitUntil(
    Promise.all([
      caches.open(SHELL).then(async (cache) => {
        await cache.addAll(SHELL_FILES);

        // and whatever that page actually loads, by name-with-hash
        const page = await cache.match("/index.html");
        const html = page ? await page.text() : "";
        let scripts = await scriptsIn(html);

        // the bundle names the worker chunk, so read that too
        for (const src of [...scripts]) {
          const held = await fetch(src).catch(() => null);

          if (held && held.ok) {
            scripts = scripts.concat(await scriptsIn(await held.clone().text()));
          }
        }

        await Promise.all(
          [...new Set(scripts)].map((src) => cache.add(src).catch(() => undefined)),
        );
      }),

      // one at a time, and forgiving: 13MB of Pyodide should not be abandoned
      // wholesale because a single file is missing
      caches.open(HEAVY).then((cache) =>
        Promise.all(
          HEAVY_FILES.map((path) =>
            cache.match(path).then((held) =>
              held ? undefined : cache.add(path).catch(() => undefined),
            ),
          ),
        ),
      ),
    ])
      // a failure here must not leave the old worker in charge for ever
      .catch(() => undefined)
      .then(() => self.skipWaiting()),
  );
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches
      .keys()
      .then((names) =>
        Promise.all(
          names
            .filter((name) => name.startsWith("planner-shell-") && name !== SHELL)
            .map((name) => caches.delete(name)),
        ),
      )
      .then(() => self.clients.claim()),
  );
});

/** Worth keeping? Anything else is served but not stored. */
function keepable(response) {
  return (
    response &&
    response.status === 200 &&
    response.type === "basic" && // same origin; not a redirect or a CORS reply
    !response.redirected
  );
}

self.addEventListener("fetch", (event) => {
  const { request } = event;

  if (request.method !== "GET") {
    return;
  }

  const url = new URL(request.url);

  if (url.origin !== self.location.origin) {
    return;
  }

  const cacheName = isHeavy(url) ? HEAVY : SHELL;

  // The heavy things never change within a release, so answer from the cache
  // and save the wait. Everything else tries the network first, so a deploy is
  // picked up as soon as the host is reachable.
  if (isHeavy(url)) {
    event.respondWith(
      caches.match(request).then(
        (held) =>
          held ??
          fetch(request).then((response) => {
            if (keepable(response)) {
              const copy = response.clone();
              caches.open(cacheName).then((cache) => cache.put(request, copy));
            }

            return response;
          }),
      ),
    );

    return;
  }

  event.respondWith(
    fetch(request)
      .then((response) => {
        if (keepable(response)) {
          const copy = response.clone();
          caches.open(cacheName).then((cache) => cache.put(request, copy));
        }

        return response;
      })
      .catch(() =>
        caches
          .match(request)
          .then((held) => held ?? caches.match("/index.html"))
          .then(
            (held) =>
              held ??
              new Response("The planner is not available offline yet.", {
                status: 503,
                headers: { "Content-Type": "text/plain" },
              }),
          ),
      ),
  );
});
