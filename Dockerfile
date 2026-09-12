# The planner: a static page that runs PythFinder in the browser.
#
# Deliberately no `# syntax=` line: it pulls a frontend image from Docker Hub
# on every build, and nothing here needs one. The three base images are the
# only things this has to fetch.
#
# Three stages, because the image needs two things built in different
# languages and neither toolchain belongs in what finally runs:
#
#   1. the library, as a wheel small enough to send to a browser
#   2. the page
#   3. nginx, serving both
#
# Build and run it with `docker compose up -d --build`; see compose.yaml.


# --- 1. the library ---------------------------------------------------------
FROM python:3.12-slim AS wheel

RUN pip install --no-cache-dir uv

WORKDIR /src

COPY pyproject.toml README.md LICENSE.txt MANIFEST.in setup.py ./
COPY pythfinder/ pythfinder/
COPY tools/ tools/

# The built wheel is 13.5MB, almost all of it menu images the planner never
# opens; slim_wheel.py takes it down to about 0.11MB. The name is fixed here
# so the page can fetch it without knowing the version.
RUN uv build --wheel --out-dir /tmp/dist \
 && python tools/slim_wheel.py /tmp/dist/*.whl --out-dir /tmp/slim \
 && cp /tmp/slim/*.whl /tmp/pythfinder.whl


# --- 2. the page ------------------------------------------------------------
FROM node:22-alpine AS site

WORKDIR /web

COPY web/package.json web/package-lock.json ./
RUN npm ci

COPY web/ ./

# The mat and the robot. They live with the library, and the wheel the browser
# gets has them stripped out, so the page carries its own copies.
COPY pythfinder/Images/Field/FLL_table_BG.png public/field/mat.png
COPY pythfinder/Images/Robot/fll_robot_team.png public/field/robot.png

# prebuild copies the Pyodide runtime out of node_modules into public/
RUN npm run build


# --- 3. what actually runs --------------------------------------------------
FROM nginx:alpine

COPY --from=site /web/dist/ /usr/share/nginx/html/
COPY --from=wheel /tmp/pythfinder.whl /usr/share/nginx/html/pythfinder.whl
COPY web/nginx.conf /etc/nginx/conf.d/default.conf

EXPOSE 80
