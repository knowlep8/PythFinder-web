import { defineConfig } from "vite";

export default defineConfig({
  build: {
    // Pyodide needs a modern target anyway, and top-level await is used in
    // main.ts through the dynamic import of the vendored runtime
    target: "es2022",
    outDir: "dist",
    emptyOutDir: true,
  },
  server: {
    port: 5173,
  },
});
