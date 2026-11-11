import { svelte } from '@sveltejs/vite-plugin-svelte';
import { defineConfig } from 'vite';

// `bun run build` writes into the Python package, which serves it (teleop-host).
// `bun run dev` proxies the API and WebSocket to a running `teleop-host --dev`.
export default defineConfig({
  plugins: [svelte()],
  base: './',
  build: {
    outDir: '../src/teleop_host/static',
    emptyOutDir: true,
    chunkSizeWarningLimit: 800,
  },
  server: {
    port: 5173,
    proxy: {
      '/api': 'http://127.0.0.1:8790',
      '/ws': { target: 'ws://127.0.0.1:8790', ws: true },
    },
  },
});
