import { defineConfig } from 'vite';
import { fileURLToPath } from 'node:url';

const r = (p: string) => fileURLToPath(new URL(p, import.meta.url));

// A GitHub project page lives at /<repo>/, not at the domain root, so every asset URL has
// to carry that prefix -- without it the page loads and every script 404s. Only CI sets this,
// so `npm run dev` and `npm run preview` stay at / locally.
const base = process.env.GITHUB_PAGES ? '/biobuzz-simulator/' : '/';

export default defineConfig({
  base,
  resolve: {
    alias: {
      '@core': r('./packages/core/src'),
      '@render': r('./packages/render/src'),
      '@ui': r('./packages/ui/src'),
    },
  },
  // 5180, not Vite's default 5173. Two checkouts of this repo on one machine both want the
  // default, and the second one silently loses the socket to the first -- the dev server
  // reports "ready on 5173" and every request still goes to the other tree. PORT overrides it.
  server: { port: Number(process.env.PORT) || 5180, strictPort: true, open: false },
  build: { target: 'es2022', chunkSizeWarningLimit: 2000 },
});
