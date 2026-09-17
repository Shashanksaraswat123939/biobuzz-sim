/**
 * Run a TypeScript tool in Node. Vite is already a dependency and its resolver does the
 * ESM ".js" -> ".ts" mapping that node --experimental-strip-types does not.
 *
 *   npm run tool -- tools/hivedrop.ts --mass 2.4
 */
import { createServer } from 'vite';

const [, , entry, ...args] = process.argv;
if (!entry) {
  console.error('usage: npm run tool -- <tools/x.ts> [args...]');
  process.exit(1);
}

const server = await createServer({ server: { middlewareMode: true }, appType: 'custom', logLevel: 'warn' });
try {
  const mod = await server.ssrLoadModule(entry.startsWith('.') || entry.startsWith('/') ? entry : `./${entry}`);
  if (typeof mod.main !== 'function') throw new Error(`${entry} has no exported main(args)`);
  await mod.main(args);
} finally {
  await server.close();
}
