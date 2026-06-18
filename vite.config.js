import { defineConfig } from 'vite';

export default defineConfig({
  base: './',
  build: {
    target: 'es2020',
    outDir: 'dist',
  },
  // sql.js ships a wasm binary; make sure it is treated as an asset.
  assetsInclude: ['**/*.wasm'],
});
