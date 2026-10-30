import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// `npm run dev` serves the page with hot reload and forwards the API to a running dashboard node.
const backend = process.env.DASHBOARD_URL ?? 'http://127.0.0.1:8080'

export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api': backend,
      '/stream': backend,
      '/ws': { target: backend.replace(/^http/, 'ws'), ws: true },
    },
  },
})
