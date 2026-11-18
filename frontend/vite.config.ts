import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

const proxy = {
  '/api': { target: 'http://127.0.0.1:8000' },
  '/ws': { target: 'ws://127.0.0.1:8000', ws: true },
}

export default defineConfig({
  plugins: [vue()],
  server: { host: '127.0.0.1', port: 5173, strictPort: true, proxy },
  preview: { host: '127.0.0.1', port: 4173, strictPort: true, proxy },
  build: {
    rollupOptions: {
      output: {
        manualChunks(id) {
          if (/[\\/]node_modules[\\/]zrender[\\/]/.test(id)) return 'chart-renderer'
          if (/[\\/]node_modules[\\/]echarts[\\/]/.test(id)) return 'charts'
        },
      },
    },
  },
})
