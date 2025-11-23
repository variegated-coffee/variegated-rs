import { defineConfig } from 'vite'
import preact from '@preact/preset-vite'
import viteCompression from 'vite-plugin-compression'
import fs from 'fs'
import path from 'path'
import { fileURLToPath } from 'url'

const __filename = fileURLToPath(import.meta.url)
const __dirname = path.dirname(__filename)

export default defineConfig({
  plugins: [
    preact(),
    viteCompression({
      algorithm: 'brotliCompress',
      ext: '.br',
      threshold: 1024,
      deleteOriginFile: false
    }),
    // Custom plugin to serve mock data
    {
      name: 'mock-data-server',
      configureServer(server) {
        server.middlewares.use((req, res, next) => {
          // Map of API endpoints to mock data files
          const mockDataMap: Record<string, string> = {
            '/status': 'status.json',
            '/configuration': 'configuration.json',
            '/machine-definition': 'machine-definition.json',
            '/routines': 'routines.json'
          }

          // Check if the request matches one of our mock endpoints
          if (req.url && mockDataMap[req.url]) {
            const mockFile = mockDataMap[req.url]
            const mockDataPath = path.resolve(__dirname, 'mock-data', mockFile)

            try {
              const mockData = fs.readFileSync(mockDataPath, 'utf-8')
              res.setHeader('Content-Type', 'application/json')
              res.setHeader('Access-Control-Allow-Origin', '*')
              res.statusCode = 200
              res.end(mockData)
              console.log(`[mock-data] Served ${req.url} from ${mockFile}`)
            } catch (error) {
              console.error(`[mock-data] Error reading ${mockFile}:`, error)
              res.statusCode = 500
              res.end(JSON.stringify({ error: 'Failed to load mock data' }))
            }
          } else {
            next()
          }
        })
      }
    }
  ],
  build: {
    minify: 'terser',
    terserOptions: {
      compress: {
        drop_console: true, // Remove console.* statements in production
        drop_debugger: true,
        pure_funcs: ['console.log', 'console.info', 'console.debug', 'console.warn']
      },
      format: {
        comments: false // Remove all comments
      }
    }
  }
})
