import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  test: {
    environment: "jsdom",
    setupFiles: "./src/setupTests.js",
    globals: true,
  },
  server: {
    proxy: {
      // "/products"로 시작하는 요청이 오면 8080번 포트(스프링)로 보낸다!
      "/products": {
        target: "http://localhost:8080",
        changeOrigin: true,
      },
    },
  },
});
