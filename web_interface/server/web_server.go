package main

import (
	"fmt"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

func main() {
	// Get port from command line args or default to 8080
	port := 8080
	if len(os.Args) > 1 {
		if p, err := strconv.Atoi(os.Args[1]); err == nil {
			port = p
		}
	}

	// Find frontend directory with multiple fallback strategies
	var webDir string
	var err error
	
	// Strategy 1: Relative to executable (if built in server/)
	execPath, err := os.Executable()
	if err == nil {
		webDir = filepath.Join(filepath.Dir(execPath), "..", "frontend")
		if _, err := os.Stat(webDir); err == nil {
			log.Printf("Using frontend directory relative to executable: %s", webDir)
		} else {
			webDir = ""
		}
	}
	
	// Strategy 2: Relative to current directory
	if webDir == "" {
		candidates := []string{
			"../frontend",           // From server/
			"./frontend",            // From web_interface/
			"./web_interface/frontend", // From project root
		}
		
		for _, candidate := range candidates {
			if _, err := os.Stat(candidate); err == nil {
				webDir = candidate
				log.Printf("Using frontend directory: %s", webDir)
				break
			}
		}
	}
	
	// Final check
	if webDir == "" {
		log.Fatal("Could not find frontend directory. Please run from web_interface/server/, web_interface/, or project root")
	}

	// Create file server
	fs := http.FileServer(http.Dir(webDir))
	
	// Add proper headers for PWA
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		// Set MIME types for PWA files
		if strings.HasSuffix(r.URL.Path, ".js") {
			w.Header().Set("Content-Type", "application/javascript")
		} else if strings.HasSuffix(r.URL.Path, ".css") {
			w.Header().Set("Content-Type", "text/css")
		} else if strings.HasSuffix(r.URL.Path, ".json") {
			w.Header().Set("Content-Type", "application/json")
		} else if strings.HasSuffix(r.URL.Path, ".png") {
			w.Header().Set("Content-Type", "image/png")
		} else if strings.HasSuffix(r.URL.Path, ".ico") {
			w.Header().Set("Content-Type", "image/x-icon")
		}
		
		// Add PWA headers
		w.Header().Set("Cache-Control", "public, max-age=31536000")
		if strings.HasSuffix(r.URL.Path, ".html") || r.URL.Path == "/" {
			w.Header().Set("Cache-Control", "no-cache")
		}
		
		// Add security headers
		w.Header().Set("X-Content-Type-Options", "nosniff")
		w.Header().Set("X-Frame-Options", "DENY")
		w.Header().Set("X-XSS-Protection", "1; mode=block")
		
		// Serve the file
		fs.ServeHTTP(w, r)
	})

	// Setup routes
	http.Handle("/", handler)
	
	// Add health check endpoint
	http.HandleFunc("/health", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusOK)
		fmt.Fprintln(w, `{"status":"ok","service":"evabot-web-server"}`)
	})

	addr := fmt.Sprintf(":%d", port)
	log.Printf("🌐 Evabot Web Server starting on http://localhost%s", addr)
	log.Printf("📁 Serving files from: %s", webDir)
	log.Printf("🏥 Health check: http://localhost%s/health", addr)
	
	if err := http.ListenAndServe(addr, nil); err != nil {
		log.Fatal("Server failed to start:", err)
	}
}
