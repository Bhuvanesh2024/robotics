#!/usr/bin/env python3
"""
RoboKinematics Web Server Launcher
Starts a local HTTP server and automatically opens the interactive visualizer in your default web browser.
"""

import http.server
import socketserver
import webbrowser
import os
import sys

PORT = 8000

class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        # Enable caching-free live updates during development
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

def main():
    web_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(web_dir)
    
    # Try preferred port, if occupied find next free port
    port = PORT
    httpd = None
    for p in range(PORT, PORT + 20):
        try:
            httpd = socketserver.TCPServer(("", p), Handler)
            port = p
            break
        except OSError:
            continue
            
    if httpd is None:
        print(f"Error: Unable to bind to ports between {PORT} and {PORT+20}.")
        sys.exit(1)
        
    url = f"http://localhost:{port}/index.html"
    print("=" * 60)
    print("  4-DOF Robot Arm Kinematics Visualizer (FK & IK)")
    print(f"  Server running at: {url}")
    print("  Press Ctrl+C to stop the server.")
    print("=" * 60)
    
    # Open browser automatically
    webbrowser.open(url)
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
        httpd.server_close()

if __name__ == "__main__":
    main()
