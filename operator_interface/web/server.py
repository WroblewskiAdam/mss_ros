#!/usr/bin/env python3
"""
MSS Web Interface Server z API do obsługi plików logów
"""

import http.server
import socketserver
import json
import os
import urllib.parse
from pathlib import Path
import mimetypes
from datetime import datetime
import csv

class MSSHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Rozszerzony handler HTTP z API dla plików logów"""
    
    def __init__(self, *args, **kwargs):
        # Ścieżka do katalogu z logami
        self.logs_dir = Path(__file__).parent.parent.parent / 'logs_mss'
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        """Obsługa żądań GET"""
        parsed_path = urllib.parse.urlparse(self.path)
        path = parsed_path.path
        
        # API endpointy
        if path == '/api/logs':
            self.handle_logs_list()
        elif path.startswith('/api/logs/download/'):
            filename = urllib.parse.unquote(path.split('/')[-1])
            self.handle_logs_download(filename)
        else:
            # Standardowa obsługa plików statycznych
            super().do_GET()
    
    def do_POST(self):
        """Obsługa żądań POST"""
        parsed_path = urllib.parse.urlparse(self.path)
        path = parsed_path.path
        
        if path.startswith('/api/logs/rename/'):
            filename = urllib.parse.unquote(path.split('/')[-1])
            self.handle_logs_rename(filename)
        else:
            self.send_error(404, "Not Found")
    
    def do_DELETE(self):
        """Obsługa żądań DELETE"""
        parsed_path = urllib.parse.urlparse(self.path)
        path = parsed_path.path
        
        if path.startswith('/api/logs/delete/'):
            filename = urllib.parse.unquote(path.split('/')[-1])
            self.handle_logs_delete(filename)
        else:
            self.send_error(404, "Not Found")
    
    def handle_logs_list(self):
        """Zwraca listę plików logów"""
        try:
            if not self.logs_dir.exists():
                self.send_json_response([])
                return
            
            logs = []
            for file_path in self.logs_dir.glob('*.csv'):
                if file_path.is_file():
                    stat = file_path.stat()
                    records = self.count_csv_records(file_path)
                    
                    logs.append({
                        'filename': file_path.name,
                        'size': stat.st_size,
                        'modified': datetime.fromtimestamp(stat.st_mtime).isoformat(),
                        'records': records
                    })
            
            self.send_json_response(logs)
            
        except Exception as e:
            self.send_error(500, f"Internal Server Error: {str(e)}")
    
    def handle_logs_download(self, filename):
        """Pobiera plik logów"""
        try:
            # Bezpieczeństwo - sprawdź czy plik jest w katalogu logów
            file_path = self.logs_dir / filename
            if not file_path.exists() or not file_path.is_file():
                self.send_error(404, "File not found")
                return
            
            # Sprawdź czy to plik CSV
            if not filename.endswith('.csv'):
                self.send_error(400, "Invalid file type")
                return
            
            # Wyślij plik
            self.send_file_response(file_path)
            
        except Exception as e:
            self.send_error(500, f"Internal Server Error: {str(e)}")
    
    def handle_logs_rename(self, filename):
        """Zmienia nazwę pliku logów"""
        try:
            # Bezpieczeństwo - sprawdź czy plik jest w katalogu logów
            file_path = self.logs_dir / filename
            if not file_path.exists() or not file_path.is_file():
                self.send_error(404, "File not found")
                return
            
            # Sprawdź czy to plik CSV
            if not filename.endswith('.csv'):
                self.send_error(400, "Invalid file type")
                return
            
            # Odczytaj dane z żądania
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "No data provided")
                return
            
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
            except json.JSONDecodeError:
                self.send_error(400, "Invalid JSON")
                return
            
            new_name = data.get('new_name')
            if not new_name:
                self.send_error(400, "No new name provided")
                return
            
            # Sprawdź czy nowa nazwa jest bezpieczna
            if not new_name.endswith('.csv'):
                self.send_error(400, "New name must end with .csv")
                return
            
            # Sprawdź czy nowa nazwa nie zawiera niebezpiecznych znaków
            if '/' in new_name or '\\' in new_name or '..' in new_name:
                self.send_error(400, "Invalid characters in new name")
                return
            
            new_file_path = self.logs_dir / new_name
            
            # Sprawdź czy plik o nowej nazwie już istnieje
            if new_file_path.exists():
                self.send_error(409, "File with new name already exists")
                return
            
            # Zmień nazwę pliku
            file_path.rename(new_file_path)
            
            self.send_json_response({'success': True, 'message': f'File renamed from {filename} to {new_name}'})
            
        except Exception as e:
            self.send_error(500, f"Internal Server Error: {str(e)}")
    
    def handle_logs_delete(self, filename):
        """Usuwa plik logów"""
        try:
            # Bezpieczeństwo - sprawdź czy plik jest w katalogu logów
            file_path = self.logs_dir / filename
            if not file_path.exists() or not file_path.is_file():
                self.send_error(404, "File not found")
                return
            
            # Sprawdź czy to plik CSV
            if not filename.endswith('.csv'):
                self.send_error(400, "Invalid file type")
                return
            
            # Usuń plik
            file_path.unlink()
            
            self.send_json_response({'success': True, 'message': f'File {filename} deleted'})
            
        except Exception as e:
            self.send_error(500, f"Internal Server Error: {str(e)}")
    
    def count_csv_records(self, file_path):
        """Liczy liczbę rekordów w pliku CSV"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                # Pomiń nagłówek
                next(reader, None)
                return sum(1 for _ in reader)
        except:
            return 0
    
    def send_json_response(self, data):
        """Wysyła odpowiedź JSON"""
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, DELETE, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        
        json_data = json.dumps(data, ensure_ascii=False, indent=2)
        self.wfile.write(json_data.encode('utf-8'))
    
    def send_file_response(self, file_path):
        """Wysyła plik jako odpowiedź"""
        try:
            with open(file_path, 'rb') as f:
                content = f.read()
            
            # Określ typ MIME
            mime_type, _ = mimetypes.guess_type(str(file_path))
            if mime_type is None:
                mime_type = 'application/octet-stream'
            
            self.send_response(200)
            self.send_header('Content-Type', mime_type)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Content-Disposition', f'attachment; filename="{file_path.name}"')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            self.wfile.write(content)
            
        except Exception as e:
            self.send_error(500, f"Error reading file: {str(e)}")
    
    def do_OPTIONS(self):
        """Obsługa żądań OPTIONS (CORS)"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, DELETE, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

def run_server(port=8080):
    """Uruchamia serwer HTTP"""
    with socketserver.TCPServer(("", port), MSSHTTPRequestHandler) as httpd:
        print(f"🌐 MSS Web Interface Server uruchomiony na porcie {port}")
        print(f"📁 Katalog logów: {Path(__file__).parent.parent.parent / 'logs_mss'}")
        print(f"🔗 Web Interface: http://localhost:{port}")
        print(f"📊 API Logów: http://localhost:{port}/api/logs")
        print("💡 Zatrzymaj: Ctrl+C")
        print()
        
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n🛑 Zatrzymywanie serwera...")
            httpd.shutdown()

if __name__ == "__main__":
    import sys
    
    port = 8080
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print("Błąd: Port musi być liczbą")
            sys.exit(1)
    
    run_server(port)
