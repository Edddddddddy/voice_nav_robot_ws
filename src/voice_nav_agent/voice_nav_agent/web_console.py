"""Small same-origin HTTP boundary for the rapid robot console."""

import json
import mimetypes
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlsplit


_STATIC_ROUTES = {
    '/': 'index.html',
    '/app.css': 'app.css',
    '/app.js': 'app.js',
}


class ConsoleApi:
    """Validate HTTP input and delegate robot effects to one narrow port."""

    def __init__(self, port, static_root):
        """Retain the effect port and installed static asset directory."""
        self.port = port
        self.static_root = Path(static_root)

    def dispatch(self, method, target, body=b''):
        """Return an HTTP status, content type, and bytes without a server."""
        path = urlsplit(target).path
        if method == 'GET' and path == '/api/state':
            return self._json(200, self.port.state_snapshot())
        if method == 'GET' and path == '/api/map':
            return self._json(200, self.port.map_snapshot())
        if method == 'GET' and path == '/health':
            return self._json(200, {'status': 'ok'})
        if method == 'GET' and path in _STATIC_ROUTES:
            return self._static(_STATIC_ROUTES[path])
        if method == 'POST' and path == '/api/command':
            text = self._command_text(body)
            return self._json(202, self.port.submit_command(text))
        if method == 'POST' and path == '/api/stop':
            self._empty_object(body)
            return self._json(202, self.port.request_stop())
        return self._json(404, {'error': 'not_found'})

    @staticmethod
    def _json(status, value):
        encoded = json.dumps(
            value, ensure_ascii=False, separators=(',', ':')
        ).encode()
        return status, 'application/json; charset=utf-8', encoded

    def _static(self, name):
        path = self.static_root / name
        if not path.is_file():
            return self._json(404, {'error': 'asset_not_found'})
        content_type = mimetypes.guess_type(name)[0] or (
            'application/octet-stream'
        )
        if content_type.startswith('text/') or name.endswith('.js'):
            content_type += '; charset=utf-8'
        return 200, content_type, path.read_bytes()

    @classmethod
    def _command_text(cls, body):
        payload = cls._payload(body)
        if set(payload) != {'text'} or not isinstance(payload['text'], str):
            raise ValueError('command requires exactly one text field')
        text = payload['text'].strip()
        if not 1 <= len(text) <= 512 or any(ord(char) < 32 for char in text):
            raise ValueError('command text must contain 1..512 visible chars')
        return text

    @classmethod
    def _empty_object(cls, body):
        if cls._payload(body):
            raise ValueError('stop body must be an empty object')

    @staticmethod
    def _payload(body):
        if len(body) > 4096:
            raise ValueError('request body is too large')
        try:
            value = json.loads(body.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError('request body must be UTF-8 JSON') from error
        if not isinstance(value, dict):
            raise ValueError('request body must be an object')
        return value


def handler_for(api):
    """Create a standard-library request handler around a ConsoleApi."""

    class ConsoleHandler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            self._dispatch('GET')

        def do_POST(self):  # noqa: N802
            length = self.headers.get('Content-Length', '0')
            try:
                size = int(length)
            except ValueError:
                self._send(*api._json(400, {'error': 'invalid_length'}))
                return
            if size < 0 or size > 4096:
                self._send(*api._json(413, {'error': 'body_too_large'}))
                return
            self._dispatch('POST', self.rfile.read(size))

        def _dispatch(self, method, body=b''):
            try:
                response = api.dispatch(method, self.path, body)
            except ValueError as error:
                response = api._json(400, {'error': str(error)})
            self._send(*response)

        def _send(self, status, content_type, body):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format, *_args):
            return

    return ConsoleHandler
