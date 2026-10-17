from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import threading

from molly_interaction.conversation import (
    ask_ollama,
    chat_request,
    clean_answer,
    parse_chat_response,
    site_knowledge,
    system_prompt,
)
import pytest


def test_site_knowledge_lines_and_limit():
    """One line per named POI with its description and narration, cut at the limit."""
    pois = [
        {'name': 'Lobby', 'description': 'Main entrance', 'narration': 'Welcome  here.'},
        {'name': '', 'description': 'nameless'},
        {'name': 'Cafe'},
    ]
    assert site_knowledge(pois) == '- Lobby: Main entrance Welcome here.\n- Cafe'
    assert site_knowledge(pois, limit=10) == ''


def test_system_prompt_languages_and_site():
    """The prompt is Indonesian by default, English on request, with the site and knowledge."""
    indonesian = system_prompt('id', 'Molly', 'Museum Kota', '- Lobby')
    assert indonesian.startswith('Kamu adalah Molly') and 'di Museum Kota' in indonesian
    assert indonesian.endswith('- Lobby')
    english = system_prompt('en', 'Molly', '', '')
    assert english.startswith('You are Molly, a four-legged tour guide robot.')
    assert system_prompt('fr', '', '', '').startswith('Kamu adalah Molly')


def test_chat_request_with_history():
    """System first, then earlier turns, then the question; not streamed."""
    body = chat_request('m', 'sys', [('q1', 'a1')], 'q2', 50)
    assert [m['role'] for m in body['messages']] == ['system', 'user', 'assistant', 'user']
    assert body['messages'][-1]['content'] == 'q2'
    assert body['stream'] is False and body['options']['num_predict'] == 50


def test_parse_chat_response():
    """The message content is returned; errors and empty answers raise ValueError."""
    ok = json.dumps({'message': {'role': 'assistant', 'content': 'Halo'}}).encode()
    assert parse_chat_response(ok) == 'Halo'
    for bad in (b'{"error": "model not found"}', b'{"message": {}}', b'[]', b'not json'):
        with pytest.raises(ValueError):
            parse_chat_response(bad)


def test_clean_answer_is_speakable():
    """Reasoning blocks and markup go, long answers end after a sentence."""
    raw = '<think>hmm\nlet me see</think>**Lobi** ada di `lantai 1`.\n\n- Buka pukul 9.'
    assert clean_answer(raw) == 'Lobi ada di lantai 1. Buka pukul 9.'
    long = 'Kalimat pertama yang cukup panjang untuk diuji. ' + 'kata ' * 200
    assert clean_answer(long, 100) == 'Kalimat pertama yang cukup panjang untuk diuji.'
    words = 'kata ' * 200
    assert clean_answer(words, 50).endswith('…') and len(clean_answer(words, 50)) <= 51


@pytest.fixture
def ollama():
    """Serve a fake Ollama /api/chat that echoes the question and records the request."""
    seen = []

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):  # noqa: N802
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            seen.append((self.path, body))
            if body['model'] == 'missing':
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b'{"error": "model not found"}')
                return
            answer = {'message': {'role': 'assistant',
                                  'content': 'Jawab: ' + body['messages'][-1]['content']}}
            data = json.dumps(answer).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *args):
            pass

    server = HTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f'http://127.0.0.1:{server.server_address[1]}', seen
    server.shutdown()
    server.server_close()


def test_ask_ollama_round_trip(ollama):
    """The request reaches /api/chat and the answer comes back; HTTP errors are OSError."""
    url, seen = ollama
    body = chat_request('qwen', 'sys', [], 'Di mana kafe?', 50)
    assert ask_ollama(url, body, 5.0) == 'Jawab: Di mana kafe?'
    assert seen[0][0] == '/api/chat' and seen[0][1]['model'] == 'qwen'
    with pytest.raises(OSError, match='404'):
        ask_ollama(url, chat_request('missing', 's', [], 'q', 5), 5.0)
    with pytest.raises(OSError):
        ask_ollama('http://127.0.0.1:9', body, 1.0)
