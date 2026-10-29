"""conversation_node in-process against a fake Ollama and speech service."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import threading

from interaction_rig import Rig
from molly_interaction.conversation_node import ConversationNode
import pytest
from rclpy.parameter import Parameter

from dharma_utils.ros import LATCHED
from dharma_utils.testing import Collector, wait_for
from molly_interfaces.msg import Conversation, Poi, PoiArray
from molly_interfaces.srv import Ask, Say


@pytest.fixture
def ollama():
    """Serve a fake Ollama that knows the cafe when the site's POIs name it."""

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            system = body['messages'][0]['content']
            content = 'Kafe ada di lantai dua.' if 'Kafe' in system else 'Saya tidak tahu.'
            data = json.dumps({'message': {'role': 'assistant', 'content': content}}).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *args):
            pass

    server = HTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{server.server_address[1]}'
    server.shutdown()
    server.server_close()


def test_ask_uses_the_site_and_speaks(ros, ollama):
    node = ConversationNode(parameter_overrides=[Parameter('url', value=ollama)])
    rig = Rig([node], 'conversation_helper')
    said = []

    def on_say(request, response):
        said.append(request.text)
        response.success, response.id = True, 1
        return response

    rig.helper.create_service(Say, 'interaction/say', on_say)
    log = Collector(rig.helper, Conversation, 'interaction/conversation', 10)
    pois = rig.helper.create_publisher(PoiArray, 'site/pois', LATCHED)
    try:
        pois.publish(PoiArray(site='mall', pois=[Poi(id='cafe', name='Kafe', description='lt 2')]))
        assert wait_for(lambda: node.pois, 5.0)
        answer = rig.call(Ask, 'interaction/ask', question='Di mana kafe?', speak=True)
        assert answer.success and answer.answer == 'Kafe ada di lantai dua.'
        assert wait_for(lambda: said == ['Kafe ada di lantai dua.'], 5.0)
        assert wait_for(lambda: log.last is not None and log.last.success, 5.0)
        assert not rig.call(Ask, 'interaction/ask', question='  ').success
    finally:
        rig.close()


def test_unreachable_model(ros):
    node = ConversationNode(
        parameter_overrides=[
            Parameter('url', value='http://127.0.0.1:9'),
            Parameter('timeout_sec', value=2.0),
        ]
    )
    rig = Rig([node], 'conversation_helper_down')
    try:
        answer = rig.call(Ask, 'interaction/ask', question='Halo?')
        assert not answer.success and 'not reachable' in answer.message
    finally:
        rig.close()
