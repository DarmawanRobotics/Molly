"""Questions from visitors to a local language model through Ollama (no ROS, unit-tested)."""

import json
import re
import urllib.error
import urllib.request

THINK = re.compile(r'<think>.*?</think>', re.DOTALL)
MARKUP = re.compile(r'[*_#`>]+')
BULLET = re.compile(r'(?m)^\s*(?:[-*•+]|\d+[.)])\s+')
SENTENCE_END = re.compile(r'[.!?…](?=\s|$)')

PROMPTS = {
    'id': (
        'Kamu adalah {name}, robot pemandu wisata berkaki empat{site}. Jawab pertanyaan '
        'pengunjung dalam bahasa Indonesia yang ramah dan singkat (paling banyak tiga kalimat), '
        'seperti sedang berbicara langsung: tanpa daftar, tanpa simbol, tanpa emoji. Jika tidak '
        'tahu, katakan terus terang dan sarankan bertanya kepada petugas.'
    ),
    'en': (
        'You are {name}, a four-legged tour guide robot{site}. Answer the visitor in friendly, '
        'short English (three sentences at most), as if speaking out loud: no lists, no symbols, '
        'no emoji. If you do not know, say so and suggest asking the staff.'
    ),
}
KNOWLEDGE = {
    'id': 'Tempat-tempat yang kamu pandu:',
    'en': 'The places you guide visitors to:',
}


def site_knowledge(pois: list, limit: int = 4000) -> str:
    """Return the POIs as short lines for the system prompt (name, description, narration)."""
    lines, total = [], 0
    for poi in pois:
        name = ' '.join(str(poi.get('name', '')).split())
        if not name:
            continue
        details = ' '.join(
            ' '.join(str(poi.get(key, '')).split()) for key in ('description', 'narration')
        ).strip()
        line = f'- {name}: {details}' if details else f'- {name}'
        if total + len(line) + 1 > limit:
            break
        lines.append(line)
        total += len(line) + 1
    return '\n'.join(lines)


def system_prompt(language: str, robot_name: str, site: str, knowledge: str) -> str:
    """Return the system prompt in the language (id or en)."""
    lang = language if language in PROMPTS else 'id'
    where = (f' di {site}' if lang == 'id' else f' at {site}') if site else ''
    prompt = PROMPTS[lang].format(name=robot_name or 'Molly', site=where)
    if knowledge:
        prompt += '\n\n' + KNOWLEDGE[lang] + '\n' + knowledge
    return prompt


def chat_request(model: str, system: str, history: list, question: str,
                 max_tokens: int) -> dict:
    """Return the body of an Ollama /api/chat request (not streamed)."""
    messages = [{'role': 'system', 'content': system}]
    for asked, answered in history:
        messages.append({'role': 'user', 'content': asked})
        messages.append({'role': 'assistant', 'content': answered})
    messages.append({'role': 'user', 'content': question})
    return {
        'model': model,
        'messages': messages,
        'stream': False,
        'options': {'num_predict': int(max_tokens), 'temperature': 0.4},
    }


def parse_chat_response(body: bytes) -> str:
    """Return the answer of an Ollama /api/chat response; ValueError when there is none."""
    data = json.loads(body.decode('utf-8'))
    if not isinstance(data, dict):
        raise ValueError('unexpected answer from the model')
    if data.get('error'):
        raise ValueError(str(data['error']))
    content = (data.get('message') or {}).get('content')
    if not isinstance(content, str):
        raise ValueError('the model answered nothing')
    return content


def clean_answer(text: str, limit: int = 600) -> str:
    """Return a speakable answer: no reasoning block or markup, cut after a sentence."""
    text = THINK.sub(' ', text)
    text = BULLET.sub('', text)
    text = MARKUP.sub('', text)
    text = ' '.join(text.split())
    if len(text) <= limit:
        return text
    cut = text[:limit]
    ends = [m.end() for m in SENTENCE_END.finditer(cut)]
    if ends and ends[-1] > limit // 3:
        return cut[:ends[-1]].strip()
    space = cut.rfind(' ')
    return (cut[:space] if space > 0 else cut).rstrip(' ,;:') + '…'


def ask_ollama(url: str, body: dict, timeout: float) -> str:
    """POST the chat request to Ollama and return the raw answer; raise OSError or ValueError."""
    request = urllib.request.Request(
        url.rstrip('/') + '/api/chat',
        data=json.dumps(body).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return parse_chat_response(response.read())
    except urllib.error.HTTPError as error:
        detail = error.read().decode('utf-8', errors='replace')[:200]
        raise OSError(f'Ollama HTTP {error.code}: {detail}') from error
