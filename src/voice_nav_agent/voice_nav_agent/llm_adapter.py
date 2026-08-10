"""Minimal loopback-only llama-server adapter for Agent fallback planning."""

import json
from urllib.parse import urlparse
from urllib.request import ProxyHandler, Request, build_opener


class LoopbackLlm:
    def __init__(self, endpoint, timeout_s=10.0):
        parsed = urlparse(endpoint)
        if parsed.scheme != 'http' or parsed.hostname != '127.0.0.1':
            raise ValueError('LLM endpoint must be loopback HTTP')
        self.endpoint, self.timeout_s = endpoint, timeout_s
        self.opener = build_opener(ProxyHandler({}))

    def plan(self, text, places):
        system = 'Return JSON only: {"kind":"mission","targets":[...]} or {"kind":"reply","text":"..."}. Targets: ' + ', '.join(places)
        payload = json.dumps({'messages': [{'role': 'system', 'content': system}, {'role': 'user', 'content': text}], 'temperature': 0, 'max_tokens': 128, 'stream': False}).encode()
        request = Request(self.endpoint, data=payload, headers={'Content-Type': 'application/json'})
        with self.opener.open(request, timeout=self.timeout_s) as response:
            content = json.load(response)['choices'][0]['message']['content']
        result = json.loads(content)
        if set(result) - {'kind', 'targets', 'text'} or result.get('kind') not in ('mission', 'reply'):
            raise ValueError('LLM response outside closed schema')
        return result
