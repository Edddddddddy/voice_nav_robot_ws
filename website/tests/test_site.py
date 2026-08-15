from __future__ import annotations

from html.parser import HTMLParser
import json
import os
from pathlib import Path
import re
import unittest
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

import yaml


WEBSITE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = WEBSITE_ROOT.parent
DOCS_ROOT = WEBSITE_ROOT / 'docs'
SITE_ROOT = Path(
    os.environ.get('VOICE_NAV_SITE_DIR', WEBSITE_ROOT / 'site')
).resolve()


class _MkDocsConfigLoader(yaml.SafeLoader):
    """Parse MkDocs callable tags as inert names for source checks."""


_MkDocsConfigLoader.add_multi_constructor(
    'tag:yaml.org,2002:python/name:',
    lambda _loader, suffix, _node: suffix,
)


def _navigation_pages(items: list[object]) -> list[str]:
    pages: list[str] = []
    for item in items:
        if not isinstance(item, dict) or len(item) != 1:
            raise AssertionError(f'invalid navigation item: {item!r}')
        value = next(iter(item.values()))
        if isinstance(value, str):
            pages.append(value)
        elif isinstance(value, list):
            pages.extend(_navigation_pages(value))
        else:
            raise AssertionError(f'invalid navigation value: {value!r}')
    return pages


class _AssetLinkParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.links: list[tuple[str, str]] = []

    def handle_starttag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        attribute = 'href' if tag in {'a', 'link'} else 'src'
        if tag not in {'a', 'link', 'script', 'img'}:
            return
        values = dict(attrs)
        if values.get(attribute):
            self.links.append((tag, values[attribute] or ''))


class SiteSourceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = yaml.load(
            (WEBSITE_ROOT / 'mkdocs.yml').read_text(encoding='utf-8'),
            Loader=_MkDocsConfigLoader,
        )
        cls.pages = _navigation_pages(cls.config['nav'])

    def test_navigation_contains_complete_first_release(self) -> None:
        self.assertGreaterEqual(len(self.pages), 10)
        self.assertEqual(len(self.pages), len(set(self.pages)))
        for page in self.pages:
            self.assertTrue((DOCS_ROOT / page).is_file(), page)

    def test_source_map_covers_navigation_pages(self) -> None:
        source_map_path = WEBSITE_ROOT / 'source-map.json'
        self.assertTrue(source_map_path.is_file(), 'source-map.json is required')
        source_map = json.loads(source_map_path.read_text(encoding='utf-8'))
        self.assertEqual(set(self.pages), set(source_map))
        for page, sources in source_map.items():
            self.assertIsInstance(sources, list, page)
            self.assertGreaterEqual(len(sources), 1, page)
            for source in sources:
                source_path = REPOSITORY_ROOT / source
                self.assertTrue(source_path.is_file(), f'{page}: {source}')

    def test_public_pages_state_their_provenance(self) -> None:
        for page in self.pages:
            if page == 'index.md':
                continue
            content = (DOCS_ROOT / page).read_text(encoding='utf-8')
            self.assertIn('页面事实依据', content, page)

    def test_package_page_matches_workspace_packages(self) -> None:
        package_names = {
            ET.parse(package_xml).getroot().findtext('name')
            for package_xml in (REPOSITORY_ROOT / 'src').glob('*/package.xml')
        }
        package_page = (
            DOCS_ROOT / 'reference' / 'packages.md'
        ).read_text(encoding='utf-8')
        self.assertEqual(6, len(package_names))
        for package_name in package_names:
            self.assertIn(f'## {package_name}', package_page)

    def test_site_copy_separates_verified_target_and_boundary(self) -> None:
        combined = '\n'.join(
            (DOCS_ROOT / page).read_text(encoding='utf-8')
            for page in self.pages
        )
        for label in ('已验证', '目标', '边界', 'Operational Stop'):
            self.assertIn(label, combined)
        self.assertNotRegex(
            combined,
            r'(?i)(password|passwd|secret|token)\s*[:=]\s*\S+',
        )

    def test_public_proxy_owns_only_the_voice_nav_path(self) -> None:
        proxy_config = (
            WEBSITE_ROOT / 'deploy' / 'public-location.conf'
        ).read_text(encoding='utf-8')
        self.assertIn('location = /voice-nav {', proxy_config)
        self.assertIn('location /voice-nav/ {', proxy_config)
        self.assertIn('proxy_pass http://voice-nav-site:80/;', proxy_config)
        self.assertNotIn('location / {', proxy_config)


class DeploymentContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.deployment_root = WEBSITE_ROOT / 'deploy'
        cls.compose = yaml.safe_load(
            (cls.deployment_root / 'compose.yaml').read_text(encoding='utf-8')
        )
        cls.guide = (cls.deployment_root / 'README.md').read_text(
            encoding='utf-8'
        )

    def test_site_can_be_deployed_and_rolled_back_from_versioned_contracts(
        self,
    ) -> None:
        service = self.compose['services']['voice-nav-site']
        self.assertRegex(
            service['image'],
            r'^nginx:[^@]+@sha256:[0-9a-f]{64}$',
        )
        self.assertEqual(['127.0.0.1:18081:80'], service['ports'])

        mounts = {mount['target']: mount for mount in service['volumes']}
        self.assertTrue(mounts['/srv/voice-nav']['read_only'])
        self.assertEqual(
            '/opt/apps/voice_nav_site',
            mounts['/srv/voice-nav']['source'],
        )
        self.assertTrue(
            mounts['/etc/nginx/conf.d/default.conf']['read_only']
        )
        self.assertEqual(
            '/opt/apps/voice_nav_site/nginx/default.conf',
            mounts['/etc/nginx/conf.d/default.conf']['source'],
        )

        network = self.compose['networks']['voice-nav-public']
        self.assertTrue(network['external'])
        self.assertEqual('voice-nav-public', network['name'])
        self.assertIn(
            'voice-nav-site',
            service['networks']['voice-nav-public']['aliases'],
        )

        command_blocks = '\n'.join(
            re.findall(
                r'^```bash\n(.*?)^```$',
                self.guide,
                re.MULTILINE | re.DOTALL,
            )
        )
        for command in (
            'docker network create voice-nav-public',
            'public_proxy=<public-proxy-container>',
            'docker network connect voice-nav-public "$public_proxy"',
            'public-location.conf',
            'include /etc/nginx/snippets/voice-nav-site.conf;',
            '/etc/nginx/snippets/voice-nav-site.conf',
            'docker cp "$proxy_config_source" "$public_proxy:/etc/nginx/conf.d/default.conf"',
            'docker exec "$public_proxy" nginx -t',
            'docker exec "$public_proxy" nginx -s reload',
            'docker compose -f compose.yaml up -d',
            'nginx -t',
            'ln -sfn "releases/$release" current.next',
            'mv -Tf current.next current',
            'http://127.0.0.1:18081/healthz',
            'previous=/opt/apps/voice_nav_site/releases/<previous-exact-head>',
            'ln -sfn "$previous_relative" current.next',
        ):
            self.assertIn(command, command_blocks)
        self.assertNotIn('sudo nginx -t', command_blocks)
        self.assertNotIn('systemctl reload nginx', command_blocks)


class BuiltSiteContractTest(unittest.TestCase):
    def test_routes_search_and_social_asset_exist(self) -> None:
        expected = (
            SITE_ROOT / 'index.html',
            SITE_ROOT / 'quick-start' / 'index.html',
            SITE_ROOT / 'architecture' / 'motion-safety' / 'index.html',
            SITE_ROOT / 'reference' / 'packages' / 'index.html',
            SITE_ROOT / 'search' / 'search_index.json',
            SITE_ROOT / 'assets' / 'images' / 'og.png',
        )
        for path in expected:
            self.assertTrue(path.is_file(), path)

        search_index = json.loads(
            (SITE_ROOT / 'search' / 'search_index.json').read_text(
                encoding='utf-8'
            )
        )
        titles = {document['title'] for document in search_index['docs']}
        self.assertIn('首页', titles)
        self.assertIn('运动安全链', titles)
        self.assertIn('公共 ROS 接口', titles)

        home = (SITE_ROOT / 'index.html').read_text(encoding='utf-8')
        self.assertIn('让一句中文指令，变成', home)
        self.assertIn(
            'http://47.116.57.154/voice-nav/assets/images/og.png', home
        )

    def test_all_generated_local_links_resolve(self) -> None:
        broken: list[str] = []
        config = yaml.load(
            (WEBSITE_ROOT / 'mkdocs.yml').read_text(encoding='utf-8'),
            Loader=_MkDocsConfigLoader,
        )
        site_path = urlsplit(config['site_url']).path.rstrip('/')
        for html_path in SITE_ROOT.rglob('*.html'):
            parser = _AssetLinkParser()
            parser.feed(html_path.read_text(encoding='utf-8'))
            for _tag, raw_url in parser.links:
                parsed = urlsplit(raw_url)
                if (
                    parsed.scheme
                    or parsed.netloc
                    or raw_url.startswith(('#', 'mailto:', 'data:'))
                ):
                    continue
                path_part = unquote(parsed.path)
                if not path_part:
                    continue
                if path_part.startswith('/'):
                    if path_part == site_path:
                        path_part = '/'
                    elif site_path and path_part.startswith(f'{site_path}/'):
                        path_part = path_part[len(site_path):]
                    target = SITE_ROOT / path_part.lstrip('/')
                else:
                    target = html_path.parent / path_part
                target = target.resolve()
                try:
                    target.relative_to(SITE_ROOT.resolve())
                except ValueError:
                    broken.append(f'{html_path}: {raw_url} escapes site')
                    continue
                if target.is_dir():
                    target = target / 'index.html'
                elif not target.suffix and not target.exists():
                    target = target / 'index.html'
                if not target.is_file():
                    broken.append(f'{html_path}: {raw_url}')
        self.assertEqual([], broken)


if __name__ == '__main__':
    unittest.main()
