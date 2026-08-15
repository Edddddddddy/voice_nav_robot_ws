# VoiceNav project website

This directory builds the Chinese-first VoiceNav Robot project site. Content
claims are mapped to repository sources in `source-map.json`; generated output
under `site/` is intentionally ignored.

## Local workflow

Create a Python virtual environment, install the reviewed lock, then build and
test from this directory:

```bash
python -m venv .venv
.venv/bin/python -m pip install --require-hashes -r requirements.lock
.venv/bin/python -m mkdocs build --strict
.venv/bin/python -m unittest discover -s tests -v
```

On Windows, use `.venv\Scripts\python.exe` in place of `.venv/bin/python`.

For a local preview:

```bash
.venv/bin/python -m mkdocs serve --dev-addr 127.0.0.1:8765
```

When dependency intent changes, edit `requirements.in` and regenerate the
hashed lock with the reviewed `pip-tools` version. Do not edit generated hash
entries by hand.

## Production model

通过验证的静态 `site/` 树会提取到带版本号的服务器 release 目录。固定 digest 的 Nginx
容器以只读方式挂载部署根目录，仅在 `127.0.0.1:18081` 发布后端；它与现有 public proxy
通过外部 Docker network `voice-nav-public` 以 DNS 名 `voice-nav-site` 通信。public proxy
只公开 `/voice-nav/`，不会改变服务器根路径和已有应用路由。完整的首次部署、原子切换、
健康检查和回滚命令见 [`deploy/README.md`](deploy/README.md)。
