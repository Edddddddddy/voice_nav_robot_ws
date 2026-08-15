# 部署契约

本仓库声明可复现的静态站拓扑，而不操作任何线上服务器：`voice-nav-site` 使用
[`compose.yaml`](compose.yaml) 中固定 tag 和 digest 的 Nginx image；部署根和 Nginx 配置
均只读挂载。站点仅绑定 `127.0.0.1:18081`，public proxy 则通过外部 Docker network
`voice-nav-public` 中的 DNS 名 `voice-nav-site` 转发 `/voice-nav/`。

目标主机的状态目录固定为：

```text
/opt/apps/voice_nav_site/
  compose.yaml
  current -> releases/<exact-head>
  releases/<exact-head>/
  nginx/default.conf
```

以下命令均从包含本仓库的目标主机 shell 运行。`<public-proxy-container>` 是已运行的 public
proxy 容器名；替换它不会修改 proxy 容器的镜像或其他路由。

## 首次安装 public proxy 与站点容器

先准备目录、版本化 manifest、共享网络和只拥有 `/voice-nav/` 的 Nginx snippet。首次创建
network 后只能把 public proxy 容器接入该 network；Compose 会为站点声明 DNS alias
`voice-nav-site`。public proxy 的配置在它自己的容器中渲染，不能安装到宿主 Nginx：宿主不能
解析 Docker network 中的 `voice-nav-site`。

```bash
repository_root="$(git rev-parse --show-toplevel)"
deployment_root=/opt/apps/voice_nav_site
public_proxy=<public-proxy-container>
sudo install -d -m 0755 "$deployment_root/releases" "$deployment_root/nginx"
sudo install -m 0644 "$repository_root/website/deploy/compose.yaml" "$deployment_root/compose.yaml"
sudo install -m 0644 "$repository_root/website/deploy/nginx.conf" "$deployment_root/nginx/default.conf"
sudo docker network inspect voice-nav-public >/dev/null 2>&1 || sudo docker network create voice-nav-public
sudo docker network connect voice-nav-public "$public_proxy"
sudo install -d -m 0755 "$deployment_root/public-proxy"
proxy_config_source="$deployment_root/public-proxy/default.conf"
proxy_location_source="$deployment_root/public-proxy/voice-nav-site.conf"
sudo docker cp "$public_proxy:/etc/nginx/conf.d/default.conf" "$proxy_config_source"
sudo install -m 0644 "$repository_root/website/deploy/public-location.conf" "$proxy_location_source"
sudo grep -Fqx '    include /etc/nginx/snippets/voice-nav-site.conf;' "$proxy_config_source" || \
  sudo sed -i '$i\    include /etc/nginx/snippets/voice-nav-site.conf;' "$proxy_config_source"
sudo docker exec "$public_proxy" mkdir -p /etc/nginx/snippets
sudo docker cp "$proxy_location_source" "$public_proxy:/etc/nginx/snippets/voice-nav-site.conf"
sudo docker cp "$proxy_config_source" "$public_proxy:/etc/nginx/conf.d/default.conf"
sudo docker exec "$public_proxy" nginx -t
sudo docker exec "$public_proxy" nginx -s reload
```

上述 `default.conf` 副本是可审计的 public proxy 配置源；命令只在其最终 server block 的 `}`
前加入一次 `include /etc/nginx/snippets/voice-nav-site.conf;`。因此 snippet 的安装、包含、
`nginx -t` 和 reload 都发生在同一 public proxy 容器中。该 snippet 保持 `proxy_pass
http://voice-nav-site:80/;`，因此不会把 loopback 端口暴露到公网。若 public proxy 的
`40-render-config.sh` 在容器启动时重新生成 `/etc/nginx/conf.d/default.conf`，须在每次渲染后
重新执行本节，以受审计副本再次写入 include。

## 初次发布

在已锁定依赖的 venv 中构建并测试，然后将生成物提取为不可覆盖的 release。以下命令以当前
Git HEAD 作为 release 名称；不要重复使用已有 release 目录。

```bash
repository_root="$(git rev-parse --show-toplevel)"
release="$(git -C "$repository_root" rev-parse HEAD)"
deployment_root=/opt/apps/voice_nav_site
cd "$repository_root/website"
python3 -m venv .venv
.venv/bin/python -m pip install --require-hashes -r requirements.lock
.venv/bin/python -m mkdocs build --strict
.venv/bin/python -m unittest discover -s tests -v
sudo install -d -m 0755 "$deployment_root/releases/$release"
sudo tar -C site -cf - . | sudo tar -C "$deployment_root/releases/$release" -xf -
cd "$deployment_root"
docker compose -f compose.yaml run --rm --no-deps voice-nav-site nginx -t
ln -sfn "releases/$release" current.next
mv -Tf current.next current
docker compose -f compose.yaml up -d
docker exec voice-nav-site nginx -t
for path in / /architecture/mission-runtime/ /search/search_index.json /assets/stylesheets/extra.css; do
  curl --fail --silent --show-error "http://127.0.0.1:18081$path"
done
curl --fail --silent --show-error http://127.0.0.1:18081/healthz
curl --fail --silent --show-error http://127.0.0.1/voice-nav/healthz
```

## 更新

构建新的 `release` 后，先保留已解析的 `current` 目标；它是此次更新的回滚值。`current.next`
只在验证配置后以 `mv -Tf` 原子替换 `current`。Nginx 容器挂载的是父目录，所以无需重建容器。

```bash
repository_root="$(git rev-parse --show-toplevel)"
release="$(git -C "$repository_root" rev-parse HEAD)"
cd "$repository_root/website"
.venv/bin/python -m mkdocs build --strict
.venv/bin/python -m unittest discover -s tests -v
cd /opt/apps/voice_nav_site
previous="$(readlink -f current)"
test -d "$previous"
sudo install -d -m 0755 "releases/$release"
sudo tar -C "$repository_root/website/site" -cf - . | sudo tar -C "releases/$release" -xf -
docker compose -f compose.yaml run --rm --no-deps voice-nav-site nginx -t
ln -sfn "releases/$release" current.next
mv -Tf current.next current
docker exec voice-nav-site nginx -t && docker exec voice-nav-site nginx -s reload
for path in / /architecture/mission-runtime/ /search/search_index.json /assets/stylesheets/extra.css; do
  curl --fail --silent --show-error "http://127.0.0.1:18081$path"
done
curl --fail --silent --show-error http://127.0.0.1:18081/healthz
curl --fail --silent --show-error http://127.0.0.1/voice-nav/healthz
```

将 `$previous` 连同本次 release id 写入变更记录；不要删除任何 `releases/<exact-head>` 目录。

## 回滚

将变更记录中的实际绝对 release 路径赋给 `previous`，确认其仍在部署根目录后执行。该操作只替换
`current`，不会改动 immutable release、Compose manifest、public proxy 或 Docker network。

```bash
cd /opt/apps/voice_nav_site
previous=/opt/apps/voice_nav_site/releases/<previous-exact-head>
test -d "$previous"
previous_relative="${previous#/opt/apps/voice_nav_site/}"
ln -sfn "$previous_relative" current.next
mv -Tf current.next current
docker exec voice-nav-site nginx -t && docker exec voice-nav-site nginx -s reload
curl --fail --silent --show-error http://127.0.0.1:18081/
curl --fail --silent --show-error http://127.0.0.1:18081/healthz
curl --fail --silent --show-error http://127.0.0.1/voice-nav/healthz
```

`current` 是唯一可替换的路径。若任一 `nginx -t` 或 health check 失败，应停止在当前命令并以
已记录的 `previous` 重复本节，而不是覆盖或删除 release 目录。
