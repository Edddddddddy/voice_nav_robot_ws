# 第三方 LLM 声明

本声明记录本地 LLM fallback 使用的精确 upstream artifact。仓库只保存 metadata；模型、source archive、
build tree、server binary 与 runtime log 均位于被忽略的 artifact root。

批准的 lock 为
[`models/locks/voice_nav_llm_v1.lock.json`](../../models/locks/voice_nav_llm_v1.lock.json)。
该文件的字节而非浮动仓库状态，是一个 provisioned bundle 的身份。

## Qwen3-0.6B-GGUF

- Upstream repository：`Qwen/Qwen3-0.6B-GGUF`
- Revision：`23749fefcc72300e3a2ad315e1317431b06b590a`
- Artifact：`Qwen3-0.6B-Q8_0.gguf`
- Size：`639446688` bytes
- SHA-256：`9465e63a22add5354d9bb4b99e90117043c7124007664907259bd16d043bb031`
- SPDX license：`Apache-2.0`
- Pinned artifact URL：`https://huggingface.co/Qwen/Qwen3-0.6B-GGUF/resolve/23749fefcc72300e3a2ad315e1317431b06b590a/Qwen3-0.6B-Q8_0.gguf?download=true`
- Pinned source and license record：`https://huggingface.co/Qwen/Qwen3-0.6B-GGUF/tree/23749fefcc72300e3a2ad315e1317431b06b590a`

模型依照 upstream Apache-2.0 grant 使用；本仓库不再分发任何模型字节。

## llama.cpp server

- Official tag：`b10276`
- Commit：`6ea215d171fd31df943bf1ac8227129f2b963160`
- Source archive size：`36570950` bytes
- Source archive SHA-256：`aa90f46e3744796af244af17c2b448589669bb02ec0755ffa8516b07bbc73098`
- SPDX license：`MIT`
- Pinned source archive：`https://codeload.github.com/ggml-org/llama.cpp/tar.gz/6ea215d171fd31df943bf1ac8227129f2b963160`
- Pinned license：`https://github.com/ggml-org/llama.cpp/blob/6ea215d171fd31df943bf1ac8227129f2b963160/LICENSE`

CPU build 为 static，使用已批准的六个 CMake flag：

```text
BUILD_SHARED_LIBS=OFF
LLAMA_BUILD_SERVER=ON
LLAMA_BUILD_TESTS=OFF
LLAMA_BUILD_EXAMPLES=OFF
GGML_NATIVE=OFF
GGML_OPENMP=ON
```

build type 为 `Release`，它不是额外 build flag。生成的 server 是从 locked source 构建、未修改的 upstream
`llama-server` executable；MIT notice 适用于该 upstream source 和其 redistribution terms。

## 运行边界

provisioning 是显式操作，在 extraction 或 build 前验证精确 size 和 SHA-256。runtime consumer 仅使用 lock
digest 下完整的 bundle，将 `llama-server` 绑定到 `127.0.0.1:8080`，且不下载或升级 artifact。JSON-schema
smoke 使用 host `127.0.0.1`、port `8080`、context `2048`、maximum output `256`、parallelism `1`、
`stream=false` 与 `/no_think`。
