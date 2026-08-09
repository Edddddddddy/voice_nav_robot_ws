# Third-Party LLM Notices

This notice records the exact upstream artifacts used by the local LLM
fallback. The repository contains metadata only; the model, source archive,
build tree, server binary, and runtime logs stay in an ignored artifact root.

The approved lock is
[`models/locks/voice_nav_llm_v1.lock.json`](../../models/locks/voice_nav_llm_v1.lock.json).
Its bytes, rather than a floating repository state, are the identity of one
provisioned bundle.

## Qwen3-0.6B-GGUF

- Upstream repository: `Qwen/Qwen3-0.6B-GGUF`
- Revision: `23749fefcc72300e3a2ad315e1317431b06b590a`
- Artifact: `Qwen3-0.6B-Q8_0.gguf`
- Size: `639446688` bytes
- SHA-256: `9465e63a22add5354d9bb4b99e90117043c7124007664907259bd16d043bb031`
- SPDX license: `Apache-2.0`
- Pinned artifact URL: `https://huggingface.co/Qwen/Qwen3-0.6B-GGUF/resolve/23749fefcc72300e3a2ad315e1317431b06b590a/Qwen3-0.6B-Q8_0.gguf?download=true`
- Pinned source and license record: `https://huggingface.co/Qwen/Qwen3-0.6B-GGUF/tree/23749fefcc72300e3a2ad315e1317431b06b590a`

The model is used under the upstream Apache-2.0 grant. No model bytes are
redistributed in this repository.

## llama.cpp server

- Official tag: `b10276`
- Commit: `6ea215d171fd31df943bf1ac8227129f2b963160`
- Source archive size: `36570950` bytes
- Source archive SHA-256: `aa90f46e3744796af244af17c2b448589669bb02ec0755ffa8516b07bbc73098`
- SPDX license: `MIT`
- Pinned source archive: `https://codeload.github.com/ggml-org/llama.cpp/tar.gz/6ea215d171fd31df943bf1ac8227129f2b963160`
- Pinned license: `https://github.com/ggml-org/llama.cpp/blob/6ea215d171fd31df943bf1ac8227129f2b963160/LICENSE`

The CPU build is static with the approved six CMake flags:

```text
BUILD_SHARED_LIBS=OFF
LLAMA_BUILD_SERVER=ON
LLAMA_BUILD_TESTS=OFF
LLAMA_BUILD_EXAMPLES=OFF
GGML_NATIVE=OFF
GGML_OPENMP=ON
```

The build type is `Release`; it is not an additional build flag. The resulting
server is an unmodified upstream `llama-server` executable built from the
locked source. The MIT notice applies to that upstream source and its
redistribution terms.

## Runtime boundary

Provisioning is explicit and verifies exact size and SHA-256 before extraction
or build. Runtime consumers use only a complete bundle under the lock digest,
bind `llama-server` to `127.0.0.1:8080`, and do not download or upgrade any
artifact. The JSON-schema smoke uses host `127.0.0.1`, port `8080`, context
`2048`, maximum output `256`, parallelism `1`, `stream=false`, and
`/no_think`.
