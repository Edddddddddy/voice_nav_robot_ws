#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd -- "${script_dir}/.." && pwd)"
prefix="${workspace_root}/.deps/voice-assets/sherpa-onnx-prefix-shared-ort-tts"
source_archive="${workspace_root}/.deps/voice-assets/sherpa-onnx/sherpa-onnx-142807252687d81b40d6315f23470a1512a00de3.tar.gz"
onnxruntime_zip="${workspace_root}/.deps/voice-assets/onnxruntime/onnxruntime-linux-x64-glibc2_17-Release-1.27.0.zip"
offline=0
fetch=0

source_url="https://codeload.github.com/k2-fsa/sherpa-onnx/tar.gz/142807252687d81b40d6315f23470a1512a00de3"
source_sha256="f0dc7c9b41b8691313daee671e826eb23946fa1320559a8d37e84f8774af76b2"
source_size=9840362
onnxruntime_url="https://github.com/csukuangfj/onnxruntime-libs/releases/download/v1.27.0/onnxruntime-linux-x64-glibc2_17-Release-1.27.0.zip"
onnxruntime_sha256="9f0c0a6998f1b94c399eeddcb443beb4a922c9a4fd431fdc9cd6de67a1935d00"
onnxruntime_size=8509524

usage() {
  cat <<'EOF'
Usage: bash scripts/prepare_sherpa_tts_prefix.sh [options]

Prepare or verify the exact shared-ORT sherpa-onnx TTS prefix.

Options:
  --prefix PATH          Output prefix (default: ignored .deps prefix)
  --source-archive PATH  Exact locked sherpa source archive cache
  --onnxruntime-zip PATH Exact locked ONNX Runtime zip cache
  --offline              Never download; fail if a cache is missing
  --fetch                Fetch only the two immutable URLs above
  --help                 Show this help
EOF
}

while (($#)); do
  case "$1" in
    --prefix) prefix="$2"; shift 2 ;;
    --source-archive) source_archive="$2"; shift 2 ;;
    --onnxruntime-zip) onnxruntime_zip="$2"; shift 2 ;;
    --offline) offline=1; shift ;;
    --fetch) fetch=1; shift ;;
    --help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ((offline && fetch)); then
  echo "--offline and --fetch are mutually exclusive" >&2
  exit 2
fi

prefix_raw="$prefix"
raw_prefix_component="$prefix_raw"
while [[ "$raw_prefix_component" != "/" && "$raw_prefix_component" != "." ]]; do
  if [[ -L "$raw_prefix_component" ]]; then
    echo "refusing symlink/reparse prefix component: $raw_prefix_component" >&2
    exit 2
  fi
  next_raw_component="$(dirname -- "$raw_prefix_component")"
  [[ "$next_raw_component" != "$raw_prefix_component" ]] || break
  raw_prefix_component="$next_raw_component"
done
prefix="$(realpath -m -- "$prefix")"
source_archive="$(realpath -m -- "$source_archive")"
onnxruntime_zip="$(realpath -m -- "$onnxruntime_zip")"
dependency_root="$(realpath -m -- "${workspace_root}/.deps/voice-assets")"
if [[ -L "$prefix" ]]; then
  echo "refusing symlink/reparse prefix: $prefix" >&2
  exit 2
fi
case "$prefix" in
  "${dependency_root}"/*) ;;
  *)
    echo "prefix must be below ignored .deps/voice-assets: $prefix" >&2
    exit 2
    ;;
esac
prefix_relative="${prefix#"${workspace_root}/"}"
git_dir_args=()
if [[ -f "$workspace_root/.git" ]]; then
  git_dir_spec="$(sed -n 's/^gitdir: //p' "$workspace_root/.git")"
  if [[ "$git_dir_spec" =~ ^([A-Za-z]):/(.*)$ ]]; then
    git_drive="${BASH_REMATCH[1],,}"
    git_dir_spec="/mnt/${git_drive}/${BASH_REMATCH[2]}"
  elif [[ "$git_dir_spec" != /* ]]; then
    git_dir_spec="$(realpath -m -- "${workspace_root}/${git_dir_spec}")"
  fi
  [[ -n "$git_dir_spec" ]] || {
    echo "worktree gitdir was empty: $workspace_root/.git" >&2
    exit 2
  }
  git_dir_args=(--git-dir="$git_dir_spec")
fi
git -C "$workspace_root" "${git_dir_args[@]}" check-ignore -q -- "$prefix_relative" || {
  echo "prefix is not proven ignored by git check-ignore: $prefix_relative" >&2
  exit 2
}
if [[ -e "$prefix" ]] && find -P "$prefix" -type l -print -quit | grep -q .; then
  echo "refusing symlink/reparse entry below prefix: $prefix" >&2
  exit 2
fi
cxx="${CXX:-c++}"
cxx_version="$("$cxx" -dumpfullversion -dumpversion 2>/dev/null)" || {
  echo "unable to execute required compiler for exact receipt: $cxx" >&2
  exit 1
}
if [[ "$cxx_version" != "13.3.0" ]]; then
  echo "compiler version mismatch: expected GNU 13.3.0, got $cxx_version" >&2
  exit 1
fi
expected_receipt="$(mktemp)"
work_root="$(mktemp -d)"
cleanup() {
  rm -f -- "$expected_receipt"
  rm -rf -- "$work_root"
}
trap cleanup EXIT

cat >"$expected_receipt" <<'EOF'
schema_version=2
id=sherpa-onnx
version=v1.13.4
revision=142807252687d81b40d6315f23470a1512a00de3
source_sha256=f0dc7c9b41b8691313daee671e826eb23946fa1320559a8d37e84f8774af76b2
onnxruntime_mode=shared
onnxruntime_version=1.27.0
onnxruntime_url=https://github.com/csukuangfj/onnxruntime-libs/releases/download/v1.27.0/onnxruntime-linux-x64-glibc2_17-Release-1.27.0.zip
onnxruntime_zip_size=8509524
onnxruntime_zip_sha256=9f0c0a6998f1b94c399eeddcb443beb4a922c9a4fd431fdc9cd6de67a1935d00
onnxruntime_git_commit=8f0278c77bf44b0cc83c098c6c722b92a36ac4b5
onnxruntime_license=MIT
onnxruntime_soname=libonnxruntime.so
onnxruntime_library=lib/libonnxruntime.so
onnxruntime_library_size=26403889
onnxruntime_library_sha256=026c7d5c609323fb16506dbc3cce801bcdffdd7566fdba49a50727e2e1e881ca
build_system=CMake
cxx_compiler=GNU 13.3.0
BUILD_SHARED_LIBS=OFF
SHERPA_ONNX_ENABLE_C_API=ON
SHERPA_ONNX_ENABLE_TESTS=OFF
SHERPA_ONNX_ENABLE_PORTAUDIO=OFF
SHERPA_ONNX_ENABLE_WEBSOCKET=OFF
SHERPA_ONNX_ENABLE_TTS=ON
SHERPA_ONNX_ENABLE_SPEAKER_DIARIZATION=OFF
SHERPA_ONNX_ENABLE_BINARY=OFF
SHERPA_ONNX_USE_PRE_INSTALLED_ONNXRUNTIME_IF_AVAILABLE=ON
c_api_header=include/sherpa-onnx/c-api/c-api.h
c_api_library=lib/libsherpa-onnx-c-api.a
core_library=lib/libsherpa-onnx-core.a
EOF

verify_regular_file_identity() {
  local file="$1" expected_size="$2" expected_sha256="$3"
  [[ -f "$file" && ! -L "$file" ]] || {
    echo "required regular file missing or symlink: $file" >&2
    return 1
  }
  [[ "$(stat -c "%s" "$file")" == "$expected_size" ]] || {
    echo "required file size mismatch: $file" >&2
    return 1
  }
  [[ "$(sha256sum "$file" | awk "{print \$1}")" == "$expected_sha256" ]] || {
    echo "required file sha256 mismatch: $file" >&2
    return 1
  }
}

verify_regular_nonempty_file() {
  local file="$1"
  [[ -f "$file" && ! -L "$file" ]] || {
    echo "required regular file missing or symlink: $file" >&2
    return 1
  }
  [[ -s "$file" ]] || {
    echo "required file is empty: $file" >&2
    return 1
  }
}

verify_prefix_artifacts() {
  local candidate="$1"
  verify_regular_file_identity \
    "$candidate/lib/libonnxruntime.so" \
    26403889 \
    026c7d5c609323fb16506dbc3cce801bcdffdd7566fdba49a50727e2e1e881ca
  verify_regular_nonempty_file "$candidate/lib/libsherpa-onnx-c-api.a"
  verify_regular_nonempty_file "$candidate/lib/libsherpa-onnx-core.a"
  verify_regular_nonempty_file "$candidate/include/sherpa-onnx/c-api/c-api.h"
}

verify_prefix() {
  local candidate="$1"
  local receipt="$candidate/share/voice_nav/sherpa-onnx-provenance.receipt"
  if [[ ! -f "$receipt" ]]; then
    echo "exact sherpa receipt mismatch: $receipt" >&2
    return 1
  fi
  if ! cmp -s "$expected_receipt" "$receipt"; then
    echo "exact sherpa receipt mismatch: $receipt" >&2
    return 1
  fi
  verify_prefix_artifacts "$candidate"
}

if [[ -d "$prefix" ]] && verify_prefix "$prefix"; then
  echo "sherpa_tts_prefix=verified"
  exit 0
fi

if [[ -e "$prefix" ]]; then
  echo "refusing to replace incomplete sherpa prefix: $prefix" >&2
  exit 1
fi

fetch_exact() {
  local url="$1" expected_size="$2" expected_sha256="$3" destination="$4"
  mkdir -p -- "$(dirname -- "$destination")"
  if [[ ! -f "$destination" ]]; then
    if ((offline)) || ((!fetch)); then
      echo "missing locked cache (use --fetch outside runtime): $destination" >&2
      exit 1
    fi
    local temporary="${destination}.tmp.$$"
    curl --fail --location --proto '=https' --tlsv1.2 --output "$temporary" "$url"
    mv -- "$temporary" "$destination"
  fi
  [[ "$(stat -c '%s' "$destination")" == "$expected_size" ]] || {
    echo "locked archive size mismatch: $destination" >&2; exit 1;
  }
  [[ "$(sha256sum "$destination" | awk '{print $1}')" == "$expected_sha256" ]] || {
    echo "locked archive sha256 mismatch: $destination" >&2; exit 1;
  }
}

fetch_exact "$source_url" "$source_size" "$source_sha256" "$source_archive"
fetch_exact "$onnxruntime_url" "$onnxruntime_size" "$onnxruntime_sha256" "$onnxruntime_zip"

mkdir -p -- "$work_root/source" "$work_root/onnxruntime" "$work_root/prefix"
tar -xzf "$source_archive" -C "$work_root/source"
source_dir="$(find "$work_root/source" -mindepth 1 -maxdepth 1 -type d -print -quit)"
[[ -n "$source_dir" ]] || { echo "sherpa source archive had no root" >&2; exit 1; }
unzip -q "$onnxruntime_zip" -d "$work_root/onnxruntime"
ort_library="$(find "$work_root/onnxruntime" -type f -name 'libonnxruntime.so*' -print -quit)"
ort_include="$(find "$work_root/onnxruntime" -type d -name include -print -quit)"
[[ -n "$ort_library" && -n "$ort_include" &&
  -f "$ort_include/onnxruntime_c_api.h" &&
  -f "$ort_include/onnxruntime_cxx_api.h" ]] || {
  echo "ONNX Runtime cache lacks the complete C++ API include set/library" >&2; exit 1;
}
mkdir -p -- "$work_root/ort-root/lib" "$work_root/ort-root/include"
cp -L -- "$ort_library" "$work_root/ort-root/lib/libonnxruntime.so"
cp -a -- "$ort_include/." "$work_root/ort-root/include/"
verify_regular_file_identity \
  "$work_root/ort-root/lib/libonnxruntime.so" \
  26403889 \
  026c7d5c609323fb16506dbc3cce801bcdffdd7566fdba49a50727e2e1e881ca

# sherpa-onnx v1.13.4's CMake contract discovers a pre-installed ORT through
# these two immutable directory variables; ONNXRUNTIME_ROOT_DIR is ignored.
export SHERPA_ONNXRUNTIME_INCLUDE_DIR="$work_root/ort-root/include"
export SHERPA_ONNXRUNTIME_LIB_DIR="$work_root/ort-root/lib"

cmake -S "$source_dir" -B "$work_root/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$work_root/prefix" \
  -DBUILD_SHARED_LIBS=OFF \
  -DSHERPA_ONNX_ENABLE_C_API=ON \
  -DSHERPA_ONNX_ENABLE_TESTS=OFF \
  -DSHERPA_ONNX_ENABLE_PORTAUDIO=OFF \
  -DSHERPA_ONNX_ENABLE_WEBSOCKET=OFF \
  -DSHERPA_ONNX_ENABLE_TTS=ON \
  -DSHERPA_ONNX_ENABLE_SPEAKER_DIARIZATION=OFF \
  -DSHERPA_ONNX_ENABLE_BINARY=OFF \
  -DSHERPA_ONNX_USE_PRE_INSTALLED_ONNXRUNTIME_IF_AVAILABLE=ON
cmake --build "$work_root/build" --target install --parallel 2

mkdir -p -- "$work_root/prefix/share/voice_nav" "$work_root/prefix/lib"
cp -L -- "$work_root/ort-root/lib/libonnxruntime.so" "$work_root/prefix/lib/libonnxruntime.so"
verify_prefix_artifacts "$work_root/prefix"
cp -- "$expected_receipt" "$work_root/prefix/share/voice_nav/sherpa-onnx-provenance.receipt"
verify_prefix "$work_root/prefix"
mv -- "$work_root/prefix" "$prefix"
echo "sherpa_tts_prefix=prepared"
