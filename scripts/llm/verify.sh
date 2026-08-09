#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd -- "${script_dir}/../.." && pwd)"

# A managed worktree may expose a Windows gitdir pointer to WSL.  The
# repository boundary check is read-only, but it still needs the normalized
# Git context before the Python module asks Git for tracked paths.
# shellcheck disable=SC1091
source "${workspace_root}/scripts/prepare_git_context.sh"
voice_nav_prepare_git_context "${workspace_root}"

exec python3 "${script_dir}/artifact_manager.py" verify \
  --repo-root "${workspace_root}" "$@"
