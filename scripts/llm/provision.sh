#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd -- "${script_dir}/../.." && pwd)"

# Normalize a managed worktree's Windows gitdir pointer before the read-only
# repository boundary check in the Python module.
# shellcheck disable=SC1091
source "${workspace_root}/scripts/prepare_git_context.sh"
voice_nav_prepare_git_context "${workspace_root}"

exec python3 "${script_dir}/artifact_manager.py" provision \
  --repo-root "${workspace_root}" "$@"
