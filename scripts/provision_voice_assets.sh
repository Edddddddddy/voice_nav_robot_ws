#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd -- "${script_dir}/.." && pwd)"

# Windows-created worktrees may carry a gitdir pointer that WSL Git cannot read.
# The formal CLI uses check-ignore before any artifact directory is created.
# shellcheck source=prepare_git_context.sh
source "${script_dir}/prepare_git_context.sh"
voice_nav_prepare_git_context "${workspace_root}"

usage() {
  cat <<'EOF'
Usage: bash scripts/provision_voice_assets.sh [--verify] [--offline]

Explicitly provision VoiceNav's locked audio dependencies and models before
runtime. Downloads are written only to ignored .deps/ and models/weights/
directories after size and SHA-256 verification.

Options:
  --verify   Verify the already provisioned assets without downloading.
  --offline  Refuse downloads; fail unless every asset is already verified.
  --help     Show this help.
EOF
}

mode="provision"
if [[ "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "${1:-}" == "--verify" ]]; then
  mode="verify"
  shift
fi

exec python3 "${script_dir}/voice_asset_manager.py" "${mode}" \
  --repo-root "${workspace_root}" "$@"
