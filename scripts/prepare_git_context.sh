#!/usr/bin/env bash

# This file is sourced by verification entry points. It deliberately does
# not invoke Git: a managed worktree created on Windows can contain a
# gitdir: pointer that a WSL Git process cannot parse until its path has been
# converted.

voice_nav_git_context_error() {
  local message="${1:-unknown failure}"
  local target="${2:-}"
  local normalized_target
  local summary="<none>"

  if [[ -n "${target}" ]]; then
    normalized_target="${target//\\//}"
    summary="${normalized_target##*/}"
    if [[ -z "${summary}" || "${summary}" == "${normalized_target}" ]]; then
      summary="${normalized_target}"
    fi
  fi
  printf 'Git context error: %s (target=%s)\n' "${message}" "${summary}" >&2
  return 1
}

voice_nav_prepare_git_context() {
  local workspace_root="${1:-}"
  local git_pointer
  local git_pointer_directory
  local pointer_contents
  local git_target
  local candidate
  local converted
  local resolved_git_dir
  local normalized_windows_target
  local shell_kernel

  if [[ -z "${workspace_root}" ]]; then
    voice_nav_git_context_error "workspace root is empty"
    return 1
  fi
  if ! workspace_root="$(cd -- "${workspace_root}" 2>/dev/null && pwd -P)"; then
    voice_nav_git_context_error "workspace root is not accessible" "${workspace_root}"
    return 1
  fi

  git_pointer="${workspace_root}/.git"
  if [[ -d "${git_pointer}" ]]; then
    candidate="${git_pointer}"
  elif [[ -f "${git_pointer}" ]]; then
    git_pointer_directory="$(dirname -- "${git_pointer}")"
    if ! pointer_contents="$(<"${git_pointer}")"; then
      voice_nav_git_context_error "cannot read .git pointer" "${git_pointer}"
      return 1
    fi
    if [[ "${pointer_contents}" != gitdir:\ * ]]; then
      voice_nav_git_context_error "malformed .git pointer" "${git_pointer}"
      return 1
    fi
    git_target="${pointer_contents#gitdir: }"
    if [[ -z "${git_target}" || "${git_target}" == *$'\n'* || "${git_target}" == *$'\r'* ]]; then
      voice_nav_git_context_error "empty or multiline gitdir target" "${git_pointer}"
      return 1
    fi

    if [[ "${git_target}" =~ ^[[:alpha:]]:[\\/].* ]]; then
      normalized_windows_target="${git_target//\\//}"
      shell_kernel="$(uname -s 2>/dev/null || true)"
      if [[ "${shell_kernel}" == Linux* ]]; then
        if ! command -v wslpath >/dev/null 2>&1; then
          voice_nav_git_context_error "Windows gitdir target cannot be converted: wslpath is unavailable" "${git_target}"
          return 1
        fi
        if ! converted="$(wslpath -u -- "${normalized_windows_target}" 2>/dev/null)"; then
          voice_nav_git_context_error "Windows gitdir target conversion failed" "${git_target}"
          return 1
        fi
        if [[ -z "${converted}" || "${converted}" == *$'\n'* || "${converted}" == *$'\r'* ]]; then
          voice_nav_git_context_error "Windows gitdir target conversion returned an invalid path" "${git_target}"
          return 1
        fi
        candidate="${converted}"
      elif [[ -d "${normalized_windows_target}" ]]; then
        candidate="${normalized_windows_target}"
      elif command -v wslpath >/dev/null 2>&1; then
        if ! converted="$(wslpath -u -- "${normalized_windows_target}" 2>/dev/null)"; then
          voice_nav_git_context_error "Windows gitdir target conversion failed" "${git_target}"
          return 1
        fi
        if [[ -z "${converted}" || "${converted}" == *$'\n'* || "${converted}" == *$'\r'* ]]; then
          voice_nav_git_context_error "Windows gitdir target conversion returned an invalid path" "${git_target}"
          return 1
        fi
        candidate="${converted}"
      else
        voice_nav_git_context_error "Windows gitdir target cannot be converted" "${git_target}"
        return 1
      fi
    elif [[ "${git_target}" == /* ]]; then
      candidate="${git_target}"
    else
      candidate="${git_pointer_directory}/${git_target}"
    fi
  else
    voice_nav_git_context_error "workspace has no .git directory or pointer" "${git_pointer}"
    return 1
  fi

  if ! command -v realpath >/dev/null 2>&1; then
    voice_nav_git_context_error "realpath is unavailable for Git directory validation" "${candidate}"
    return 1
  fi
  if ! resolved_git_dir="$(realpath -e -- "${candidate}" 2>/dev/null)"; then
    voice_nav_git_context_error "Git directory target does not exist" "${candidate}"
    return 1
  fi
  if [[ ! -d "${resolved_git_dir}" || ! -f "${resolved_git_dir}/HEAD" ]]; then
    voice_nav_git_context_error "Git directory target is missing a valid HEAD" "${resolved_git_dir}"
    return 1
  fi

  export GIT_DIR="${resolved_git_dir}"
  export GIT_WORK_TREE="${workspace_root}"
}
