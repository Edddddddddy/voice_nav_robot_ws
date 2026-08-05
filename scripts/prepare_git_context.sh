#!/usr/bin/env bash

# This file is sourced by verification entry points. It resolves the managed
# worktree pointer before invoking read-only Git probes: a worktree created on
# Windows can contain a gitdir: pointer that WSL Git cannot parse until its
# path has been converted.

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

voice_nav_read_single_line() {
  local path="${1:-}"
  local file_descriptor
  local extra_line=""

  VOICE_NAV_SINGLE_LINE=""
  if [[ -z "${path}" ]]; then
    return 1
  fi
  if ! exec {file_descriptor}<"${path}"; then
    return 1
  fi
  if ! IFS= read -r VOICE_NAV_SINGLE_LINE <&"${file_descriptor}"; then
    if [[ -z "${VOICE_NAV_SINGLE_LINE}" ]]; then
      exec {file_descriptor}<&-
      return 1
    fi
  fi
  if [[ -z "${VOICE_NAV_SINGLE_LINE}" ]]; then
    exec {file_descriptor}<&-
    return 1
  fi
  if [[ "${VOICE_NAV_SINGLE_LINE}" == *$'\r' ]]; then
    exec {file_descriptor}<&-
    return 1
  fi
  if IFS= read -r extra_line <&"${file_descriptor}"; then
    exec {file_descriptor}<&-
    return 1
  fi
  if [[ -n "${extra_line}" ]]; then
    exec {file_descriptor}<&-
    return 1
  fi
  exec {file_descriptor}<&-
}

voice_nav_convert_windows_git_target() {
  local windows_target="${1:-}"
  local output_file

  if ! output_file="$(mktemp)"; then
    voice_nav_git_context_error "cannot allocate a temporary conversion result" "${windows_target}"
    return 1
  fi
  if ! wslpath -u -- "${windows_target}" >"${output_file}" 2>/dev/null; then
    rm -f -- "${output_file}" || true
    voice_nav_git_context_error "Windows gitdir target conversion failed" "${windows_target}"
    return 1
  fi
  if ! voice_nav_read_single_line "${output_file}"; then
    rm -f -- "${output_file}" || true
    voice_nav_git_context_error "Windows gitdir target conversion returned an invalid path" "${windows_target}"
    return 1
  fi
  rm -f -- "${output_file}" || true
  if [[ -z "${VOICE_NAV_SINGLE_LINE}" ]]; then
    voice_nav_git_context_error "Windows gitdir target conversion returned an empty path" "${windows_target}"
    return 1
  fi
  VOICE_NAV_CONVERTED_TARGET="${VOICE_NAV_SINGLE_LINE}"
}

voice_nav_validate_git_context() {
  local workspace_root="${1:-}"
  local tracked_files

  if ! command -v git >/dev/null 2>&1; then
    voice_nav_git_context_error "git is unavailable for context validation" "${workspace_root}"
    return 1
  fi
  if ! (
    cd -- "${workspace_root}" &&
    GIT_OPTIONAL_LOCKS=0 git rev-parse --git-dir >/dev/null 2>&1
  ); then
    voice_nav_git_context_error "Git directory probe failed" "${workspace_root}"
    return 1
  fi
  if ! (
    cd -- "${workspace_root}" &&
    GIT_OPTIONAL_LOCKS=0 git rev-parse --verify 'HEAD^{commit}' >/dev/null 2>&1
  ); then
    voice_nav_git_context_error "Git HEAD probe failed" "${workspace_root}"
    return 1
  fi
  if ! tracked_files="$(
    cd -- "${workspace_root}" &&
    GIT_OPTIONAL_LOCKS=0 git ls-files --cached
  )"; then
    voice_nav_git_context_error "Git tracked-files probe failed" "${workspace_root}"
    return 1
  fi
  if [[ -z "${tracked_files}" ]]; then
    voice_nav_git_context_error "Git tracked-files probe returned no files" "${workspace_root}"
    return 1
  fi
  if ! (
    cd -- "${workspace_root}" &&
    GIT_OPTIONAL_LOCKS=0 git status --porcelain --untracked-files=no >/dev/null 2>&1
  ); then
    voice_nav_git_context_error "Git worktree status probe failed" "${workspace_root}"
    return 1
  fi
}

voice_nav_prepare_git_context() {
  local workspace_root="${1:-}"
  local git_pointer
  local git_pointer_directory
  local pointer_contents
  local git_target
  local candidate
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
    if ! voice_nav_read_single_line "${git_pointer}"; then
      voice_nav_git_context_error "cannot read .git pointer" "${git_pointer}"
      return 1
    fi
    pointer_contents="${VOICE_NAV_SINGLE_LINE}"
    if [[ "${pointer_contents}" != gitdir:\ * ]]; then
      voice_nav_git_context_error "malformed .git pointer" "${git_pointer}"
      return 1
    fi
    git_target="${pointer_contents#gitdir: }"
    if [[ -z "${git_target}" || "${git_target}" == *$'\n'* || "${git_target}" == *$'\r'* || ( "${git_target:0:1}" == '"' && "${git_target: -1}" == '"' ) || ( "${git_target:0:1}" == "'" && "${git_target: -1}" == "'" ) ]]; then
      voice_nav_git_context_error "empty or invalid gitdir target" "${git_pointer}"
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
        if ! voice_nav_convert_windows_git_target "${normalized_windows_target}"; then
          return 1
        fi
        candidate="${VOICE_NAV_CONVERTED_TARGET}"
      elif [[ -d "${normalized_windows_target}" ]]; then
        candidate="${normalized_windows_target}"
      elif command -v wslpath >/dev/null 2>&1; then
        if ! voice_nav_convert_windows_git_target "${normalized_windows_target}"; then
          return 1
        fi
        candidate="${VOICE_NAV_CONVERTED_TARGET}"
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
  voice_nav_validate_git_context "${workspace_root}"
}
