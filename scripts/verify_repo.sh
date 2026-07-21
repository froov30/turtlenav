#!/usr/bin/env bash
set -euo pipefail

# verify_repo.sh - Local verification script for GitHub readiness
# Usage:
#   ./verify_repo.sh            # run quick checks (git, remotes, tracked artifacts, colcon list)
#   ./verify_repo.sh --build    # also run colcon build/test (may be slow)

REPO_DEFAULT="/home/dhruv/turtle_game_ws/src/ros2_turtle_chaser"
WS_DEFAULT="/home/dhruv/turtle_game_ws"

BUILD=0
REPO="$REPO_DEFAULT"
WS="$WS_DEFAULT"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build)
      BUILD=1
      shift
      ;;
    --repo)
      REPO="$2"; shift 2
      ;;
    --ws)
      WS="$2"; shift 2
      ;;
    -h|--help)
      sed -n '1,120p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

echo "Repository: $REPO"
echo "Workspace: $WS"

echo "\n== Git status (short) =="
if command -v git >/dev/null 2>&1; then
  git -C "$REPO" status --short || true
  echo "Current branch: $(git -C "$REPO" rev-parse --abbrev-ref HEAD || true)"
  echo "Remotes:"
  git -C "$REPO" remote -v || true
else
  echo "git not found in PATH"
fi

echo "\n== Tracked generated artifacts scan =="
if command -v git >/dev/null 2>&1; then
  git -C "$REPO" ls-files | egrep -i '(^|/)(build|install|log|__pycache__|\.pytest_cache|\.vscode|\.idea|\.env|\.pyc$|\.so$|\.egg-info$)' || true
else
  echo "git not available to check tracked files"
fi

echo "\n== Colcon package list =="
if command -v colcon >/dev/null 2>&1; then
  (cd "$WS" && colcon list --base-paths src) || true
else
  echo "colcon not found in PATH; skip package list"
fi

if [[ "$BUILD" -eq 1 ]]; then
  echo "\n== Building and testing (this can take several minutes) =="
  if command -v colcon >/dev/null 2>&1; then
    (cd "$WS" && colcon build --symlink-install) || { echo "colcon build failed"; exit 3; }
    (cd "$WS" && colcon test --packages-select turtle_game turtle_game_interfaces) || true
    (cd "$WS" && colcon test-result --verbose) || true
  else
    echo "colcon not found; cannot build/test"
  fi
else
  echo "\nBuild step skipped; re-run with --build to run colcon build/test"
fi

echo "\n== README / LICENSE sanity =="
README_PATH="$REPO/README.md"
LICENSE_PATH="$REPO/LICENSE"
if [[ -f "$README_PATH" ]]; then
  echo "README found: $README_PATH"
  echo "First 10 lines:"; sed -n '1,10p' "$README_PATH"
else
  echo "README.md not found at $README_PATH"
fi
if [[ -f "$LICENSE_PATH" ]]; then
  echo "\nLICENSE found: $LICENSE_PATH"
  sed -n '1,10p' "$LICENSE_PATH"
else
  echo "LICENSE not found at $LICENSE_PATH"
fi

echo "\n== Done. Review outputs above and fix any FAIL items interactively. =="
