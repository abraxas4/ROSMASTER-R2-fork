#!/bin/bash
# Restore split large files after git pull.
# Runtime expects the full files; parts are only for git storage.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WORKSPACE="$REPO_ROOT/code/yahboomcar_ros2_ws"
MANIFEST="$WORKSPACE/large_files.manifest"

if [[ ! -f "$MANIFEST" ]]; then
    echo "No large_files.manifest found, skipping restore."
    exit 0
fi

restore_one() {
    local rel_path="$1"
    local target="$WORKSPACE/$rel_path"
    local target_dir
    target_dir="$(dirname "$target")"
    local base_name
    base_name="$(basename "$target")"
    local part_glob="$target_dir/${base_name}.part.*"

    shopt -s nullglob
    local parts=( "$target_dir/${base_name}.part."* )
    shopt -u nullglob

    if [[ ${#parts[@]} -eq 0 ]]; then
        echo "  SKIP (no parts): $rel_path"
        return 0
    fi

    local newest_part
    newest_part="$(ls -t "${parts[@]}" | head -1)"

    if [[ -f "$target" ]] && [[ "$target" -nt "$newest_part" ]]; then
        echo "  OK (already restored): $rel_path"
        return 0
    fi

    echo "  RESTORE: $rel_path (${#parts[@]} parts)"
    cat "${parts[@]}" > "$target"
}

echo "=== Restore large files ==="

while IFS= read -r line || [[ -n "$line" ]]; do
    line="${line%%#*}"
    line="$(echo "$line" | xargs)"
    [[ -z "$line" ]] && continue
    restore_one "$line"
done < "$MANIFEST"

echo "Done."