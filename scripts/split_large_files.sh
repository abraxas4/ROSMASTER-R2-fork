#!/bin/bash
# Split manifest large files into git-friendly parts (run after updating originals).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WORKSPACE="$REPO_ROOT/code/yahboomcar_ros2_ws"
MANIFEST="$WORKSPACE/large_files.manifest"
CHUNK_SIZE="${CHUNK_SIZE:-45M}"

if [[ ! -f "$MANIFEST" ]]; then
    echo "ERROR: manifest not found: $MANIFEST"
    exit 1
fi

split_one() {
    local rel_path="$1"
    local target="$WORKSPACE/$rel_path"
    local target_dir
    target_dir="$(dirname "$target")"
    local base_name
    base_name="$(basename "$target")"

    if [[ ! -f "$target" ]]; then
        echo "  SKIP (missing source): $rel_path"
        return 0
    fi

    rm -f "$target_dir/${base_name}.part."*
    split -b "$CHUNK_SIZE" -d -a 2 "$target" "$target_dir/${base_name}.part."
    echo "  SPLIT: $rel_path -> $(ls "$target_dir/${base_name}.part."* | wc -l) parts"
}

echo "=== Split large files (chunk=$CHUNK_SIZE) ==="

while IFS= read -r line || [[ -n "$line" ]]; do
    line="${line%%#*}"
    line="$(echo "$line" | xargs)"
    [[ -z "$line" ]] && continue
    split_one "$line"
done < "$MANIFEST"

echo "Done. Commit the .part.* files, not the restored originals."