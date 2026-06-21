#!/bin/bash
# Re-enable Yahboom phone app autostart.

set -euo pipefail

AUTOSTART="$HOME/.config/autostart/start_app.sh.desktop"
DISABLED="$HOME/.config/autostart/start_app.sh.desktop.disabled"

if [[ -f "$DISABLED" ]]; then
  mv "$DISABLED" "$AUTOSTART"
  echo "Re-enabled: start_app.sh.desktop"
elif [[ -f "$AUTOSTART" ]]; then
  echo "Already enabled: $AUTOSTART"
else
  echo "Not found: $DISABLED"
  exit 1
fi