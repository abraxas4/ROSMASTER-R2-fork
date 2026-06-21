#!/bin/bash
# Disable Yahboom boot-time extras (reversible). Does not delete anything.

set -euo pipefail

AUTOSTART="$HOME/.config/autostart/start_app.sh.desktop"
DISABLED="$HOME/.config/autostart/start_app.sh.desktop.disabled"

if [[ -f "$AUTOSTART" ]]; then
  mv "$AUTOSTART" "$DISABLED"
  echo "Disabled: start_app.sh.desktop (rosmaster_main.py phone app)"
elif [[ -f "$DISABLED" ]]; then
  echo "Already disabled: $DISABLED"
else
  echo "Not found: $AUTOSTART"
fi

if pgrep -f rosmaster_main.py >/dev/null 2>&1; then
  echo "Stopping running rosmaster_main.py..."
  pkill -f rosmaster_main.py || true
fi

echo ""
echo "Optional (manual):"
echo "  sudo systemctl disable --now yahboom_oled.service   # OLED display only"
echo ""
echo "Re-enable phone app autostart:"
echo "  mv ~/.config/autostart/start_app.sh.desktop.disabled ~/.config/autostart/start_app.sh.desktop"