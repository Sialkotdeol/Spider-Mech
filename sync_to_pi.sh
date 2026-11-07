#!/usr/bin/env bash
# Copy this project from the Mac to the Pi.
#
# Never copies the Pi's own data: calibration.json (servo calibration)
# and poses.json (saved poses) are tuned on the robot and must stay
# there. Any other file the sync replaces is kept on the Pi with a
# .before-sync suffix, so nothing is lost silently.
#
# Run it as a file, never paste its contents into a terminal:
#         ~/Desktop/spider_mech/sync_to_pi.sh
#         PI=pi@spidermech.local ~/Desktop/spider_mech/sync_to_pi.sh
set -euo pipefail
PI="${PI:-pi@192.168.68.107}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Refuse to copy anything that is not this project. Pasted into a
# shell, the path above resolves to the current folder (often the
# whole home folder), which must never be sent to the Pi.
if [ ! -f "$HERE/backend/main.py" ] || [ ! -f "$HERE/frontend/index.html" ]; then
  echo "sync_to_pi.sh: '$HERE' is not the spider_mech project folder - nothing copied." >&2
  echo "Run the script as a file: ~/Desktop/spider_mech/sync_to_pi.sh" >&2
  exit 1
fi

rsync -av --backup --suffix=.before-sync \
  --exclude '__pycache__' --exclude '.venv' --exclude '.DS_Store' --exclude '.pytest_cache' \
  --exclude '*.before-sync' \
  --exclude 'backend/calibration.json' --exclude 'backend/calibration.json.tmp' \
  --exclude 'backend/poses.json' \
  "$HERE/" "$PI:~/Desktop/spider_mech/"

echo
echo "Synced to $PI:~/Desktop/spider_mech (calibration.json and poses.json left untouched)."
echo "Restart uvicorn on the Pi, then reload the browser."
