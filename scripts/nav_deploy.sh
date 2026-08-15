#!/bin/bash
# Deploy source/config plus the runtime scripts, then run the separate RDK
# cleanup and dependency-closure build. This never starts a camera or motion.
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TARGET=${SMARTCAR_RDK_HOST:-root@192.168.128.10}
if [[ "$TARGET" != *@* ]]; then
  TARGET="root@$TARGET"
fi
DRY_RUN=false

for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=true ;;
    *) echo "usage: bash scripts/nav_deploy.sh [--dry-run]"; exit 2 ;;
  esac
done

cd "$REPO_ROOT"
if $DRY_RUN; then
  SMARTCAR_RDK_HOST="$TARGET" python3 scripts/sync_to_rdk.py push --dry-run
  echo "dry run: no RDK backup, cleanup, or build was started"
  exit 0
fi

backup=$(ssh -o BatchMode=yes -o ConnectTimeout=8 "$TARGET" \
  'set -e; mkdir -p /root/nav_backups; path=/root/nav_backups/pre_nav_deploy_$(date +%Y%m%d_%H%M%S).tar.gz; tar -C /home/sunrise/ros2_ws -czf "$path" src config; printf "%s" "$path"')
echo "RDK backup: $backup"
SMARTCAR_RDK_HOST="$TARGET" python3 scripts/sync_to_rdk.py push
ssh -o BatchMode=yes -o ConnectTimeout=8 "$TARGET" 'bash /home/sunrise/ros2_ws/scripts/nav_prepare.sh'
