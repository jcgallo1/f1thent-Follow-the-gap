#!/bin/bash
# Lanza el controlador hibrido PP + FTG (mapa SaoPaulo)
set -e

PKG_DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/humble/setup.bash
source "$PKG_DIR/install/setup.bash"

echo "=== reactive_race / raceline_follower ==="
echo "Codigo: $PKG_DIR/reactive_race/raceline_follower.py"
echo "Esperando /scan y /ego_racecar/odom del simulador..."
echo ""

ros2 run reactive_race raceline_follower
