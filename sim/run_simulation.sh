#!/bin/bash
# Euler Swarm Simulation Launcher

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

NUM_DRONES=${1:-10}
WORLD_FILE=${2:-"gazebo/worlds/swarm_50.world"}

echo "=== Euler Swarm Simulation ==="
echo "Drones: $NUM_DRONES"
echo "World: $WORLD_FILE"
echo ""

export GAZEBO_MODEL_PATH="$SCRIPT_DIR/gazebo/models:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="$SCRIPT_DIR/gazebo/plugins/build:$GAZEBO_PLUGIN_PATH"

if [ ! -f "$SCRIPT_DIR/gazebo/plugins/build/libeuler_gazebo_plugin.so" ]; then
    echo "Building Gazebo plugin..."
    mkdir -p "$SCRIPT_DIR/gazebo/plugins/build"
    cd "$SCRIPT_DIR/gazebo/plugins/build"
    cmake .. && make
    cd "$SCRIPT_DIR"
fi

if ! command -v gazebo &> /dev/null; then
    echo "Error: Gazebo not found. Please install Gazebo."
    echo "  macOS: brew install gazebo"
    echo "  Ubuntu: sudo apt install gazebo"
    exit 1
fi

echo "Starting Gazebo..."
gazebo --verbose "$SCRIPT_DIR/$WORLD_FILE"

