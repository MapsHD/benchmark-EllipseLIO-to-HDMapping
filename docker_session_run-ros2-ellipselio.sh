#!/bin/bash

IMAGE_NAME='ellipselio_humble'
TMUX_SESSION='ros2_EllipseLIO'

DATASET_CONTAINER_PATH='/ros2_ws/dataset/input.bag'
DATASET_ROS2_PATH='/tmp/dataset_ros2'
BAG_OUTPUT_CONTAINER='/ros2_ws/recordings'

RECORDED_BAG_NAME="recorded-EllipseLIO"
HDMAPPING_OUT_NAME="output_hdmapping"

# EllipseLIO publishes these by default (see ellipselio/src/map_processing.cpp).
ODOM_TOPIC=${ODOM_TOPIC:-/ellipselio_odom}
CLOUD_TOPIC=${CLOUD_TOPIC:-/cloud_scan}

# Config file inside the ellipselio package (config/<file>.yaml). The config
# selects the LiDAR/IMU topics that EllipseLIO subscribes to.
CONFIG_FILE=${CONFIG_FILE:-qt64_spires.yaml}

usage() {
  echo "Usage:"
  echo "  $0 <input.bag> <output_dir>"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  echo
  echo "Environment variables:"
  echo "  CONFIG_FILE   - ellipselio config file name (default: qt64_spires.yaml)"
  echo "                  available: os64_ncd.yaml, os128_ncd.yaml, os64_geode.yaml,"
  echo "                             qt64_spires.yaml, vlp16_bot.yaml, vlp16_geode.yaml,"
  echo "                             vlp16_graco.yaml"
  echo "  ODOM_TOPIC    - recorded odometry topic (default: /ellipselio_odom)"
  echo "  CLOUD_TOPIC   - recorded cloud topic    (default: /cloud_scan)"
  exit 1
}

echo "=== EllipseLIO rosbag pipeline ==="

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
fi

if [[ $# -eq 2 ]]; then
  DATASET_HOST_PATH="$1"
  BAG_OUTPUT_HOST="$2"
elif [[ $# -eq 0 ]]; then
  command -v zenity >/dev/null || {
    echo "Error: zenity is not available"
    exit 1
  }
  DATASET_HOST_PATH=$(zenity --file-selection --title="Select BAG file")
  BAG_OUTPUT_HOST=$(zenity --file-selection --directory --title="Select output directory")
else
  usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$BAG_OUTPUT_HOST" ]]; then
  echo "Error: no file or directory selected"
  exit 1
fi

if [[ ! -f "$DATASET_HOST_PATH" && ! -d "$DATASET_HOST_PATH" ]]; then
  echo "Error: BAG path does not exist: $DATASET_HOST_PATH"
  exit 1
fi

mkdir -p "$BAG_OUTPUT_HOST"

DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
BAG_OUTPUT_HOST=$(realpath "$BAG_OUTPUT_HOST")

echo "Input bag    : $DATASET_HOST_PATH"
echo "Output dir   : $BAG_OUTPUT_HOST"
echo "Config file  : $CONFIG_FILE"
echo "Odom topic   : $ODOM_TOPIC"
echo "Cloud topic  : $CLOUD_TOPIC"

xhost +local:docker >/dev/null

# ── Phase 1: run EllipseLIO + record output topics ────────────────────────────
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c '

    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash

    # ── Convert ROS 1 bag to ROS 2 format if needed ──
    if [[ "'"$DATASET_CONTAINER_PATH"'" == *.bag ]]; then
      echo "[convert] Converting ROS 1 bag to ROS 2 format..."
      echo "[convert] Input: '"$DATASET_CONTAINER_PATH"'"
      ls -la "'"$DATASET_CONTAINER_PATH"'" || { echo "[convert] ERROR: input bag not found!"; exit 1; }
      rm -rf "'"$DATASET_ROS2_PATH"'"
      rosbags-convert "'"$DATASET_CONTAINER_PATH"'" --dst "'"$DATASET_ROS2_PATH"'"
      if [[ ! -d "'"$DATASET_ROS2_PATH"'" ]]; then
        echo "[convert] ERROR: rosbags-convert failed! Output not created."
        exit 1
      fi
      ROS2_BAG="'"$DATASET_ROS2_PATH"'"
    else
      ROS2_BAG="'"$DATASET_CONTAINER_PATH"'"
    fi

    export ROS2_BAG
    echo "[convert] ROS 2 bag ready at: $ROS2_BAG"
    ls -la $ROS2_BAG/

    tmux new-session -d -s '"$TMUX_SESSION"'

    # ---------- PANE 0: EllipseLIO standalone launch ----------
    tmux send-keys -t '"$TMUX_SESSION"' '\''
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
sleep 2
ros2 launch ellipselio ellipselio_standalone.launch.py \
  config_file:='"$CONFIG_FILE"' \
  use_sim_time:=true \
  rviz:=false
'\'' C-m

    # ---------- PANE 1: RViz visualization ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "[rviz] launching RViz2..."
rviz2 -d /ros2_ws/install/ellipselio/share/ellipselio/rviz/ellipselio.rviz --ros-args -p use_sim_time:=true
'\'' C-m

    # ---------- PANE 2: ros2 bag record ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
rm -rf '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] start"
ros2 bag record '"$ODOM_TOPIC"' '"$CLOUD_TOPIC"' -o '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] exit"
'\'' C-m

    # ---------- PANE 3: ros2 bag play ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 5
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "[play] start"
ros2 bag play $ROS2_BAG --clock; tmux wait-for -S BAG_DONE;
echo "[play] done"
'\'' C-m

    # ---------- PANE 4: diagnostics ----------
    tmux split-window -h -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 8
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "=== ROS 2 DIAGNOSTICS ==="
echo ""
echo "--- Active topics ---"
ros2 topic list
echo ""
echo "--- Checking EllipseLIO output: '"$ODOM_TOPIC"' ---"
timeout 5 ros2 topic hz '"$ODOM_TOPIC"' 2>&1 &
echo ""
echo "--- Checking EllipseLIO output: '"$CLOUD_TOPIC"' ---"
timeout 5 ros2 topic hz '"$CLOUD_TOPIC"' 2>&1 &
wait
echo ""
echo "=== If output topics show nothing, verify lidar/imu topic names ==="
echo "=== in the chosen config file match topics inside the bag.       ==="
echo ""
echo "--- Node list ---"
ros2 node list
echo ""
echo "[diag] done — you can type ROS 2 commands here, e.g.:"
echo "  ros2 topic list"
echo "  ros2 topic echo '"$ODOM_TOPIC"'"
echo "  ros2 topic hz '"$CLOUD_TOPIC"'"
'\'' C-m

    # ---------- Control window ----------
    tmux new-window -t '"$TMUX_SESSION"' -n control '\''
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "[control] waiting for play end"
tmux wait-for BAG_DONE
echo "[control] bag playback finished — shutting down"

# Give EllipseLIO a moment to process remaining queued scans
sleep 3

# Graceful stop: Ctrl+C to each pane
# Pane layout: 0=ellipselio, 1=rviz, 2=recorder, 3=play, 4=diag
echo "[control] sending Ctrl+C to all panes..."
tmux send-keys -t '"$TMUX_SESSION"':0.2 C-c
sleep 1
tmux send-keys -t '"$TMUX_SESSION"':0.1 C-c
sleep 1
tmux send-keys -t '"$TMUX_SESSION"':0.0 C-c
sleep 3

# Force-kill by process name
echo "[control] force-killing remaining processes..."
pkill -9 ellipselio_mapping_node 2>/dev/null || true
pkill -9 component_container 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
sleep 1

echo "[control] terminating tmux"
tmux kill-server
'\''

    tmux attach -t '"$TMUX_SESSION"'
  '

# ── Phase 2: convert recorded bag to HDMapping session ────────────────────────
echo "=== Converting recorded bag to HDMapping session ==="

docker run -it --rm \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    set -e
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    ros2 run ellipselio-to-hdmapping listener \
      \"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME\" \
      \"$BAG_OUTPUT_CONTAINER/$HDMAPPING_OUT_NAME-EllipseLIO\" \
      \"$ODOM_TOPIC\" \
      \"$CLOUD_TOPIC\"
  "

echo "=== DONE ==="
