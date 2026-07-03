#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_NAME='ellipselio_humble'
TMUX_SESSION='ros2_EllipseLIO'

DATASET_CONTAINER_PATH='/ros2_ws/dataset/input.bag'
DATASET_ROS2_PATH='/tmp/dataset_ros2'
BAG_OUTPUT_CONTAINER='/ros2_ws/recordings'
WRAPPER_CONFIG_CONTAINER='/ros2_ws/wrapper_config'
WRAPPER_CONFIG_HOST="${SCRIPT_DIR}/config"

RECORDED_BAG_NAME="recorded-EllipseLIO"
HDMAPPING_OUT_NAME="output_hdmapping"

# EllipseLIO publishes these by default (see ellipselio/src/map_processing.cpp).
ODOM_TOPIC=${ODOM_TOPIC:-/ellipselio_odom}
CLOUD_TOPIC=${CLOUD_TOPIC:-/cloud_scan}

# This wrapper is the Bunker-DVI-Dataset-reg-1 branch — the dataset uses a
# Livox Mid-360 (ROS 1 livox_ros_driver layout). The matching config lives in
# config/livox_reg.yaml and is mounted read-only into the container. Override
# with CONFIG_FILE=<some_upstream_yaml> to fall back to one of the configs that
# ship inside the ellipselio package (qt64_spires.yaml, vlp16_*, os*_*).
CONFIG_FILE=${CONFIG_FILE:-livox_reg.yaml}

# Wrapper configs override the upstream ellipselio package share directory when
# the requested file exists in this repo's config/ directory.
if [[ -f "${WRAPPER_CONFIG_HOST}/${CONFIG_FILE}" ]]; then
  USE_WRAPPER_CONFIG=1
else
  USE_WRAPPER_CONFIG=0
fi

if [[ "${USE_WRAPPER_CONFIG}" == "1" ]] && \
  grep -qE 'pointcloud_native' "${WRAPPER_CONFIG_HOST}/${CONFIG_FILE}"; then
  RUN_LIVOX_BRIDGE=1
else
  RUN_LIVOX_BRIDGE=0
fi

usage() {
  echo "Usage:"
  echo "  $0 <input.bag | ros2_bag_dir> <output_dir>"
  echo
  echo "  input.bag    : ROS 1 bag file — converted to ROS 2 format on the fly"
  echo "  ros2_bag_dir : ROS 2 bag directory — played directly, no conversion"
  echo "                 (this is what the HDMapping orchestration passes)"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  echo
  echo "Environment variables:"
  echo "  CONFIG_FILE   - ellipselio config file name"
  echo "                  default (this branch): livox_reg.yaml (from config/)"
  echo "                  upstream-shipped: os64_ncd.yaml, os128_ncd.yaml,"
  echo "                                    os64_geode.yaml, qt64_spires.yaml,"
  echo "                                    vlp16_bot.yaml, vlp16_geode.yaml,"
  echo "                                    vlp16_graco.yaml"
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

# Orchestration convention: a directory input is an already-converted ROS 2 bag
# (played directly); a file input is a ROS 1 .bag (converted on the fly).
if [[ -d "$DATASET_HOST_PATH" ]]; then
  INPUT_IS_DIR=1
else
  INPUT_IS_DIR=0
fi

echo "Input bag         : $DATASET_HOST_PATH"
echo "Input type        : $([[ $INPUT_IS_DIR == 1 ]] && echo 'ROS 2 bag directory (no conversion)' || echo 'ROS 1 bag file (convert to ROS 2)')"
echo "Output dir        : $BAG_OUTPUT_HOST"
echo "Config file       : $CONFIG_FILE"
echo "Wrapper config    : $([[ $USE_WRAPPER_CONFIG == 1 ]] && echo "${WRAPPER_CONFIG_HOST}/${CONFIG_FILE} -> ${WRAPPER_CONFIG_CONTAINER}/${CONFIG_FILE}" || echo '(using upstream package config)')"
echo "Livox bridge node : $([[ $RUN_LIVOX_BRIDGE == 1 ]] && echo 'yes' || echo 'no')"
echo "Odom topic        : $ODOM_TOPIC"
echo "Cloud topic       : $CLOUD_TOPIC"

xhost +local:docker >/dev/null

WRAPPER_CONFIG_MOUNT=()
if [[ "${USE_WRAPPER_CONFIG}" == "1" ]]; then
  WRAPPER_CONFIG_MOUNT=(-v "${WRAPPER_CONFIG_HOST}":"${WRAPPER_CONFIG_CONTAINER}":ro)
  EXTRA_LAUNCH_ARGS="config_path:=${WRAPPER_CONFIG_CONTAINER}"
else
  EXTRA_LAUNCH_ARGS=""
fi

# ── Phase 1: run EllipseLIO + record output topics ────────────────────────────
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_HOME=/tmp/.ros \
  -e CONFIG_FILE="${CONFIG_FILE}" \
  -e INPUT_IS_DIR="${INPUT_IS_DIR}" \
  -e USE_WRAPPER_CONFIG="${USE_WRAPPER_CONFIG}" \
  -e RUN_LIVOX_BRIDGE="${RUN_LIVOX_BRIDGE}" \
  -e WRAPPER_CONFIG_CONTAINER="${WRAPPER_CONFIG_CONTAINER}" \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "${WRAPPER_CONFIG_MOUNT[@]}" \
  "$IMAGE_NAME" \
  /bin/bash -c '

    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash

    if [[ "$USE_WRAPPER_CONFIG" == "1" ]]; then
      if [[ ! -f "$WRAPPER_CONFIG_CONTAINER/$CONFIG_FILE" ]]; then
        echo "[config] ERROR: mounted config missing: $WRAPPER_CONFIG_CONTAINER/$CONFIG_FILE"
        exit 1
      fi
      echo "[config] mounted: $WRAPPER_CONFIG_CONTAINER/$CONFIG_FILE"
      ls -la "$WRAPPER_CONFIG_CONTAINER"
    fi

    # ── Convert ROS 1 bag to ROS 2 format if needed ──
    # INPUT_IS_DIR is decided on the host by the real input type: a directory
    # is an already-converted ROS 2 bag (orchestration passes these), a file
    # is a ROS 1 .bag that still needs conversion. (The old check tested the
    # fixed container mount path, which always ends in .bag, so it converted
    # unconditionally and broke on ROS 2 directory inputs.)
    if [[ "$INPUT_IS_DIR" == "1" ]]; then
      echo "[convert] Input is already a ROS 2 bag directory — skipping conversion."
      ROS2_BAG="'"$DATASET_CONTAINER_PATH"'"
    else
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
    fi

    export ROS2_BAG
    echo "[convert] ROS 2 bag ready at: $ROS2_BAG"
    ls -la $ROS2_BAG/

    # Path inside the container for the diagnostic log dir.
    # MUST be exported so that the fresh shells launched by `tmux send-keys`
    # in each pane inherit it (otherwise tee would write to /<name>.log).
    export LOG_DIR='"$BAG_OUTPUT_CONTAINER"'/logs
    mkdir -p "$LOG_DIR"
    rm -f "$LOG_DIR"/*.log
    echo "[setup] diagnostic logs -> $LOG_DIR"

    tmux new-session -d -s '"$TMUX_SESSION"'

    # ---------- PANE 0: EllipseLIO standalone launch ----------
    tmux send-keys -t '"$TMUX_SESSION"' '\''
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
sleep 2
stdbuf -oL -eL ros2 launch ellipselio ellipselio_standalone.launch.py \
  config_file:='"$CONFIG_FILE"' \
  '"$EXTRA_LAUNCH_ARGS"' \
  use_sim_time:=true \
  rviz:=false 2>&1 | tee "$LOG_DIR"/ellipselio.log
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

    # ---------- PANE 5: optional Livox PointCloud2 format bridge ----------
    if [[ "'"$RUN_LIVOX_BRIDGE"'" == "1" ]]; then
      tmux split-window -h -t '"$TMUX_SESSION"'
      tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "[bridge] livox_format_bridge: /livox/pointcloud (float time) -> /livox/pointcloud_native (double timestamp)"
stdbuf -oL -eL ros2 run ellipselio-to-hdmapping livox_format_bridge --ros-args -p use_sim_time:=true 2>&1 | tee "$LOG_DIR"/bridge.log
'\'' C-m
    fi

    # ---------- PANE 4: diagnostics ----------
    tmux split-window -h -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 8
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
(
echo "=== ROS 2 DIAGNOSTICS ==="
echo ""
echo "--- Active topics ---"
ros2 topic list
echo ""
if [[ "'"$RUN_LIVOX_BRIDGE"'" == "1" ]]; then
  echo "--- Bridge input rate /livox/pointcloud ---"
  timeout 5 ros2 topic hz /livox/pointcloud 2>&1 | tail -3 &
  echo "--- Bridge output rate /livox/pointcloud_native (must be > 0!) ---"
  timeout 5 ros2 topic hz /livox/pointcloud_native 2>&1 | tail -3 &
  wait
  echo ""
fi
echo "--- Algorithm input topics ---"
for t in /livox/imu /livox/pointcloud_native; do
  echo "  hz $t:"
  timeout 4 ros2 topic hz $t 2>&1 | tail -2
done
echo ""
echo "--- EllipseLIO output: '"$ODOM_TOPIC"' ---"
timeout 5 ros2 topic hz '"$ODOM_TOPIC"' 2>&1 | tail -3
echo "--- EllipseLIO output: '"$CLOUD_TOPIC"' ---"
timeout 5 ros2 topic hz '"$CLOUD_TOPIC"' 2>&1 | tail -3
echo ""
echo "--- Node list ---"
ros2 node list
echo ""
echo "--- ellipselio node params (lidar/imu) ---"
NODE=$(ros2 node list 2>/dev/null | grep -i ellipselio | head -1)
if [[ -n "$NODE" ]]; then
  for p in lidar.topic lidar.type lidar.rate lidar.scan_lines imu.topic imu.rate; do
    v=$(ros2 param get $NODE $p 2>/dev/null | tail -1)
    printf "  %-22s = %s\n" "$p" "$v"
  done
else
  echo "  (ellipselio node not found!)"
fi
) 2>&1 | tee "$LOG_DIR"/diag.log
echo ""
echo "[diag] done. Type ROS 2 commands here, e.g. ros2 topic echo '"$ODOM_TOPIC"'"
'\'' C-m

    # ---------- Control window ----------
    # This window is what the user attaches to. All the noisy work
    # (launch / rviz / record / play / bridge / diag) is on window 0.
    tmux new-window -t '"$TMUX_SESSION"' -n control '\''
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
clear
LOG_DIR='"$BAG_OUTPUT_CONTAINER"'/logs
echo "EllipseLIO running. Waiting for playback to end..."
echo "(logs -> $LOG_DIR/{ellipselio,bridge,diag}.log)"
tmux wait-for BAG_DONE
echo "[control] bag playback finished — collecting diagnostics"

# Give the algorithm a moment to flush queued scans.
sleep 3

echo ""
echo "================ ALGORITHM DIAGNOSTIC SUMMARY ================"
if [[ -s "$LOG_DIR/ellipselio.log" ]]; then
  echo ""
  echo "--- ellipselio.log: last 5 ERROR / WARN lines ---"
  grep -E "\\[ERROR\\]|\\[WARN\\]|Lidar has no new data|IMU has no new data|No synced measurements|Lidar start time is before|IMU start time later than" \
      "$LOG_DIR/ellipselio.log" | tail -5
  echo ""
  echo "--- ellipselio.log: last 15 lines (any) ---"
  tail -15 "$LOG_DIR/ellipselio.log"
else
  echo "(no ellipselio.log content)"
fi
if [[ -s "$LOG_DIR/bridge.log" ]]; then
  echo ""
  echo "--- bridge.log: last 8 lines ---"
  tail -8 "$LOG_DIR/bridge.log"
fi
if [[ -s "$LOG_DIR/diag.log" ]]; then
  echo ""
  echo "--- diag.log (full) ---"
  cat "$LOG_DIR/diag.log"
fi
echo "============================================================="
echo ""

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
