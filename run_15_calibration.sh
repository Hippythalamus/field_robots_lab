#!/usr/bin/env bash
# Run 15 sequential tank farm patrol missions for RideScan calibration.
#
# Each run:
#   1. Launches scenario_runner tank_farm_navigation in its own process
#      group (setsid) so we can later signal the whole tree.
#   2. Waits on /scenario_runner/complete (latched Bool, ROS-native signal).
#   3. Drains telemetry briefly so the recorder can flush.
#   4. Sends SIGINT to the entire process group. ros2 bag finalises its
#      MCAP + metadata.yaml here; this takes ~15-25s with multi-husky.
#   5. Verifies bag/ contains both bag_0.mcap AND metadata.yaml.

# --- Config ---
NUM_RUNS=15
START_RUN=1
COMPLETE_TIMEOUT_S=480
DRAIN_S=5
COOLDOWN_S=10
SHUTDOWN_WAIT_S=35           # bag finalisation can take 20-25s

EXPERIMENTS_DIR="$HOME/field_robots_lab_experiments"
LOG_DIR="$EXPERIMENTS_DIR/run_logs"
SUMMARY_FILE="$EXPERIMENTS_DIR/run_15_summary.txt"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --runs)  NUM_RUNS="$2"; shift 2 ;;
        --start) START_RUN="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

mkdir -p "$LOG_DIR"

cleanup_gazebo() {
    pkill -9 -f gzserver >/dev/null 2>&1
    pkill -9 -f gzclient >/dev/null 2>&1
    pkill -9 -f gazebo >/dev/null 2>&1
    pkill -9 -f spawner >/dev/null 2>&1
    pkill -9 -f robot_state_publisher >/dev/null 2>&1
    pkill -9 -f waypoint_navigator >/dev/null 2>&1
    pkill -9 -f mission_orchestrator >/dev/null 2>&1
    pkill -9 -f mission_starter >/dev/null 2>&1
    pkill -9 -f telemetry_recorder >/dev/null 2>&1
    pkill -9 -f recorder_node >/dev/null 2>&1
    pkill -9 -f "ros2 bag" >/dev/null 2>&1
    sleep 3
}

# Count inspection_completed log lines (one per tank, 6 per run total)
count_completed_inspections() {
    grep -c "INSPECTION " "$1" 2>/dev/null || echo 0
}

verify_bag() {
    local bag_dir="$1"
    if [ ! -d "$bag_dir" ]; then
        echo "MISSING"
        return
    fi
    local mcap_file
    mcap_file=$(ls "$bag_dir"/*.mcap 2>/dev/null | head -1)
    if [ -z "$mcap_file" ]; then
        echo "NO_MCAP"
        return
    fi
    if [ ! -f "$bag_dir/metadata.yaml" ]; then
        local size_mb
        size_mb=$(du -m "$mcap_file" | awk '{print $1}')
        echo "${size_mb}MB_NO_META"
        return
    fi
    local size_mb
    size_mb=$(du -m "$mcap_file" | awk '{print $1}')
    echo "${size_mb}MB"
}

source /opt/ros/humble/setup.bash
source "$HOME/field_robots_lab/install/setup.bash"
source "$HOME/husky_ws/install/setup.bash"

if [ "$START_RUN" -eq 1 ]; then
    cat > "$SUMMARY_FILE" <<EOF
=== Tank Farm Patrol — 15-Run Calibration Baseline ===
Started: $(date -Iseconds)
Mission: 3 Husky A200, 6 tanks (6 inspection_completed events per run,
                                12 events total counting started+completed)

run | status     | wall_time | completed_inspections | bag
----+------------+-----------+-----------------------+--------------
EOF
fi

for ((run=START_RUN; run<=NUM_RUNS; run++)); do
    run_id=$(printf "%02d" "$run")
    exp_name="tank_farm_run_${run_id}"
    log_file="$LOG_DIR/run_${run_id}.log"

    echo ""
    echo "======================================================================="
    echo "[$(date +%H:%M:%S)] Run ${run}/${NUM_RUNS} — $exp_name"
    echo "======================================================================="

    cleanup_gazebo
    rm -rf "$EXPERIMENTS_DIR/$exp_name"

    run_start=$(date +%s)

    # Spawn the launch in its own process group so we can signal the
    # whole tree cleanly. setsid puts it in a new session, then `kill
    # -INT -$PGID` reaches every descendant.
    setsid bash -c "exec ros2 launch scenario_runner tank_farm_navigation.launch.py \
        experiment_name:='$exp_name' > '$log_file' 2>&1" &
    LAUNCH_PID=$!

    # The PID we get is the new session leader; its PGID == its PID.
    LAUNCH_PGID="$LAUNCH_PID"

    echo "[$(date +%H:%M:%S)] Launch PGID $LAUNCH_PGID, log: $log_file"

    echo "[$(date +%H:%M:%S)] Waiting on /scenario_runner/complete (timeout ${COMPLETE_TIMEOUT_S}s)"

    if timeout "$COMPLETE_TIMEOUT_S" \
            ros2 topic echo --once /scenario_runner/complete std_msgs/msg/Bool \
            >/dev/null 2>&1; then
        completion_status="COMPLETE"
        echo "[$(date +%H:%M:%S)]   /scenario_runner/complete received"
    else
        completion_status="TIMEOUT"
        echo "[$(date +%H:%M:%S)]   TIMEOUT after ${COMPLETE_TIMEOUT_S}s"
    fi

    # Drain so recorder writes the last messages
    sleep "$DRAIN_S"

    # Signal the whole process group with SIGINT
    if kill -0 "-$LAUNCH_PGID" 2>/dev/null; then
        echo "[$(date +%H:%M:%S)]   Sending SIGINT to process group $LAUNCH_PGID"
        kill -INT "-$LAUNCH_PGID" 2>/dev/null
        # Wait up to SHUTDOWN_WAIT_S for bag finalisation
        for ((w=0; w<SHUTDOWN_WAIT_S; w++)); do
            sleep 1
            kill -0 "-$LAUNCH_PGID" 2>/dev/null || break
        done
        if kill -0 "-$LAUNCH_PGID" 2>/dev/null; then
            echo "[$(date +%H:%M:%S)]   Escalating to SIGTERM on group"
            kill -TERM "-$LAUNCH_PGID" 2>/dev/null
            sleep 3
        fi
        if kill -0 "-$LAUNCH_PGID" 2>/dev/null; then
            echo "[$(date +%H:%M:%S)]   Final SIGKILL on group"
            kill -KILL "-$LAUNCH_PGID" 2>/dev/null
        fi
    fi

    cleanup_gazebo

    run_end=$(date +%s)
    wall_time=$((run_end - run_start))
    inspections=$(count_completed_inspections "$log_file")
    bag_status=$(verify_bag "$EXPERIMENTS_DIR/$exp_name/bag")

    printf "%3d | %-10s | %4ds     | %21d | %s\n" \
        "$run" "$completion_status" "$wall_time" \
        "$inspections" "$bag_status" \
        >> "$SUMMARY_FILE"

    echo "[$(date +%H:%M:%S)] Run $run done — status=$completion_status, wall=${wall_time}s, inspections=$inspections, bag=$bag_status"

    if [ "$run" -lt "$NUM_RUNS" ]; then
        echo "[$(date +%H:%M:%S)] Cooldown ${COOLDOWN_S}s ..."
        sleep "$COOLDOWN_S"
    fi
done

echo "" >> "$SUMMARY_FILE"
echo "Finished: $(date -Iseconds)" >> "$SUMMARY_FILE"
echo ""
echo "======================================================================="
echo "All runs done. Summary:"
echo "======================================================================="
cat "$SUMMARY_FILE"
echo ""
echo "Bag files: $EXPERIMENTS_DIR/tank_farm_run_*/bag/"
echo "Logs:      $LOG_DIR/run_*.log"
echo "Summary:   $SUMMARY_FILE"
