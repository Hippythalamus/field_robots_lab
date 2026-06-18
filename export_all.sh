#!/usr/bin/env bash
# Export all 15 mission bags to RideScan-compatible CSV files.
# Three robots per run -> 45 CSVs total.
#
# Output structure:
#   csv_out/
#     robot_1/
#       tank_farm_run_01_robot_1.csv
#       ...
#       tank_farm_run_15_robot_1.csv
#     robot_2/
#       ...
#     robot_3/
#       ...
#
# Each CSV is validated by mcap_to_csv.py; failures are listed at the end.

EXPERIMENTS_DIR="$HOME/field_robots_lab_experiments"
OUT_DIR="$HOME/field_robots_lab_experiments/csv_out"
EXPORTER="$HOME/field_robots_lab/mcap_to_csv.py"
ROBOTS=(robot_1 robot_2 robot_3)

source /opt/ros/humble/setup.bash
source "$HOME/field_robots_lab/install/setup.bash"

mkdir -p "$OUT_DIR"

failures=()
successes=0

for ((run=1; run<=15; run++)); do
    run_id=$(printf "%02d" "$run")
    bag_dir="$EXPERIMENTS_DIR/tank_farm_run_${run_id}/bag"

    if [ ! -d "$bag_dir" ]; then
        failures+=("run ${run_id}: bag dir missing")
        continue
    fi

    for robot in "${ROBOTS[@]}"; do
        out_subdir="$OUT_DIR/$robot"
        mkdir -p "$out_subdir"
        out_file="$out_subdir/tank_farm_run_${run_id}_${robot}.csv"

        echo "[run ${run_id} / ${robot}] -> $out_file"

        if python3 "$EXPORTER" \
            --bag "$bag_dir" \
            --robot "$robot" \
            --out "$out_file" 2>&1; then
            successes=$((successes + 1))
        else
            failures+=("run ${run_id} / ${robot}: export failed")
        fi
    done
done

echo ""
echo "======================================================================="
echo "Summary"
echo "======================================================================="
echo "Successful exports: $successes"
echo "Failed exports:     ${#failures[@]}"
if [ ${#failures[@]} -gt 0 ]; then
    echo ""
    echo "Failures:"
    for f in "${failures[@]}"; do
        echo "  - $f"
    done
fi

echo ""
echo "CSV files in: $OUT_DIR"
ls -la "$OUT_DIR"/*/
