#!/usr/bin/env bash
# Runs the Gazebo bench headless once per Livox mount tilt and checks that every tilt gives the
# same detection in base_link as the flat mount (and that each run passes on its own).
#
#   rosrun detect_pallet_3d_gazebo run_tilt_matrix.sh [front|edge] [extra roslaunch args...]
#   e.g. rosrun detect_pallet_3d_gazebo run_tilt_matrix.sh edge pallet_yaw:=0.0
#
# TILTS="roll pitch;..." overrides the mounts (rad), e.g. TILTS="0 0;0 0.2" run_tilt_matrix.sh
# TOL_POS (m, default 0.01) / TOL_YAW_DEG (default 1.5): max difference of a tilt's mean to flat.
# The ray pattern on the pallet differs per tilt, so results agree within noise, not bit-exact.
set -u

SCENARIO=${1:-front}
shift || true
TILTS=${TILTS:-"0 0;0 0.3491;0 0.15;0 -0.15;0.1 -0.1"}
TOL_POS=${TOL_POS:-0.01}
TOL_YAW_DEG=${TOL_YAW_DEG:-1.5}

OUT_DIR=$(mktemp -d /tmp/livox_tilt_XXXX)
RESULTS="$OUT_DIR/results.jsonl"
echo "scenario=$SCENARIO  logs and results in $OUT_DIR"

i=0
IFS=';' read -ra MOUNTS <<< "$TILTS"
for m in "${MOUNTS[@]}"; do
  read -r R P <<< "$m"
  i=$((i + 1))
  echo "--- run $i: livox_roll=$R livox_pitch=$P"
  roslaunch detect_pallet_3d_gazebo pallet_detection_sim.launch \
    gui:=false rviz:=false eval:=true exit_when_done:=true \
    scenario:="$SCENARIO" livox_roll:="$R" livox_pitch:="$P" result_file:="$RESULTS" "$@" \
    > "$OUT_DIR/run_${i}.log" 2>&1
  grep "\[eval\]" "$OUT_DIR/run_${i}.log" | grep -v "waiting\|enabled" | sed 's/^/    /'
  # let gzserver / rosmaster exit before the next run
  sleep 3
done

python3 - "$RESULTS" "$TOL_POS" "$TOL_YAW_DEG" <<'EOF'
import json, sys
path, tol_pos, tol_yaw = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
try:
    runs = [json.loads(l) for l in open(path) if l.strip()]
except FileNotFoundError:
    sys.exit("no results written (see the run logs)")
if not runs:
    sys.exit("no results written (see the run logs)")
flat = next((r for r in runs if r["livox_roll"] == 0 and r["livox_pitch"] == 0), runs[0])
ok = True
print("\n%-8s %-8s %-6s %5s %10s %10s %9s  %s" % ("roll", "pitch", "det", "n", "d_perp m", "d_along m", "d_yaw deg", "result"))
for r in runs:
    for det in ("front", "edge"):
        if det not in r:
            continue
        a, f = r[det], flat.get(det)
        if not a["n"] or not f or not f["n"]:
            print("%-8.3f %-8.3f %-6s %5d %10s %10s %9s  FAIL (no detection)" % (r["livox_roll"], r["livox_pitch"], det, a["n"], "-", "-", "-"))
            ok = False
            continue
        dp = a["perp"]["mean"] - f["perp"]["mean"]
        dl = a["along"]["mean"] - f["along"]["mean"]
        dy = a["yaw"]["mean"] - f["yaw"]["mean"]
        same = abs(dp) <= tol_pos and abs(dy) <= tol_yaw and (det == "front" or abs(dl) <= tol_pos)
        good = same and a["pass"]
        ok &= good
        print("%-8.3f %-8.3f %-6s %5d %+10.4f %+10.4f %+9.2f  %s%s" % (
            r["livox_roll"], r["livox_pitch"], det, a["n"], dp, dl, dy,
            "PASS" if good else "FAIL", "" if a["pass"] else " (run itself failed)"))
print("\nd_* = tilt mean - flat mean (front: d_along not scored, it is the visible-part middle)")
print("ALL PASS" if ok else "SOME FAILED")
sys.exit(0 if ok else 1)
EOF
