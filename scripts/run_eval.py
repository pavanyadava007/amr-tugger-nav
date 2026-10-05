"""Run the evaluation: every condition gets the same seeded missions, split over parallel containers.

Each container runs its own stack (simulator + Nav2 + monitor + bridge) with --network none, so the DDS graphs are
isolated. Output: results/raw/<condition>_<shard>.jsonl (one line per mission + one summary line per shard).
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONDITIONS = {  # name -> (field_monitor enabled, k_approach)
    "off": ("false", "0.0"),
    "on": ("true", "0.0"),
    "on_k1.6": ("true", "1.6"),
}
TRACE = ["1:people", "2:people", "5:people", "1:none"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default="amr-tugger-nav:jazzy")
    ap.add_argument("--none", type=int, default=30, help="missions without people (seeds 0..n-1)")
    ap.add_argument("--people", type=int, default=60, help="missions with crossing people (seeds 0..n-1)")
    ap.add_argument("--shards", type=int, default=6, help="containers per condition")
    a = ap.parse_args()
    missions = [f"{s}:people" for s in range(a.people)] + [f"{s}:none" for s in range(a.none)]
    raw = ROOT / "results" / "raw"
    raw.mkdir(parents=True, exist_ok=True)
    for old in raw.glob("*.jsonl"):
        old.unlink()
    procs = []
    for cond, (enabled, k) in CONDITIONS.items():
        for i in range(a.shards):
            part = missions[i::a.shards]
            cmd = (
                ". /ws/ros2_ws/install/setup.sh && "
                f"(ros2 launch tugger_nav sim_stack.launch.py field_monitor:={enabled} k_approach:={k} "
                f"> /ws/results/raw/{cond}_{i}.launch.log 2>&1 &) && "
                f"python3 /ws/scripts/run_missions.py --condition {cond} --missions {','.join(part)} "
                f"--trace {','.join(TRACE)} --out /ws/results/raw/{cond}_{i}.jsonl"
            )
            log = open(raw / f"{cond}_{i}.runner.log", "w")
            procs.append((cond, i, subprocess.Popen(
                ["docker", "run", "--rm", "--network", "none", "-v", f"{ROOT}/results:/ws/results",
                 "-v", f"{ROOT}/scripts:/ws/scripts:ro", a.image, "bash", "-c", cmd],
                stdout=log, stderr=subprocess.STDOUT)))
    t0 = time.time()
    bad = 0
    for cond, i, p in procs:
        rc = p.wait()
        print(f"{cond} shard {i}: exit {rc} after {time.time() - t0:.0f} s", flush=True)
        bad += rc != 0
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
