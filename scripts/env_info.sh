#!/bin/sh
# Record where the numbers were measured (run inside the container).
python3 - <<'P'
import json, os, platform, subprocess
cpu = [l.split(":", 1)[1].strip() for l in subprocess.run(["lscpu"], capture_output=True, text=True).stdout.splitlines()
       if l.startswith("Model name")]
json.dump({"cpu_model": cpu[0] if cpu else "unknown", "logical_cpus": os.cpu_count(), "kernel": platform.release(),
           "ros_distro": os.environ.get("ROS_DISTRO"), "python": platform.python_version(), "in_docker": True},
          open("/ws/results/env.json", "w"), indent=2)
P
