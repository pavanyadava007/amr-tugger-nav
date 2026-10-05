"""Build site/data.json for the static Hugging Face Space from the recorded traces in results/raw/*.jsonl.

Field widths come from config/field_monitor.yaml (robot_width + 2 * lateral_margin, + 2 * warning_lateral);
field lengths are the ones the C++ monitor published at each moment.
"""
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "ros2_ws/src/tugger_nav"


def yaml_float(text, key):
    return float(re.search(rf"^\s*{key}:\s*([0-9.]+)", text, re.M).group(1))


def main():
    cfg = (PKG / "config/field_monitor.yaml").read_text()
    width = yaml_float(cfg, "robot_width")
    length = yaml_float(cfg, "robot_length")
    prot_w = width + 2 * yaml_float(cfg, "lateral_margin")
    warn_w = prot_w + 2 * yaml_float(cfg, "warning_lateral")
    layout = json.loads((PKG / "maps/layout.json").read_text())
    missions = []
    for f in sorted((ROOT / "results/raw").glob("*.jsonl")):
        for line in f.read_text().splitlines():
            d = json.loads(line)
            if "trace" not in d:
                continue
            tr = d["trace"]
            samples = tr["samples"][::2]  # 5 Hz is enough for the replay
            missions.append({
                "id": d["id"], "condition": d["condition"], "seed": d["seed"], "kind": d["kind"],
                "success": d["success"], "mission_time_s": d["mission_time_s"],
                "min_clearance_people_m": d["min_clearance_people_m"], "people_contacts": d["people_contacts"],
                "protective_stops": d["protective_stops"], "warning_entries": d["warning_entries"],
                "loop": tr["loop"], "columns": tr["columns"], "samples": samples,
            })
    order = {"off": 0, "on": 1, "on_k1.6": 2}
    missions.sort(key=lambda m: (m["kind"] != "people", m["seed"], order.get(m["condition"], 9)))
    data = {"layout": layout, "robot": {"length": length, "width": width},
            "fields": {"protective_width": prot_w, "warning_width": warn_w}, "missions": missions}
    out = ROOT / "site/data.json"
    out.write_text(json.dumps(data, separators=(",", ":")))
    print(f"{len(missions)} missions, {out.stat().st_size / 1024:.0f} KiB")


if __name__ == "__main__":
    main()
