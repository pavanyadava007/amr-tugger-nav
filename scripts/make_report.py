"""Aggregate results/raw/*.jsonl (+ tests.json, env.json) into results/summary.json and docs/RESULTS.md.

Every number in docs/RESULTS.md comes from this script. Do not edit RESULTS.md by hand.
"""
import json
import statistics as st
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RES = ROOT / "results"
ORDER = ["off", "on", "on_k1.6"]
LABEL = {"off": "OFF (Nav2 alone)", "on": "ON (k_approach = 0)", "on_k1.6": "ON (k_approach = 1.6 m/s)"}
MOVING = 0.05  # m/s: a contact counts as "robot moving" above this speed


def pct(xs, q):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def r(x, n=2):
    return None if x is None else round(x, n)


def load():
    recs, shards = [], []
    for f in sorted((RES / "raw").glob("*.jsonl")):
        for line in f.read_text().splitlines():
            d = json.loads(line)
            (shards if d.get("summary") else recs).append(d)
    return recs, shards


def summarise(rs):
    ok = [x for x in rs if x["success"]]
    times = [x["mission_time_s"] for x in ok]
    clear_p = [x["min_clearance_people_m"] for x in rs if x["min_clearance_people_m"] is not None]
    contacts = [c for x in rs for c in x["contacts"]]
    stops = [s for x in rs for s in x["stops"]]
    return {
        "missions": len(rs),
        "success": len(ok),
        "success_rate": r(len(ok) / len(rs), 4) if rs else None,
        "failures": sorted({e for x in rs if not x["success"] for e in x["errors"]} or []),
        "mission_time_mean_s": r(st.mean(times)) if times else None,
        "mission_time_median_s": r(st.median(times)) if times else None,
        "mission_time_p95_s": r(pct(times, 0.95)),
        "min_clearance_people_min_m": r(min(clear_p), 3) if clear_p else None,
        "min_clearance_people_median_m": r(st.median(clear_p), 3) if clear_p else None,
        "min_clearance_static_min_m": r(min(x["min_clearance_static_m"] for x in rs), 3) if rs else None,
        "missions_with_person_contact": sum(1 for x in rs if x["people_contacts"] > 0),
        "person_contacts": len(contacts),
        "person_contacts_robot_moving": sum(1 for c in contacts if abs(c["robot_speed"]) > MOVING),
        "person_contacts_front_moving": sum(1 for c in contacts if c["front"] and abs(c["robot_speed"]) > MOVING),
        "person_contacts_side_or_rear": sum(1 for c in contacts if not c["front"]),
        "max_robot_speed_at_front_contact_mps": r(max((abs(c["robot_speed"]) for c in contacts if c["front"]), default=None), 3),
        "max_robot_speed_at_contact_mps": r(max((abs(c["robot_speed"]) for c in contacts), default=None), 3),
        "static_contacts": sum(x["static_contacts"] for x in rs),
        "protective_stops": len(stops),
        "protective_stops_person": sum(1 for s in stops if s["cause"] == "person"),
        "false_stops": sum(1 for s in stops if s["cause"] != "person"),
        "false_stops_static": sum(1 for s in stops if s["cause"] == "static"),
        "warning_entries": sum(x["warning_entries"] for x in rs),
        "protective_intrusions": sum(x["protective_intrusions"] for x in rs),
        "time_in_protective_stop_mean_s": r(st.mean(x["time_in_protective_stop_s"] for x in rs)) if rs else None,
        "rtf_mean": r(st.mean(x["rtf"] for x in rs if x["rtf"]), 3) if rs else None,
        "rtf_min": r(min(x["rtf"] for x in rs if x["rtf"]), 3) if rs else None,
    }


def main():
    recs, shards = load()
    by = {c: [x for x in recs if x["condition"] == c] for c in ORDER}
    summary = {"conditions": {}, "paired": {}, "env": None, "tests": None}
    for c in ORDER:
        rs = by[c]
        if not rs:
            continue
        evals = [v for x in rs for v in x["eval_time_us"]]
        cpu = {}
        for s in (s for s in shards if s["condition"] == c):
            for k, v in s["cpu_cores_by_process"].items():
                cpu.setdefault(k, []).append(v)
        summary["conditions"][c] = {
            "all": summarise(rs),
            "people": summarise([x for x in rs if x["kind"] == "people"]),
            "none": summarise([x for x in rs if x["kind"] == "none"]),
            "field_eval_us_p50": r(pct(evals, 0.5), 1), "field_eval_us_p99": r(pct(evals, 0.99), 1),
            "field_eval_scans": len(evals),
            "cpu_cores_mean_by_process": {k: r(st.mean(v), 3) for k, v in sorted(cpu.items()) if st.mean(v) >= 0.001},
        }
    # Paired comparison on the same seeds (people missions).
    off = {x["seed"]: x for x in by["off"] if x["kind"] == "people"}
    for c in ORDER[1:]:
        pairs = [(off[x["seed"]], x) for x in by[c] if x["kind"] == "people" and x["seed"] in off]
        if not pairs:
            continue
        both = [(a, b) for a, b in pairs if a["success"] and b["success"]]
        cl = [(a["min_clearance_people_m"], b["min_clearance_people_m"]) for a, b in pairs
              if a["min_clearance_people_m"] is not None and b["min_clearance_people_m"] is not None]
        summary["paired"][c] = {
            "pairs": len(pairs),
            "both_successful": len(both),
            "mission_time_delta_mean_s": r(st.mean(b["mission_time_s"] - a["mission_time_s"] for a, b in both)) if both else None,
            "clearance_higher_than_off": sum(1 for a, b in cl if b > a + 1e-6),
            "clearance_lower_than_off": sum(1 for a, b in cl if b < a - 1e-6),
            "clearance_delta_median_m": r(st.median(b - a for a, b in cl), 3) if cl else None,
            "seeds_with_contact_off": sum(1 for a, b in pairs if a["people_contacts"] > 0),
            "seeds_with_contact_on": sum(1 for a, b in pairs if b["people_contacts"] > 0),
        }
    for name in ("env", "tests"):
        p = RES / f"{name}.json"
        if p.exists():
            summary[name] = json.loads(p.read_text())
    (RES / "summary.json").write_text(json.dumps(summary, indent=2))
    write_md(summary)
    print("wrote results/summary.json and docs/RESULTS.md")


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:g}"
    return str(v)


def write_md(s):
    conds = [c for c in ORDER if c in s["conditions"]]
    L = ["# Results", "", "Generated by `scripts/make_report.py` from `results/raw/*.jsonl`; do not edit by hand.", ""]
    env = s.get("env") or {}
    L += [f"**Everything is simulation.** Measured on {env.get('cpu_model', 'unknown CPU')} "
          f"({env.get('logical_cpus', '?')} logical CPUs), inside Docker (ROS 2 {env.get('ros_distro', '?')}), "
          "no GPU used. Each condition ran in parallel containers with the simulator paced at real time.", ""]
    t = s.get("tests")
    if t:
        g = sum(x["tests"] for x in t["suites"] if x["kind"] == "gtest")
        lt = sum(x["tests"] for x in t["suites"] if x["kind"] == "launch_testing")
        L += [f"**Tests:** {t['total']} test cases, {t['failures']} failures ({g} GoogleTest, {lt} launch_testing).", ""]
    first = s["conditions"][conds[0]]
    L += [f"**Missions:** {first['all']['missions']} per condition "
          f"({first['people']['missions']} with crossing people, {first['none']['missions']} without), "
          "same seeds in every condition. One mission = one VDA 5050 order for the full tugger loop (11 nodes, 60 m).", ""]

    def table(part, title):
        rows = [
            ("Missions", "missions"), ("Mission success", "success"), ("Mission time mean [s]", "mission_time_mean_s"),
            ("Mission time p95 [s]", "mission_time_p95_s"),
            ("Min clearance to a person, worst mission [m]", "min_clearance_people_min_m"),
            ("Min clearance to a person, median mission [m]", "min_clearance_people_median_m"),
            ("Missions with a person contact", "missions_with_person_contact"),
            ("Person contacts (total)", "person_contacts"),
            ("Person contacts with the robot moving > 0.05 m/s", "person_contacts_robot_moving"),
            ("  robot moving and person ahead of its front edge", "person_contacts_front_moving"),
            ("  person at the robot's side or rear", "person_contacts_side_or_rear"),
            ("Max robot speed at a contact [m/s]", "max_robot_speed_at_contact_mps"),
            ("Max robot speed at a front contact [m/s]", "max_robot_speed_at_front_contact_mps"),
            ("Min clearance to racks/walls [m]", "min_clearance_static_min_m"),
            ("Protective stops", "protective_stops"), ("  of which caused by a person", "protective_stops_person"),
            ("  false stops (no person at the trigger point)", "false_stops"),
            ("Warning slowdowns", "warning_entries"),
            ("Protective field intrusions observed", "protective_intrusions"),
            ("Time in protective stop per mission, mean [s]", "time_in_protective_stop_mean_s"),
            ("Real-time factor, mean / min", None),
        ]
        out = [f"### {title}", "", "| metric | " + " | ".join(LABEL[c] for c in conds) + " |",
               "|---|" + "---|" * len(conds)]
        for name, key in rows:
            vals = []
            for c in conds:
                d = s["conditions"][c][part]
                if key is None:
                    vals.append(f"{fmt(d['rtf_mean'])} / {fmt(d['rtf_min'])}")
                elif key == "success":
                    vals.append(f"{d['success']}/{d['missions']}")
                elif c == "off" and key in ("protective_stops", "protective_stops_person", "false_stops", "warning_entries",
                                            "time_in_protective_stop_mean_s"):
                    vals.append("- (observe only)")
                else:
                    vals.append(fmt(d[key]))
            out.append(f"| {name} | " + " | ".join(vals) + " |")
        return out + [""]

    L += ["## Ablation: field monitor ON vs OFF", ""]
    L += table("people", "Missions with crossing people")
    L += table("none", "Missions without people")
    L += ["## Paired comparison (same people seeds)", "", "| | " + " | ".join(LABEL[c] for c in conds[1:]) + " |",
          "|---|" + "---|" * len(conds[1:])]
    prow = [("Pairs (seed run in OFF and in this condition)", "pairs"),
            ("Mission time change vs OFF, mean [s] (both successful)", "mission_time_delta_mean_s"),
            ("Seeds where min person clearance is higher than OFF", "clearance_higher_than_off"),
            ("Seeds where it is lower than OFF", "clearance_lower_than_off"),
            ("Median clearance change vs OFF [m]", "clearance_delta_median_m"),
            ("Seeds with a person contact: OFF", "seeds_with_contact_off"),
            ("Seeds with a person contact: this condition", "seeds_with_contact_on")]
    for name, key in prow:
        L.append(f"| {name} | " + " | ".join(fmt(s["paired"].get(c, {}).get(key)) for c in conds[1:]) + " |")
    L += ["", "## Compute", "", "| | " + " | ".join(LABEL[c] for c in conds) + " |", "|---|" + "---|" * len(conds)]
    L.append("| Field check per scan, p50 / p99 [us] | " + " | ".join(
        f"{fmt(s['conditions'][c]['field_eval_us_p50'])} / {fmt(s['conditions'][c]['field_eval_us_p99'])}" for c in conds) + " |")
    L.append("| Scans evaluated | " + " | ".join(fmt(s["conditions"][c]["field_eval_scans"]) for c in conds) + " |")
    procs = sorted({p for c in conds for p in s["conditions"][c]["cpu_cores_mean_by_process"]})
    for p in procs:
        L.append(f"| CPU cores, {p} | " + " | ".join(
            fmt(s["conditions"][c]["cpu_cores_mean_by_process"].get(p)) for c in conds) + " |")
    L += ["", "## Definitions", "",
          "- **Mission success**: the VDA 5050 state reports the order's last node (sequenceId 20) as `lastNodeId`, "
          "no `nodeStates`, not driving and no errors, within 240 s of simulated time.",
          "- **Mission time**: simulated time from publishing the order to that state (successful missions).",
          "- **Min clearance to a person**: smallest distance between the robot footprint rectangle (1.0 x 0.7 m) and "
          "a person disc (radius 0.25 m) over the mission, computed by the simulator every 10 ms from ground truth. "
          "0 means contact.",
          "- **Person contact**: a step where that distance becomes <= 0 after being > 0. People are scripted and do "
          "not avoid the robot, so a person can walk into a stopped robot; the speed at contact separates the cases. "
          "**Front contact**: the person's centre is ahead of the robot's front edge in the robot frame (the robot drove "
          "into the person); otherwise the person reached the side or rear, which a forward field cannot cover.",
          "- **Protective stop**: a transition of the monitor into PROTECTIVE_STOP. **False stop**: a protective stop "
          "whose nearest violating scan point is not within 0.3 m of a person's surface (ground truth at that time), "
          "i.e. it was triggered by a rack, wall or machine.",
          "- **Warning slowdown**: a transition into WARNING (speed capped at 0.3 m/s).",
          "- **Protective field intrusion**: a scan that starts a protective field violation; counted in every "
          "condition, including OFF where the monitor only observes.",
          "- **Real-time factor**: simulated time / wall time per mission (the simulator is paced at 1.0; Nav2's "
          "controller runs on a wall-clock rate, so faster than real time would change its behaviour).",
          "- **CPU cores**: CPU seconds of each process in the container over the shard / wall seconds, mean over shards.",
          ""]
    (ROOT / "docs").mkdir(exist_ok=True)
    (ROOT / "docs" / "RESULTS.md").write_text("\n".join(L))
    write_readme_block(s, conds, table("people", f"Missions with crossing people ({first['people']['missions']} per condition)"))


def write_readme_block(s, conds, people_table):
    """Replace the block between the results markers in README.md with numbers from the summary."""
    t = s.get("tests") or {}
    env = s.get("env") or {}
    first = s["conditions"][conds[0]]["all"]["missions"]
    lines = [f"Measured in simulation on {env.get('cpu_model', 'unknown CPU')} inside Docker; {first} missions per "
             f"condition on the same seeds. Tests: {t.get('total', '?')} cases, {t.get('failures', '?')} failures.", ""]
    lines += ["**" + people_table[0].lstrip("# ") + "**", ""] + people_table[2:]
    readme = ROOT / "README.md"
    text = readme.read_text()
    a, b = text.index("<!-- results:start -->"), text.index("<!-- results:end -->")
    readme.write_text(text[:a] + "<!-- results:start -->\n" + "\n".join(lines).rstrip() + "\n" + text[b:])


if __name__ == "__main__":
    main()
