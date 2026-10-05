"""Run tugger loop missions against a running stack (inside the container) and write one JSON line per mission.

Each mission: reset the simulator (robot at node A, scripted people for the seed), send a VDA 5050 order for the
whole loop, wait until the VDA 5050 state reports it finished (or 240 s simulated time), then record what the
simulator's ground truth and the field monitor's status saw.
"""
import argparse
import json
import math
import os
import random
import time

import rclpy
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Empty
from tugger_msgs.msg import FieldStatus

try:
    from nav2_msgs.srv import ClearEntireCostmap
except ImportError:  # pragma: no cover
    ClearEntireCostmap = None

TIMEOUT_SIM_S = 240.0
PERSON_RADIUS = 0.25


def make_scenario(layout, seed, kind):
    """Deterministic scenario for (seed, kind): loop direction and up to three people at the rack-gap sites."""
    rng = random.Random(seed * 7919 + (1 if kind == "people" else 0))
    loop = list(layout["loop"])
    ccw = rng.random() < 0.5
    if not ccw:
        loop = loop[::-1]
    nodes = layout["nodes"]
    first, second = nodes[loop[0]], nodes[loop[1]]
    yaw = math.atan2(second[1] - first[1], second[0] - first[0])
    people = []
    if kind == "people":
        for site_name in rng.sample(sorted(layout["sites"]), 3):
            site = layout["sites"][site_name]
            a, b = (site["hidden"], site["far"]) if rng.random() < 0.5 else (site["far"], site["hidden"])
            mid = site["path"]
            standing = rng.random() < 0.3
            wps = [[a[0], a[1], 0.0]]
            if standing:
                wps.append([mid[0] + rng.uniform(-0.3, 0.3), mid[1] + rng.uniform(-0.6, 0.6), round(rng.uniform(3, 8), 2)])
            wps.append([b[0], b[1], 0.0])
            people.append({"site": site_name, "kind": "standing" if standing else "crossing", "waypoints": wps,
                           "speed": round(rng.uniform(0.8, 1.5), 3), "radius": PERSON_RADIUS,
                           "trigger": [mid[0], mid[1], round(rng.uniform(3.0, 9.0), 2)]})
    return {"loop": loop, "robot": {"x": first[0], "y": first[1], "yaw": yaw}, "people": people}


def make_order(order_id, layout, loop):
    nodes, edges = [], []
    for k, name in enumerate(loop):
        x, y = layout["nodes"][name]
        nodes.append({"nodeId": name, "sequenceId": 2 * k, "released": True, "actions": [],
                      "nodePosition": {"x": x, "y": y, "mapId": "warehouse"}})
        if k:
            edges.append({"edgeId": f"{loop[k - 1]}-{name}", "sequenceId": 2 * k - 1, "released": True,
                          "startNodeId": loop[k - 1], "endNodeId": name, "actions": []})
    return {"headerId": 1, "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime()), "version": "2.1.0",
            "manufacturer": "PortfolioSim", "serialNumber": "tugger-01", "orderId": order_id, "orderUpdateId": 0,
            "nodes": nodes, "edges": edges}


def rect_distance(px, py, r):
    x0, y0, x1, y1 = r
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def proc_cpu():
    """CPU seconds (user + system) per process name, for every process visible in this container."""
    hz = os.sysconf("SC_CLK_TCK")
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/stat") as f:
                stat = f.read()
            with open(f"/proc/{pid}/cmdline") as f:
                cmd = f.read().split("\0")
        except OSError:
            continue
        fields = stat.rsplit(")", 1)[1].split()
        secs = (int(fields[11]) + int(fields[12])) / hz
        name = os.path.basename(cmd[0]) if cmd and cmd[0] else stat.split("(")[1].split(")")[0]
        if name.startswith("python"):
            name = "python:" + (os.path.basename(cmd[1]) if len(cmd) > 1 else "")
        out[name] = out.get(name, 0.0) + secs
    return out


class Runner:
    def __init__(self, layout):
        self.layout = layout
        self.node = rclpy.create_node("mission_runner")
        self.state = None
        self.status = None
        self.metrics = None
        self.gt = None
        self.sim_t = 0.0
        self.status_log = []
        self.gt_log = []
        n = self.node
        n.create_subscription(String, "vda5050/state", self._on_state, 50)
        n.create_subscription(FieldStatus, "field_monitor/status", self._on_status, 50)
        n.create_subscription(String, "sim/metrics", lambda m: setattr(self, "metrics", json.loads(m.data)), 10)
        n.create_subscription(String, "sim/ground_truth", self._on_gt, 50)
        n.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.order_pub = n.create_publisher(String, "vda5050/order", 10)
        self.instant_pub = n.create_publisher(String, "vda5050/instantActions", 10)
        self.scenario_pub = n.create_publisher(String, "sim/scenario", 10)
        srv = ClearEntireCostmap if ClearEntireCostmap is not None else Empty
        self.clear = [n.create_client(srv, s) for s in ("local_costmap/clear_entirely_local_costmap",
                                                         "global_costmap/clear_entirely_global_costmap")]

    def _on_state(self, m):
        self.state = json.loads(m.data)

    def _on_status(self, m):
        self.status = m
        self.status_log.append(m)

    def _on_gt(self, m):
        self.gt = json.loads(m.data)
        self.gt_log.append(self.gt)

    def _on_clock(self, m):
        self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9

    def spin_until(self, cond, wall_timeout):
        end = time.time() + wall_timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if cond():
                return True
        return False

    def wait_ready(self):
        client = ActionClient(self.node, NavigateThroughPoses, "navigate_through_poses")
        if not client.wait_for_server(timeout_sec=120.0):
            raise RuntimeError("Nav2 not available")
        assert self.spin_until(lambda: self.status is not None and self.state is not None and self.metrics, 60.0)
        self.spin_until(lambda: False, 5.0)  # costmaps receive the map

    def clear_costmaps(self):
        for c in self.clear:
            if c.wait_for_service(timeout_sec=5.0):
                fut = c.call_async(c.srv_type.Request())
                self.spin_until(fut.done, 5.0)

    def run(self, cond, seed, kind, keep_trace=False):
        sc = make_scenario(self.layout, seed, kind)
        mid = f"{cond}-{kind}-{seed}"
        self.scenario_pub.publish(String(data=json.dumps({"id": mid, "robot": sc["robot"], "people": sc["people"]})))
        self.spin_until(lambda: self.metrics is not None and self.metrics["id"] == mid, 10.0)
        self.spin_until(lambda: False, 1.0)
        self.clear_costmaps()
        self.spin_until(lambda: False, 1.0)
        base = self.status
        self.status_log.clear()
        self.gt_log.clear()
        order = make_order(mid, self.layout, sc["loop"])
        final_seq = 2 * (len(sc["loop"]) - 1)
        t0, w0 = self.sim_t, time.time()
        self.order_pub.publish(String(data=json.dumps(order)))

        def done():
            s = self.state
            if s is None or s["orderId"] != mid:
                return False
            if any(e["errorType"] in ("navigationError", "orderError", "noRouteError", "validationError") for e in s["errors"]):
                return True
            return s["lastNodeSequenceId"] == final_seq and not s["nodeStates"] and not s["driving"]

        while not done() and self.sim_t - t0 < TIMEOUT_SIM_S:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        t1, w1 = self.sim_t, time.time()
        s = self.state
        errors = [e["errorType"] for e in s["errors"]] if s and s["orderId"] == mid else ["orderNotAccepted"]
        success = bool(s and s["orderId"] == mid and s["lastNodeSequenceId"] == final_seq and not s["nodeStates"]
                       and not errors)
        if not success:
            cancel = {"headerId": 2, "timestamp": order["timestamp"], "version": "2.1.0", "manufacturer": "PortfolioSim",
                      "serialNumber": "tugger-01",
                      "actions": [{"actionId": f"cancel-{mid}", "actionType": "cancelOrder", "blockingType": "HARD"}]}
            self.instant_pub.publish(String(data=json.dumps(cancel)))
            self.spin_until(lambda: False, 2.0)
        self.spin_until(lambda: self.metrics is not None and self.metrics["id"] == mid and self.metrics["t"] >= t1, 5.0)
        m = self.metrics
        last = self.status_log[-1] if self.status_log else base
        # Protective stops with their trigger point, classified with the ground truth at that moment.
        stops = []
        prev = base.protective_stops
        time_in_stop = 0.0
        prev_t = None
        gt_i = 0
        for st in self.status_log:
            t = st.header.stamp.sec + st.header.stamp.nanosec * 1e-9
            if prev_t is not None and st.state == FieldStatus.PROTECTIVE_STOP:
                time_in_stop += t - prev_t
            prev_t = t
            if st.protective_stops > prev:
                prev = st.protective_stops
                while gt_i + 1 < len(self.gt_log) and self.gt_log[gt_i + 1]["t"] <= t:
                    gt_i += 1
                g = self.gt_log[gt_i] if self.gt_log else {"people": []}
                px, py = st.nearest_violation.x, st.nearest_violation.y
                d_person = min([math.hypot(px - q[0], py - q[1]) - PERSON_RADIUS for q in g["people"]], default=1e9)
                d_static = min(rect_distance(px, py, r) for r in self.layout["blocks"])
                cause = "person" if d_person <= 0.3 else ("static" if d_static <= 0.15 else "other")
                stops.append({"t": round(t - t0, 2), "x": round(px, 2), "y": round(py, 2), "cause": cause,
                              "speed": round(st.selection_speed, 3)})
        rec = {
            "id": mid, "condition": cond, "seed": seed, "kind": kind, "direction": "ccw" if sc["loop"][1] == "B" else "cw",
            "people": len(sc["people"]), "success": success, "errors": errors,
            "mission_time_s": round(t1 - t0, 2), "wall_time_s": round(w1 - w0, 2),
            "rtf": round((t1 - t0) / (w1 - w0), 4) if w1 > w0 else None,
            "distance_m": round(m["distance"], 2),
            "min_clearance_people_m": m["min_clearance_people"], "min_clearance_static_m": m["min_clearance_static"],
            "people_contacts": m["people_contacts"], "static_contacts": m["static_contacts"],
            "contacts": [{"t": round(c["t"] - t0, 2), "robot_speed": round(c["v"], 3), "front": c["front"],
                          "lx": round(c["lx"], 3), "ly": round(c["ly"], 3)} for c in m["contacts"]],
            "protective_stops": last.protective_stops - base.protective_stops,
            "warning_entries": last.warning_entries - base.warning_entries,
            "protective_intrusions": last.protective_intrusions - base.protective_intrusions,
            "time_in_protective_stop_s": round(time_in_stop, 2),
            "stops": stops,
            "eval_time_us": [round(st.eval_time_us, 1) for st in self.status_log],
        }
        if keep_trace:
            rec["trace"] = self.trace(t0, sc)
        return rec

    def trace(self, t0, sc):
        """Ground truth at 10 Hz with the field monitor status closest in time (for the web replay)."""
        out = []
        j = 0
        st_log = self.status_log
        for g in self.gt_log:
            if g["t"] < t0:
                continue
            while j + 1 < len(st_log) and (st_log[j + 1].header.stamp.sec + st_log[j + 1].header.stamp.nanosec * 1e-9) <= g["t"]:
                j += 1
            st = st_log[j] if st_log else None
            out.append([round(g["t"] - t0, 2), round(g["x"], 3), round(g["y"], 3), round(g["yaw"], 3), round(g["v"], 3),
                        [[round(p[0], 2), round(p[1], 2)] for p in g["people"]],
                        st.state if st else 0,
                        round(st.protective_length, 3) if st else 0, round(st.warning_length, 3) if st else 0])
        return {"columns": ["t", "x", "y", "yaw", "v", "people", "state", "prot_len", "warn_len"],
                "samples": out, "people_plan": sc["people"], "loop": sc["loop"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--condition", required=True)
    ap.add_argument("--missions", required=True, help="comma separated seed:kind")
    ap.add_argument("--trace", default="", help="comma separated seed:kind to record trajectories for")
    ap.add_argument("--layout", default="/ws/ros2_ws/install/tugger_nav/share/tugger_nav/maps/layout.json")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    layout = json.load(open(a.layout))
    rclpy.init()
    r = Runner(layout)
    r.wait_ready()
    trace = set(a.trace.split(",")) if a.trace else set()
    cpu0, wall0, sim0 = proc_cpu(), time.time(), r.sim_t
    with open(a.out, "w") as f:
        for item in a.missions.split(","):
            seed, kind = item.split(":")
            rec = r.run(a.condition, int(seed), kind, keep_trace=item in trace)
            f.write(json.dumps(rec) + "\n")
            f.flush()
            print(f"{rec['id']}: success={rec['success']} t={rec['mission_time_s']}s stops={rec['protective_stops']} "
                  f"warn={rec['warning_entries']} minP={rec['min_clearance_people_m']} contacts={rec['people_contacts']} "
                  f"err={rec['errors']}", flush=True)
        cpu1, wall1, sim1 = proc_cpu(), time.time(), r.sim_t
        cpu = {k: round((cpu1.get(k, 0.0) - cpu0.get(k, 0.0)) / (wall1 - wall0), 4) for k in cpu1}
        f.write(json.dumps({"summary": True, "condition": a.condition, "wall_s": round(wall1 - wall0, 1),
                            "sim_s": round(sim1 - sim0, 1), "cpu_cores_by_process": cpu}) + "\n")
    r.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
