"""Integration test: full stack (simulator, Nav2, field monitor, VDA 5050 bridge) in one launch.

An order A -> B -> C goes in on /vda5050/order; the robot drives 13.5 m and the VDA 5050 state must report
lastNodeId C, no nodeStates / edgeStates left, not driving and no errors. Invalid orders must be rejected.
"""
import json
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from std_msgs.msg import String
from tugger_msgs.msg import FieldStatus


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("tugger_nav")
    stack = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "sim_stack.launch.py")),
        launch_arguments={"field_monitor": "true", "log_level": "error"}.items(),
    )
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


def header(i):
    return {"headerId": i, "timestamp": "2026-10-05T10:00:00.000Z", "version": "2.1.0",
            "manufacturer": "PortfolioSim", "serialNumber": "tugger-01"}


def make_order(order_id, pts):
    nodes, edges = [], []
    for k, (name, x, y) in enumerate(pts):
        nodes.append({"nodeId": name, "sequenceId": 2 * k, "released": True, "actions": [],
                      "nodePosition": {"x": x, "y": y, "mapId": "warehouse"}})
        if k:
            prev = pts[k - 1][0]
            edges.append({"edgeId": f"{prev}-{name}", "sequenceId": 2 * k - 1, "released": True,
                          "startNodeId": prev, "endNodeId": name, "actions": []})
    return {**header(1), "orderId": order_id, "orderUpdateId": 0, "nodes": nodes, "edges": edges}


class TestOrderToFinished(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("order_test")
        cls.states = []
        cls.status = []
        cls.node.create_subscription(String, "vda5050/state", lambda m: cls.states.append(json.loads(m.data)), 10)
        cls.node.create_subscription(FieldStatus, "field_monitor/status", lambda m: cls.status.append(m), 10)
        cls.pub = cls.node.create_publisher(String, "vda5050/order", 10)
        client = ActionClient(cls.node, NavigateThroughPoses, "navigate_through_poses")
        assert client.wait_for_server(timeout_sec=90.0), "Nav2 did not come up"
        # bt_navigator is active once its action server is up; give the costmaps a moment to receive the map.
        cls.spin_for(5.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def spin_for(cls, seconds, until=None):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if until is not None and until():
                return True
        return False

    def send(self, doc):
        self.pub.publish(String(data=json.dumps(doc)))

    def test_1_field_monitor_runs_and_is_clear_at_standstill(self):
        self.assertTrue(self.spin_for(10.0, lambda: len(self.status) > 10))
        last = self.status[-1]
        self.assertTrue(last.enabled)
        self.assertEqual(last.state, FieldStatus.CLEAR)
        self.assertEqual(last.field_set, 0)

    def test_2_invalid_order_is_rejected_by_schema(self):
        bad = make_order("bad-1", [("A", 6.0, 6.0), ("B", 12.5, 6.0)])
        del bad["nodes"][1]["sequenceId"]
        self.states.clear()
        self.send(bad)
        ok = self.spin_for(10.0, lambda: any(e["errorType"] == "validationError" for s in self.states for e in s["errors"]))
        self.assertTrue(ok, "no validationError in the state")
        self.assertNotEqual(self.states[-1]["orderId"], "bad-1")

    def test_3_order_runs_to_the_final_node(self):
        order = make_order("test-1", [("A", 6.0, 6.0), ("B", 12.5, 6.0), ("C", 19.5, 6.0)])
        self.states.clear()
        self.send(order)

        def finished():
            s = self.states[-1] if self.states else None
            return (s is not None and s["orderId"] == "test-1" and s["lastNodeId"] == "C" and not s["nodeStates"]
                    and not s["edgeStates"] and not s["driving"])

        self.assertTrue(self.spin_for(120.0, finished), f"not finished, last state: {self.states[-1:]}")
        s = self.states[-1]
        self.assertEqual(s["errors"], [])
        self.assertEqual(s["lastNodeSequenceId"], 4)
        self.assertLess(abs(s["agvPosition"]["x"] - 19.5), 0.6)
        seen = [st["lastNodeId"] for st in self.states if st["orderId"] == "test-1"]
        self.assertIn("B", seen)  # B was reported as traversed on the way

    def test_4_new_order_must_start_at_the_vehicle(self):
        far = make_order("far-1", [("H", 12.5, 16.0), ("I", 6.0, 16.0)])
        self.states.clear()
        self.send(far)
        ok = self.spin_for(10.0, lambda: any(e["errorType"] == "noRouteError" for s in self.states for e in s["errors"]))
        self.assertTrue(ok)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_own_nodes_exit_cleanly(self, proc_info):
        for name in ("warehouse_sim", "protective_field_monitor", "vda5050_bridge"):
            matches = [p for p in proc_info.process_names() if name in p]
            for p in matches:
                self.assertIn(proc_info[p].returncode, (0, -2, -15), p)
