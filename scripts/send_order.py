"""Send one VDA 5050 order through the given nodes of the layout and print the state until it is finished.

Usage (inside the container, stack running): python3 /ws/scripts/send_order.py A B C D
"""
import json
import sys
import time

import rclpy
from run_missions import make_order
from std_msgs.msg import String


def main():
    names = sys.argv[1:] or ["A", "B", "C"]
    layout = json.load(open("/ws/ros2_ws/install/tugger_nav/share/tugger_nav/maps/layout.json"))
    rclpy.init()
    node = rclpy.create_node("send_order")
    last = {}
    node.create_subscription(String, "vda5050/state", lambda m: last.update(json.loads(m.data)), 10)
    pub = node.create_publisher(String, "vda5050/order", 10)
    time.sleep(1.0)
    order_id = f"manual-{int(time.time())}"
    pub.publish(String(data=json.dumps(make_order(order_id, layout, names))))
    printed = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if last.get("orderId") != order_id:
            continue
        view = (last["lastNodeId"], len(last["nodeStates"]), last["driving"], [e["errorType"] for e in last["errors"]],
                last["safetyState"]["fieldViolation"])
        if view != printed:
            printed = view
            print(f"lastNodeId={view[0]} nodeStates={view[1]} driving={view[2]} errors={view[3]} fieldViolation={view[4]}")
        if view[3] or (view[1] == 0 and not view[2]):
            break
    rclpy.shutdown()


if __name__ == "__main__":
    main()
