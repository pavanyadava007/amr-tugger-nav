"""Run inside the image after `colcon test`: summarise the JUnit XML files into results/tests.json."""
import glob
import json
import sys
import xml.etree.ElementTree as ET

out = {"suites": [], "total": 0, "failures": 0, "skipped": 0}
for f in sorted(glob.glob("/ws/ros2_ws/build/tugger_nav/test_results/tugger_nav/*.xml")):
    root = ET.parse(f).getroot()
    suites = [root] if root.tag == "testsuite" else root.findall("testsuite")
    n = sum(int(s.get("tests", 0)) for s in suites)
    bad = sum(int(s.get("failures", 0)) + int(s.get("errors", 0)) for s in suites)
    skip = sum(int(s.get("skipped", 0)) for s in suites)
    kind = "launch_testing" if f.endswith(".py.xunit.xml") else "gtest"
    out["suites"].append({"file": f.rsplit("/", 1)[1], "kind": kind, "tests": n, "failures": bad, "skipped": skip})
    out["total"] += n
    out["failures"] += bad
    out["skipped"] += skip
with open(sys.argv[1], "w") as fh:
    json.dump(out, fh, indent=2)
print(f"{out['total']} test cases, {out['failures']} failures")
sys.exit(1 if out["failures"] or not out["total"] else 0)
