#include <gtest/gtest.h>

#include <string>

#include "tugger_nav/vda5050_core.hpp"

using namespace tugger_nav::vda;

#ifndef SCHEMA_DIR
#error SCHEMA_DIR must be defined
#endif

namespace {
json header() {
  return {{"headerId", 1}, {"timestamp", "2026-10-05T10:00:00.000Z"}, {"version", "2.1.0"},
          {"manufacturer", "PortfolioSim"}, {"serialNumber", "tugger-01"}};
}
json node(const std::string & id, int seq, double x, double y, bool released = true) {
  return {{"nodeId", id}, {"sequenceId", seq}, {"released", released}, {"actions", json::array()},
          {"nodePosition", {{"x", x}, {"y", y}, {"mapId", "warehouse"}}}};
}
json edge(const std::string & a, const std::string & b, int seq, bool released = true) {
  return {{"edgeId", a + "-" + b}, {"sequenceId", seq}, {"released", released}, {"startNodeId", a},
          {"endNodeId", b}, {"actions", json::array()}};
}
json order(const std::string & id, int update, json nodes, json edges) {
  json o = header();
  o["orderId"] = id;
  o["orderUpdateId"] = update;
  o["nodes"] = nodes;
  o["edges"] = edges;
  return o;
}
json abc() {
  return order("o1", 0, {node("A", 0, 0, 0), node("B", 2, 5, 0), node("C", 4, 10, 0)},
               {edge("A", "B", 1), edge("B", "C", 3)});
}
const std::string kDir = SCHEMA_DIR;
}  // namespace

TEST(Schema, ValidOrderPasses) {
  SchemaValidator v(kDir + "/order.schema");
  EXPECT_EQ(v.validate(abc()), "");
}

TEST(Schema, MissingFieldAndWrongTypeFail) {
  SchemaValidator v(kDir + "/order.schema");
  json o = abc();
  o.erase("orderId");
  EXPECT_NE(v.validate(o), "");
  o = abc();
  o["nodes"][0]["sequenceId"] = "zero";
  EXPECT_NE(v.validate(o), "");
  o = abc();
  o["timestamp"] = "yesterday";  // format: date-time
  EXPECT_NE(v.validate(o), "");
  o = abc();
  o["nodes"][1]["actions"] = json::array({{{"actionId", "a1"}, {"actionType", "pick"}}});  // blockingType missing
  EXPECT_NE(v.validate(o), "");
}

TEST(Semantics, AcceptsWellFormedOrder) {
  Order o;
  EXPECT_TRUE(parseOrder(abc(), "warehouse", o).empty());
  EXPECT_EQ(o.nodes.size(), 3u);
  EXPECT_EQ(o.edges[1].end_node, "C");
}

TEST(Semantics, RejectsBadSequencesEdgesAndMaps) {
  Order o;
  json j = abc();
  j["nodes"][1]["sequenceId"] = 3;
  EXPECT_FALSE(parseOrder(j, "warehouse", o).empty());
  j = abc();
  j["edges"][0]["endNodeId"] = "C";
  EXPECT_FALSE(parseOrder(j, "warehouse", o).empty());
  j = abc();
  j["edges"].erase(1);
  EXPECT_FALSE(parseOrder(j, "warehouse", o).empty());
  EXPECT_FALSE(parseOrder(abc(), "other_map", o).empty());
  j = abc();
  j["nodes"][1].erase("nodePosition");
  EXPECT_FALSE(parseOrder(j, "warehouse", o).empty());
  // Horizon node followed by a base node.
  j = order("o1", 0, {node("A", 0, 0, 0), node("B", 2, 5, 0, false), node("C", 4, 10, 0)},
            {edge("A", "B", 1, false), edge("B", "C", 3)});
  EXPECT_FALSE(parseOrder(j, "warehouse", o).empty());
}

TEST(Tracker, ProgressThroughNodesByDistance) {
  Order o;
  ASSERT_TRUE(parseOrder(abc(), "warehouse", o).empty());
  OrderTracker t;
  std::vector<Error> errs;
  ASSERT_EQ(t.submit(o, 0.1, 0.0, 1.0, errs), OrderTracker::Decision::Accepted);
  EXPECT_EQ(t.lastNodeId(), "A");
  EXPECT_EQ(t.nodes().size(), 2u);
  EXPECT_EQ(t.edges().size(), 2u);
  EXPECT_FALSE(t.updatePosition(3.0, 0.0, 0.5));
  EXPECT_TRUE(t.updatePosition(4.7, 0.1, 0.5));
  EXPECT_EQ(t.lastNodeId(), "B");
  EXPECT_EQ(t.lastNodeSeq(), 2);
  EXPECT_EQ(t.edges().size(), 1u);
  EXPECT_TRUE(t.updatePosition(10.0, 0.0, 0.5));
  EXPECT_FALSE(t.active());
  EXPECT_TRUE(t.nodes().empty());
  EXPECT_TRUE(t.edges().empty());
}

TEST(Tracker, RejectsFarStartAndBusyVehicle) {
  Order o;
  ASSERT_TRUE(parseOrder(abc(), "warehouse", o).empty());
  OrderTracker t;
  std::vector<Error> errs;
  EXPECT_EQ(t.submit(o, 5.0, 5.0, 1.0, errs), OrderTracker::Decision::Rejected);
  EXPECT_EQ(errs.at(0).type, "noRouteError");
  errs.clear();
  ASSERT_EQ(t.submit(o, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Accepted);
  Order o2 = o;
  o2.order_id = "o2";
  EXPECT_EQ(t.submit(o2, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Rejected);
  EXPECT_EQ(errs.at(0).type, "orderError");
}

TEST(Tracker, OrderUpdatesStitchAtBaseEnd) {
  // Base A-B, horizon C.
  json j = order("o1", 0, {node("A", 0, 0, 0), node("B", 2, 5, 0), node("C", 4, 10, 0, false)},
                 {edge("A", "B", 1), edge("B", "C", 3, false)});
  Order o;
  ASSERT_TRUE(parseOrder(j, "warehouse", o).empty());
  OrderTracker t;
  std::vector<Error> errs;
  ASSERT_EQ(t.submit(o, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Accepted);
  EXPECT_EQ(t.remainingBase().size(), 1u);
  // Same update id again: ignored.
  EXPECT_EQ(t.submit(o, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Ignored);
  // Update 1 releases B-C-D, starting at the base end B.
  json u = order("o1", 1, {node("B", 2, 5, 0), node("C", 4, 10, 0), node("D", 6, 15, 0)},
                 {edge("B", "C", 3), edge("C", "D", 5)});
  Order ou;
  ASSERT_TRUE(parseOrder(u, "warehouse", ou).empty());
  ASSERT_EQ(t.submit(ou, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Updated);
  EXPECT_EQ(t.remainingBase().size(), 3u);  // B, C, D
  EXPECT_EQ(t.updateId(), 1);
  // Lower update id: rejected.
  Order old = o;
  old.update_id = 0;
  EXPECT_EQ(t.submit(old, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Rejected);
  EXPECT_EQ(errs.back().type, "orderUpdateError");
  // Update that does not start at the base end: rejected.
  json bad = order("o1", 2, {node("C", 4, 10, 0), node("D", 6, 15, 0)}, {edge("C", "D", 5)});
  Order ob;
  ASSERT_TRUE(parseOrder(bad, "warehouse", ob).empty());
  EXPECT_EQ(t.submit(ob, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Rejected);
}

TEST(Tracker, FinishIfAtEndAndCancel) {
  Order o;
  ASSERT_TRUE(parseOrder(abc(), "warehouse", o).empty());
  OrderTracker t;
  std::vector<Error> errs;
  ASSERT_EQ(t.submit(o, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Accepted);
  EXPECT_FALSE(t.finishIfAtEnd(5.0, 0.0, 0.5));  // not at the last node
  EXPECT_TRUE(t.finishIfAtEnd(10.2, 0.0, 0.5));  // B was skipped, goal reached at C
  EXPECT_EQ(t.lastNodeId(), "C");
  EXPECT_FALSE(t.active());
  OrderTracker t2;
  ASSERT_EQ(t2.submit(o, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Accepted);
  t2.cancel();
  EXPECT_FALSE(t2.active());
  EXPECT_TRUE(t2.nodes().empty());
}

TEST(Segments, ClosedLoopIsSplitBeforeReturningToTheStart) {
  auto mk = [](const std::string & id, double x, double y) {
    Node n;
    n.id = id;
    n.x = x;
    n.y = y;
    return n;
  };
  // Loop A(0,0) -> B(10,0) -> C(10,10) -> D(0,10) -> A(0,0); robot at A, base = B C D A.
  std::vector<Node> base = {mk("B", 10, 0), mk("C", 10, 10), mk("D", 0, 10), mk("A", 0, 0)};
  EXPECT_EQ(segmentLength(base, 0.0, 0.0, 1.5), 3u);  // B C D, then A in a second goal
  EXPECT_EQ(segmentLength({mk("A", 0, 0)}, 0.0, 0.0, 1.5), 1u);  // last node alone, even if close
  EXPECT_EQ(segmentLength(base, 50.0, 50.0, 1.5), 4u);  // open route: one goal
  std::vector<Node> near = {mk("B", 1, 0), mk("C", 10, 0)};
  EXPECT_EQ(segmentLength(near, 0.0, 0.0, 1.5), 2u);  // the first node is always included
}

TEST(Tracker, TraverseThroughSkipsPassedNodes) {
  Order o;
  ASSERT_TRUE(parseOrder(abc(), "warehouse", o).empty());
  OrderTracker t;
  std::vector<Error> errs;
  ASSERT_EQ(t.submit(o, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Accepted);
  EXPECT_FALSE(t.traverseThrough(2, 3.0, 0.0, 0.5));  // not at B
  EXPECT_TRUE(t.traverseThrough(2, 5.3, 0.0, 0.5));
  EXPECT_EQ(t.lastNodeId(), "B");
  EXPECT_TRUE(t.active());
}

TEST(State, BuiltStateMatchesOfficialSchema) {
  SchemaValidator v(kDir + "/state.schema");
  OrderTracker idle;
  StateInputs in;
  in.timestamp = isoTimestampNow();
  in.manufacturer = "PortfolioSim";
  in.serial_number = "tugger-01";
  in.map_id = "warehouse";
  EXPECT_EQ(v.validate(buildState(idle, in)), "");
  // With an order, an action and an error.
  json j = abc();
  j["nodes"][1]["actions"] = json::array({{{"actionId", "a1"}, {"actionType", "pick"}, {"blockingType", "HARD"}}});
  Order o;
  ASSERT_TRUE(parseOrder(j, "warehouse", o).empty());
  OrderTracker t;
  std::vector<Error> errs;
  ASSERT_EQ(t.submit(o, 0.0, 0.0, 1.0, errs), OrderTracker::Decision::Accepted);
  Error e;
  e.type = "navigationError";
  e.description = "test";
  e.refs = {{"nodeId", "B"}};
  in.errors = {e};
  in.protective_stop = true;
  in.field_violation = true;
  const json s = buildState(t, in);
  EXPECT_EQ(v.validate(s), "");
  EXPECT_EQ(s["safetyState"]["eStop"], "AUTOACK");
  EXPECT_EQ(s["actionStates"][0]["actionStatus"], "FAILED");
  EXPECT_EQ(s["nodeStates"].size(), 2u);
  EXPECT_EQ(s["lastNodeId"], "A");
}

TEST(State, TimestampIsIso8601Utc) {
  const std::string ts = isoTimestampNow();
  ASSERT_EQ(ts.size(), 24u);
  EXPECT_EQ(ts[10], 'T');
  EXPECT_EQ(ts.back(), 'Z');
}
