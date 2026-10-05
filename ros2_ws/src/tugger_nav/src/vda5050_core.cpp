#include "tugger_nav/vda5050_core.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <nlohmann/json-schema.hpp>
#include <stdexcept>

namespace tugger_nav::vda {

json Error::toJson() const {
  json j = {{"errorType", type}, {"errorLevel", level}, {"errorDescription", description}};
  json r = json::array();
  for (const auto & [k, v] : refs) {
    r.push_back({{"referenceKey", k}, {"referenceValue", v}});
  }
  j["errorReferences"] = r;
  return j;
}

SchemaValidator::SchemaValidator(const std::string & schema_path) {
  std::ifstream f(schema_path);
  if (!f) {
    throw std::runtime_error("cannot open schema " + schema_path);
  }
  json schema = json::parse(f);
  v_ = std::make_unique<nlohmann::json_schema::json_validator>(
    nullptr, nlohmann::json_schema::default_string_format_check);
  v_->set_root_schema(schema);
}

SchemaValidator::~SchemaValidator() = default;

std::string SchemaValidator::validate(const json & doc) const {
  try {
    v_->validate(doc);
  } catch (const std::exception & e) {
    return e.what();
  }
  return "";
}

namespace {
Error orderError(const std::string & what, const std::string & key = "", const std::string & value = "") {
  Error e;
  e.type = "orderError";
  e.description = what;
  if (!key.empty()) {
    e.refs.emplace_back(key, value);
  }
  return e;
}
}  // namespace

std::vector<Error> parseOrder(const json & doc, const std::string & map_id, Order & out) {
  std::vector<Error> errs;
  Order o;
  o.order_id = doc.at("orderId").get<std::string>();
  o.update_id = doc.at("orderUpdateId").get<int64_t>();
  for (const auto & jn : doc.at("nodes")) {
    Node n;
    n.id = jn.at("nodeId").get<std::string>();
    n.seq = jn.at("sequenceId").get<int64_t>();
    n.released = jn.at("released").get<bool>();
    n.actions = jn.at("actions");
    if (jn.contains("nodePosition")) {
      const auto & p = jn["nodePosition"];
      n.has_position = true;
      n.x = p.at("x").get<double>();
      n.y = p.at("y").get<double>();
      n.map_id = p.at("mapId").get<std::string>();
      if (p.contains("theta")) {
        n.theta = p["theta"].get<double>();
      }
      n.allowed_deviation_xy = p.value("allowedDeviationXY", 0.0);
    }
    o.nodes.push_back(n);
  }
  for (const auto & je : doc.at("edges")) {
    Edge e;
    e.id = je.at("edgeId").get<std::string>();
    e.seq = je.at("sequenceId").get<int64_t>();
    e.released = je.at("released").get<bool>();
    e.start_node = je.at("startNodeId").get<std::string>();
    e.end_node = je.at("endNodeId").get<std::string>();
    e.actions = je.at("actions");
    if (je.contains("maxSpeed")) {
      e.max_speed = je["maxSpeed"].get<double>();
    }
    o.edges.push_back(e);
  }
  if (o.nodes.empty()) {
    errs.push_back(orderError("order has no nodes", "orderId", o.order_id));
    return errs;
  }
  if (o.edges.size() + 1 != o.nodes.size()) {
    errs.push_back(orderError("an order with n nodes needs n-1 edges", "orderId", o.order_id));
    return errs;
  }
  if (!o.nodes.front().released) {
    errs.push_back(orderError("first node must be released", "nodeId", o.nodes.front().id));
  }
  // Sequence ids: node, edge, node, ... strictly +1, nodes even.
  for (size_t i = 0; i < o.nodes.size(); ++i) {
    const int64_t expect = o.nodes.front().seq + 2 * static_cast<int64_t>(i);
    if (o.nodes[i].seq != expect || o.nodes[i].seq % 2 != 0) {
      errs.push_back(orderError("node sequenceIds must be even and increase by 2", "nodeId", o.nodes[i].id));
    }
    if (i + 1 < o.nodes.size()) {
      const Edge & e = o.edges[i];
      if (e.seq != expect + 1) {
        errs.push_back(orderError("edge sequenceId must lie between its nodes", "edgeId", e.id));
      }
      if (e.start_node != o.nodes[i].id || e.end_node != o.nodes[i + 1].id) {
        errs.push_back(orderError("edge start/end does not match the neighbouring nodes", "edgeId", e.id));
      }
      if (e.released && !o.nodes[i + 1].released) {
        errs.push_back(orderError("a released edge must end in a released node", "edgeId", e.id));
      }
    }
    if (i > 0 && o.nodes[i].released && !o.nodes[i - 1].released) {
      errs.push_back(orderError("base nodes must come before horizon nodes", "nodeId", o.nodes[i].id));
    }
    if (!o.nodes[i].has_position) {
      errs.push_back(orderError("this vehicle needs nodePosition on every node", "nodeId", o.nodes[i].id));
    } else if (o.nodes[i].map_id != map_id) {
      errs.push_back(orderError("node is on an unknown map '" + o.nodes[i].map_id + "'", "nodeId", o.nodes[i].id));
    }
  }
  if (errs.empty()) {
    out = o;
  }
  return errs;
}

void OrderTracker::collectActions(const Order & o) {
  for (const auto & n : o.nodes) {
    for (const auto & a : n.actions) {
      action_states_.push_back({{"actionId", a.at("actionId")}, {"actionType", a.at("actionType")},
                                {"actionStatus", "FAILED"},
                                {"resultDescription", "actions are not executed by this simulated vehicle"}});
    }
  }
  for (const auto & e : o.edges) {
    for (const auto & a : e.actions) {
      action_states_.push_back({{"actionId", a.at("actionId")}, {"actionType", a.at("actionType")},
                                {"actionStatus", "FAILED"},
                                {"resultDescription", "actions are not executed by this simulated vehicle"}});
    }
  }
}

OrderTracker::Decision OrderTracker::submit(const Order & o, double robot_x, double robot_y,
                                            double start_tolerance, std::vector<Error> & errs) {
  if (hasOrder() && o.order_id == order_id_) {
    if (o.update_id < update_id_) {
      Error e;
      e.type = "orderUpdateError";
      e.description = "orderUpdateId is lower than the current one";
      e.refs = {{"orderId", o.order_id}, {"orderUpdateId", std::to_string(o.update_id)}};
      errs.push_back(e);
      return Decision::Rejected;
    }
    if (o.update_id == update_id_) {
      return Decision::Ignored;  // duplicate (e.g. resent after a connection loss)
    }
    // The update must continue from the end of the current base.
    const Node * base_end = nullptr;
    for (const auto & n : nodes_) {
      if (n.released) {
        base_end = &n;
      }
    }
    const bool at_last = base_end == nullptr;  // whole base already traversed: stitch at the last node
    const std::string stitch_id = at_last ? last_node_id_ : base_end->id;
    const int64_t stitch_seq = at_last ? last_node_seq_ : base_end->seq;
    if (o.nodes.front().id != stitch_id || o.nodes.front().seq != stitch_seq) {
      Error e;
      e.type = "orderUpdateError";
      e.description = "update does not start at the last base node";
      e.refs = {{"orderId", o.order_id}, {"nodeId", o.nodes.front().id}};
      errs.push_back(e);
      return Decision::Rejected;
    }
    std::vector<Node> keep_nodes;
    for (const auto & n : nodes_) {
      if (n.seq < stitch_seq) {
        keep_nodes.push_back(n);
      }
    }
    std::vector<Edge> keep_edges;
    for (const auto & e : edges_) {
      if (e.seq < stitch_seq) {
        keep_edges.push_back(e);
      }
    }
    for (const auto & n : o.nodes) {
      if (!(at_last && n.seq == stitch_seq)) {  // an already traversed stitch node is not repeated
        keep_nodes.push_back(n);
      }
    }
    keep_edges.insert(keep_edges.end(), o.edges.begin(), o.edges.end());
    nodes_ = keep_nodes;
    edges_ = keep_edges;
    update_id_ = o.update_id;
    collectActions(o);
    return Decision::Updated;
  }
  if (active()) {
    errs.push_back(orderError("vehicle is still executing order '" + order_id_ + "'", "orderId", o.order_id));
    return Decision::Rejected;
  }
  const Node & first = o.nodes.front();
  const double d = std::hypot(first.x - robot_x, first.y - robot_y);
  if (d > tolerance(first, start_tolerance)) {
    Error e;
    e.type = "noRouteError";
    e.description = "first node is not at the vehicle position";
    e.refs = {{"nodeId", first.id}};
    errs.push_back(e);
    return Decision::Rejected;
  }
  order_id_ = o.order_id;
  update_id_ = o.update_id;
  nodes_ = o.nodes;
  edges_ = o.edges;
  action_states_ = json::array();
  collectActions(o);
  traverse(0);  // the vehicle is at the first node
  return Decision::Accepted;
}

void OrderTracker::traverse(size_t node_index) {
  const Node n = nodes_[node_index];
  last_node_id_ = n.id;
  last_node_seq_ = n.seq;
  std::vector<Node> rest_nodes;
  for (const auto & m : nodes_) {
    if (m.seq > n.seq) {
      rest_nodes.push_back(m);
    }
  }
  std::vector<Edge> rest_edges;
  for (const auto & e : edges_) {
    if (e.seq > n.seq) {
      rest_edges.push_back(e);
    }
  }
  nodes_ = rest_nodes;
  edges_ = rest_edges;
}

bool OrderTracker::active() const {
  for (const auto & n : nodes_) {
    if (n.released) {
      return true;
    }
  }
  return false;
}

std::vector<Node> OrderTracker::remainingBase() const {
  std::vector<Node> out;
  for (const auto & n : nodes_) {
    if (n.released) {
      out.push_back(n);
    }
  }
  return out;
}

bool OrderTracker::updatePosition(double x, double y, double default_tolerance) {
  if (nodes_.empty() || !nodes_.front().released) {
    return false;
  }
  const Node & n = nodes_.front();
  if (std::hypot(n.x - x, n.y - y) <= tolerance(n, default_tolerance)) {
    traverse(0);
    return true;
  }
  return false;
}

bool OrderTracker::traverseThrough(int64_t seq, double x, double y, double default_tolerance) {
  for (size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i].seq == seq && nodes_[i].released) {
      if (std::hypot(nodes_[i].x - x, nodes_[i].y - y) > tolerance(nodes_[i], default_tolerance)) {
        return false;
      }
      traverse(i);
      return true;
    }
  }
  return false;
}

bool OrderTracker::finishIfAtEnd(double x, double y, double default_tolerance) {
  const auto base = remainingBase();
  return !base.empty() && traverseThrough(base.back().seq, x, y, default_tolerance);
}

size_t segmentLength(const std::vector<Node> & base, double robot_x, double robot_y, double min_distance) {
  for (size_t k = 1; k < base.size(); ++k) {
    if (std::hypot(base[k].x - robot_x, base[k].y - robot_y) < min_distance) {
      return k;
    }
  }
  return base.size();
}

void OrderTracker::cancel() {
  // VDA 5050: after cancelOrder the vehicle stops; remaining nodes and edges are removed from the state.
  nodes_.clear();
  edges_.clear();
}

json buildState(const OrderTracker & t, const StateInputs & in) {
  json nodes = json::array();
  for (const auto & n : t.nodes()) {
    nodes.push_back({{"nodeId", n.id}, {"sequenceId", n.seq}, {"released", n.released}});
  }
  json edges = json::array();
  for (const auto & e : t.edges()) {
    edges.push_back({{"edgeId", e.id}, {"sequenceId", e.seq}, {"released", e.released}});
  }
  json errors = json::array();
  for (const auto & e : in.errors) {
    errors.push_back(e.toJson());
  }
  return json{
    {"headerId", in.header_id},
    {"timestamp", in.timestamp},
    {"version", kVersion},
    {"manufacturer", in.manufacturer},
    {"serialNumber", in.serial_number},
    {"orderId", t.orderId()},
    {"orderUpdateId", t.updateId()},
    {"lastNodeId", t.lastNodeId()},
    {"lastNodeSequenceId", t.lastNodeSeq()},
    {"nodeStates", nodes},
    {"edgeStates", edges},
    {"driving", in.driving},
    {"agvPosition", {{"x", in.x}, {"y", in.y}, {"theta", in.theta}, {"mapId", in.map_id},
                     {"positionInitialized", true}}},
    {"velocity", {{"vx", in.vx}, {"vy", 0.0}, {"omega", in.omega}}},
    {"actionStates", t.actionStates()},
    {"batteryState", {{"batteryCharge", 100.0}, {"charging", false}}},  // not simulated
    {"operatingMode", "AUTOMATIC"},
    {"errors", errors},
    {"safetyState", {{"eStop", in.protective_stop ? "AUTOACK" : "NONE"}, {"fieldViolation", in.field_violation}}},
  };
}

std::string isoTimestampNow() {
  const auto now = std::chrono::system_clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&tt, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
  char out[40];
  std::snprintf(out, sizeof(out), "%s.%03dZ", buf, static_cast<int>(ms));
  return out;
}

}  // namespace tugger_nav::vda
