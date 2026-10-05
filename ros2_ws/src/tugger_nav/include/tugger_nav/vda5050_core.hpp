// VDA 5050 v2 order handling without ROS: schema validation, order semantics, node/edge progress, state JSON.
// Implements a subset of the standard (see README "Limits"): base/horizon, order updates, cancelOrder,
// node traversal by distance. Node and edge actions are not executed; they are reported as FAILED.
#pragma once
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace nlohmann::json_schema {
class json_validator;
}

namespace tugger_nav::vda {

using nlohmann::json;
inline constexpr const char * kVersion = "2.1.0";

struct Error {
  std::string type;
  std::string level = "WARNING";  // WARNING or FATAL
  std::string description;
  std::vector<std::pair<std::string, std::string>> refs;
  json toJson() const;
};

// Validates documents against one JSON schema file (the official VDA 5050 schemas, draft-07 compatible subset).
class SchemaValidator {
public:
  explicit SchemaValidator(const std::string & schema_path);
  ~SchemaValidator();
  // Empty string when valid, otherwise the first validation error.
  std::string validate(const json & doc) const;

private:
  std::unique_ptr<nlohmann::json_schema::json_validator> v_;
};

struct Node {
  std::string id;
  int64_t seq = 0;
  bool released = true;
  bool has_position = false;
  double x = 0.0;
  double y = 0.0;
  std::optional<double> theta;
  double allowed_deviation_xy = 0.0;
  std::string map_id;
  json actions = json::array();
};

struct Edge {
  std::string id;
  int64_t seq = 0;
  bool released = true;
  std::string start_node;
  std::string end_node;
  std::optional<double> max_speed;
  json actions = json::array();
};

struct Order {
  std::string order_id;
  int64_t update_id = 0;
  std::vector<Node> nodes;
  std::vector<Edge> edges;
};

// Semantic checks on a schema-valid order: alternating sequence ids, edge connectivity, base before horizon,
// positions on the configured map. Returns the errors; `out` is filled when there are none.
std::vector<Error> parseOrder(const json & doc, const std::string & map_id, Order & out);

class OrderTracker {
public:
  enum class Decision { Accepted, Updated, Ignored, Rejected };
  // Accept a new order or an update; robot pose is used for the "first node is reachable" rule.
  Decision submit(const Order & o, double robot_x, double robot_y, double start_tolerance, std::vector<Error> & errs);
  // Mark the next base node traversed when the robot is within tolerance. Returns true if a node was traversed.
  bool updatePosition(double x, double y, double default_tolerance);
  // Called when a navigation goal ending at node `seq` succeeded: traverse all nodes up to it if the robot is
  // within tolerance of that node (intermediate nodes can be passed just outside their tolerance).
  bool traverseThrough(int64_t seq, double x, double y, double default_tolerance);
  // Same for the last base node.
  bool finishIfAtEnd(double x, double y, double default_tolerance);
  void cancel();

  bool hasOrder() const { return !order_id_.empty(); }
  // True while released (base) nodes are still to be traversed.
  bool active() const;
  // Base nodes not yet traversed, in order (the navigation goals).
  std::vector<Node> remainingBase() const;
  const std::string & orderId() const { return order_id_; }
  int64_t updateId() const { return update_id_; }
  const std::string & lastNodeId() const { return last_node_id_; }
  int64_t lastNodeSeq() const { return last_node_seq_; }
  const std::vector<Node> & nodes() const { return nodes_; }  // not yet traversed (base + horizon)
  const std::vector<Edge> & edges() const { return edges_; }
  const json & actionStates() const { return action_states_; }
  static double tolerance(const Node & n, double default_tolerance) {
    return n.allowed_deviation_xy > 0.0 ? n.allowed_deviation_xy : default_tolerance;
  }

private:
  void traverse(size_t node_index);
  void collectActions(const Order & o);
  std::string order_id_;
  int64_t update_id_ = 0;
  std::string last_node_id_;
  int64_t last_node_seq_ = 0;
  std::vector<Node> nodes_;
  std::vector<Edge> edges_;
  json action_states_ = json::array();
};

// Number of leading base nodes to send to Nav2 as one goal. A goal must not end near where the robot is now:
// Nav2's goal checker compares the robot with the end of the path, so a closed loop (last node = start node)
// would be reported as reached immediately. The segment therefore stops before the first node closer than
// min_distance to the robot; the next segment is sent when this one succeeds. Always >= 1 for a non-empty base.
size_t segmentLength(const std::vector<Node> & base, double robot_x, double robot_y, double min_distance);

struct StateInputs {
  int64_t header_id = 0;
  std::string timestamp;
  std::string manufacturer;
  std::string serial_number;
  std::string map_id;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double vx = 0.0;
  double omega = 0.0;
  bool driving = false;
  bool protective_stop = false;
  bool field_violation = false;
  std::vector<Error> errors;
};

json buildState(const OrderTracker & t, const StateInputs & in);

// ISO 8601 UTC timestamp with milliseconds from the system clock.
std::string isoTimestampNow();

}  // namespace tugger_nav::vda
