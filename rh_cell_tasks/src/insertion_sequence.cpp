// Remote-handling cell — Phase 1, step 1
// UR5e removes a "cassette" from a pedestal and inserts it through a tight port
// in a wall (10 mm clearance per side), then extracts it and puts it back.
//
// Run with the UR MoveIt simulation already up:
//   ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
//   ros2 run rh_cell_tasks insertion_sequence --ros-args -p use_sim_time:=true

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

namespace
{
// ---------------------------------------------------------------- cell layout (metres)
// Insertion direction is +x. The tool's z-axis points along +x for every pose.
constexpr double kPortX = 0.60;          // wall centre plane
constexpr double kWallThick = 0.02;      // wall spans x = 0.59 .. 0.61
constexpr double kPortY = -0.25;         // port centre
constexpr double kPortZ = 0.35;
constexpr double kPortSize = 0.09;       // square opening, 90 mm
constexpr double kWallYMin = -0.55, kWallYMax = 0.05;
constexpr double kWallZMin = 0.10, kWallZMax = 0.60;

constexpr double kCasLen = 0.12;         // along x (insertion axis)
constexpr double kCasW = 0.07;           // y and z: 70 mm -> 10 mm clearance per side
constexpr double kPedX = 0.45, kPedY = 0.25, kCasZ = 0.35;  // cassette start centre

constexpr double kDefaultSpeed = 0.3;      // planned moves: 30% of joint limits

constexpr double kToolToCasCentre = 0.065;  // tool0 -> cassette centre along +x (5 mm gap + half length)

// tool0 positions along the sequence
constexpr double kGraspX = kPedX - kToolToCasCentre;                                 // 0.385
constexpr double kPreGraspX = kGraspX - 0.085;                                       // 0.300
constexpr double kLift = 0.05;
constexpr double kPreInsertX = 0.50 - kToolToCasCentre;                              // cassette front 30 mm before wall
constexpr double kInsertX = (kPortX - kWallThick / 2.0 + kCasLen / 2.0) - kToolToCasCentre;  // rear face flush with wall

// Tool z-axis along world +x: rotation of +90 deg about y.
geometry_msgs::msg::Pose toolPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = 0.0;
  p.orientation.y = 0.7071067811865476;
  p.orientation.z = 0.0;
  p.orientation.w = 0.7071067811865476;
  return p;
}

moveit_msgs::msg::CollisionObject makeBox(const std::string& id, const std::string& frame,
                                          double sx, double sy, double sz,
                                          double cx, double cy, double cz)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {sx, sy, sz};
  geometry_msgs::msg::Pose pose;
  pose.position.x = cx;
  pose.position.y = cy;
  pose.position.z = cz;
  pose.orientation.w = 1.0;
  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

std::vector<moveit_msgs::msg::CollisionObject> buildCell(const std::string& frame)
{
  const double half = kPortSize / 2.0;
  const double yL0 = kWallYMin, yL1 = kPortY - half;   // left of opening
  const double yR0 = kPortY + half, yR1 = kWallYMax;   // right of opening
  const double zB0 = kWallZMin, zB1 = kPortZ - half;   // below opening
  const double zT0 = kPortZ + half, zT1 = kWallZMax;   // above opening
  const double wallH = kWallZMax - kWallZMin;
  const double wallZc = (kWallZMax + kWallZMin) / 2.0;

  std::vector<moveit_msgs::msg::CollisionObject> objs;
  objs.push_back(makeBox("floor", frame, 2.0, 2.0, 0.02, 0.0, 0.0, -0.011));
  objs.push_back(makeBox("wall_left", frame, kWallThick, yL1 - yL0, wallH, kPortX, (yL0 + yL1) / 2.0, wallZc));
  objs.push_back(makeBox("wall_right", frame, kWallThick, yR1 - yR0, wallH, kPortX, (yR0 + yR1) / 2.0, wallZc));
  objs.push_back(makeBox("wall_bottom", frame, kWallThick, kPortSize, zB1 - zB0, kPortX, kPortY, (zB0 + zB1) / 2.0));
  objs.push_back(makeBox("wall_top", frame, kWallThick, kPortSize, zT1 - zT0, kPortX, kPortY, (zT0 + zT1) / 2.0));

  const double pedTop = kCasZ - kCasW / 2.0 - 0.005;  // 5 mm below the cassette
  objs.push_back(makeBox("pedestal", frame, 0.10, 0.10, pedTop, kPedX, kPedY, pedTop / 2.0));
  objs.push_back(makeBox("cassette", frame, kCasLen, kCasW, kCasW, kPedX, kPedY, kCasZ));
  return objs;
}

// Read a latched std_msgs/String topic (robot_state_publisher and move_group publish these).
std::string readLatchedString(const std::string& topic, std::chrono::seconds timeout)
{
  auto helper = std::make_shared<rclcpp::Node>("rh_topic_fetch");
  std::string value;
  auto sub = helper->create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).transient_local().reliable(),
      [&value](const std_msgs::msg::String::SharedPtr msg) { value = msg->data; });
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (value.empty() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(helper);
    std::this_thread::sleep_for(100ms);
  }
  return value;
}

// Read one string parameter from move_group; empty if move_group does not hold it.
std::string readMoveGroupParam(const std::string& name)
{
  auto helper = std::make_shared<rclcpp::Node>("rh_param_fetch");
  rclcpp::SyncParametersClient client(helper, "move_group");
  if (!client.wait_for_service(2s))
    return "";
  const auto params = client.get_parameters({name});
  if (params.empty() || params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    return "";
  return params[0].as_string();
}

// Give this node robot_description and robot_description_semantic so MoveIt can build the
// robot model. Topics first (always published by the UR launch), move_group parameters as fallback.
bool fetchRobotDescription(const rclcpp::Node::SharedPtr& node)
{
  for (const std::string name : {"robot_description", "robot_description_semantic"})
  {
    RCLCPP_INFO(node->get_logger(), "Fetching %s ...", name.c_str());
    std::string value = readLatchedString("/" + name, 15s);
    if (value.empty())
      value = readMoveGroupParam(name);
    if (value.empty())
    {
      RCLCPP_ERROR(node->get_logger(),
                   "Could not get %s from topic or move_group. Is ur_sim_moveit.launch.py fully started?",
                   name.c_str());
      return false;
    }
    if (node->has_parameter(name))
      node->set_parameter(rclcpp::Parameter(name, value));
    else
      node->declare_parameter(name, value);
    RCLCPP_INFO(node->get_logger(), "Got %s (%zu chars).", name.c_str(), value.size());
  }
  return true;
}

class CellSequence
{
public:
  explicit CellSequence(const rclcpp::Node::SharedPtr& node)
    : node_(node), log_(node->get_logger()), mg_(node, "ur_manipulator")
  {
    mg_.setEndEffectorLink("tool0");
    mg_.setPlanningTime(5.0);
    mg_.setNumPlanningAttempts(5);
    mg_.setGoalPositionTolerance(0.002);     // 2 mm
    mg_.setGoalOrientationTolerance(0.01);   // ~0.6 deg
    mg_.setMaxVelocityScalingFactor(kDefaultSpeed);
    mg_.setMaxAccelerationScalingFactor(kDefaultSpeed);
    frame_ = mg_.getPlanningFrame();
    RCLCPP_INFO(log_, "Planning frame: %s | EEF link: %s", frame_.c_str(), mg_.getEndEffectorLink().c_str());
  }

  bool setupScene()
  {
    mg_.detachObject("cassette");  // clean up a previous run
    psi_.removeCollisionObjects({"floor", "wall_left", "wall_right", "wall_bottom", "wall_top", "pedestal", "cassette"});
    std::this_thread::sleep_for(500ms);
    if (!psi_.applyCollisionObjects(buildCell(frame_)))
    {
      RCLCPP_ERROR(log_, "Failed to add the cell to the planning scene.");
      return false;
    }
    RCLCPP_INFO(log_, "Cell added: wall with %.0f mm port, cassette %.0f mm -> %.0f mm clearance per side.",
                kPortSize * 1000, kCasW * 1000, (kPortSize - kCasW) * 500);
    return true;
  }

  bool run()
  {
    const std::vector<std::pair<std::string, std::function<bool()>>> steps = {
      {"Move to ready", [&] { return moveJoints({0.0, -1.57, 1.57, -1.57, -1.57, 0.0}); }},
      {"Pre-grasp", [&] { return movePlanned(toolPose(kPreGraspX, kPedY, kCasZ)); }},
      {"Approach cassette", [&] { return moveLinear(toolPose(kGraspX, kPedY, kCasZ), 0.1); }},
      {"Grip (attach)", [&] { return attach(); }},
      {"Lift off pedestal", [&] { return moveLinear(toolPose(kGraspX, kPedY, kCasZ + kLift), 0.1); }},
      {"Back away", [&] {
         if (!moveLinear(toolPose(kPreGraspX, kPedY, kCasZ + kLift), 0.2))
           return false;
         std::this_thread::sleep_for(300ms);
         q_above_pedestal_ = mg_.getCurrentJointValues();  // remembered for the return trip
         return !q_above_pedestal_.empty();
       }},
      {"Pre-insert at port", [&] { return movePlanned(toolPose(kPreInsertX, kPortY, kPortZ)); }},
      {"INSERT through port", [&] { return moveLinear(toolPose(kInsertX, kPortY, kPortZ), 0.05); }},
      {"Hold inserted", [&] { std::this_thread::sleep_for(2s); return true; }},
      {"EXTRACT from port", [&] { return moveLinear(toolPose(kPreInsertX, kPortY, kPortZ), 0.05); }},
      // Joint-space goal recorded on the way out: no IK sampling, so it cannot miss.
      {"Return above pedestal", [&] { return moveJoints(q_above_pedestal_); }},
      {"Approach pedestal", [&] { return moveLinear(toolPose(kGraspX, kPedY, kCasZ + kLift), 0.1); }},
      {"Lower onto pedestal", [&] { return moveLinear(toolPose(kGraspX, kPedY, kCasZ), 0.05); }},
      {"Release (detach)", [&] { return detach(); }},
      {"Retreat", [&] { return moveLinear(toolPose(kPreGraspX, kPedY, kCasZ), 0.2); }},
      {"Back to ready", [&] { return moveJoints({0.0, -1.57, 1.57, -1.57, -1.57, 0.0}); }},
    };

    const auto t0 = node_->now();
    for (std::size_t i = 0; i < steps.size(); ++i)
    {
      RCLCPP_INFO(log_, "[%2zu/%zu] %s", i + 1, steps.size(), steps[i].first.c_str());
      if (!steps[i].second())
      {
        RCLCPP_ERROR(log_, "Step failed: %s. Stopping the sequence.", steps[i].first.c_str());
        return false;
      }
    }
    RCLCPP_INFO(log_, "Cycle complete in %.1f s (sim time).", (node_->now() - t0).seconds());
    return true;
  }

private:
  bool moveJoints(const std::vector<double>& q)
  {
    mg_.setJointValueTarget(q);
    return planAndExecute();
  }

  bool movePlanned(const geometry_msgs::msg::Pose& target)
  {
    mg_.setPoseTarget(target, "tool0");
    return planAndExecute();
  }

  // Pose goals use a sampled IK seed, so a single attempt can fail on a reachable pose.
  // Retry a few times before giving up.
  bool planAndExecute()
  {
    constexpr int kMaxTries = 5;
    for (int attempt = 1; attempt <= kMaxTries; ++attempt)
    {
      mg_.setStartStateToCurrentState();
      MoveGroupInterface::Plan plan;
      if (static_cast<bool>(mg_.plan(plan)))
        return static_cast<bool>(mg_.execute(plan));
      RCLCPP_WARN(log_, "Planning attempt %d/%d failed%s", attempt, kMaxTries,
                  attempt < kMaxTries ? ", retrying..." : ".");
      std::this_thread::sleep_for(300ms);
    }
    RCLCPP_ERROR(log_, "Planning failed. Check terminal 1 for the move_group reason (IK / collision).");
    return false;
  }

  // Straight-line tool motion, collision-checked. move_group times the path using the robot's
  // joint limits and the scaling factors we send (`speed` = fraction of max velocity/acceleration).
  bool moveLinear(const geometry_msgs::msg::Pose& target, double speed)
  {
    mg_.setMaxVelocityScalingFactor(speed);
    mg_.setMaxAccelerationScalingFactor(speed);
    std::vector<geometry_msgs::msg::Pose> waypoints{target};
    moveit_msgs::msg::RobotTrajectory traj_msg;
    const double fraction = mg_.computeCartesianPath(waypoints, 0.005, traj_msg);
    mg_.setMaxVelocityScalingFactor(kDefaultSpeed);
    mg_.setMaxAccelerationScalingFactor(kDefaultSpeed);
    if (fraction < 0.999)
    {
      RCLCPP_ERROR(log_, "Straight-line path only %.1f%% feasible.", fraction * 100.0);
      return false;
    }
    return static_cast<bool>(mg_.execute(traj_msg));
  }

  bool attach()
  {
    const bool ok = mg_.attachObject("cassette", "tool0", {"tool0", "wrist_3_link"});
    std::this_thread::sleep_for(500ms);
    return ok;
  }

  bool detach()
  {
    const bool ok = mg_.detachObject("cassette");
    std::this_thread::sleep_for(500ms);
    return ok;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger log_;
  MoveGroupInterface mg_;
  PlanningSceneInterface psi_;
  std::string frame_;
  std::vector<double> q_above_pedestal_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "rh_insertion_sequence", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  if (!fetchRobotDescription(node))
  {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });

  int rc = 1;
  {
    CellSequence cell(node);
    if (cell.setupScene() && cell.run())
      rc = 0;
  }

  rclcpp::shutdown();
  spinner.join();
  return rc;
}
