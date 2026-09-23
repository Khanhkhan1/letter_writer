// letter_writer_node.cpp
//
// Draws the first letter of the student's name ("K") with the UR3/UR3e
// end-effector using MoveIt 2 Cartesian path planning.
//
// Program flow:
//   1. Go to the "home" named target (safe retreat pose from the SRDF).
//   2. For each stroke of the letter:
//        a. Point-to-point move to a "pen up" pose above the stroke's
//           starting point (regular joint-space planning).
//        b. Point-to-point move straight down to the stroke's starting
//           point ("pen down").
//        c. computeCartesianPath() across the stroke's waypoints and
//           execute the resulting trajectory ("draw").
//        d. Lift the pen back up, ready for the next stroke.
//
// The letter is decomposed into strokes, each a std::vector<Pose>. This
// makes it straightforward to add new letters: just add another
// buildLetter_X() function returning a std::vector<std::vector<Pose>>.

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

using geometry_msgs::msg::Pose;
using Stroke = std::vector<Pose>;

namespace
{

struct LetterParams
{
  std::string planning_group;
  double origin_x;      // Cartesian origin of the letter, X (m)
  double origin_y;      // Cartesian origin of the letter, Y (m)
  double draw_height;   // Z height of the drawing plane, "pen down" (m)
  double lift_height;   // extra Z added above draw_height for "pen up" (m)
  double letter_height; // height (Y extent) of the letter (m)
  double eef_step;      // Cartesian interpolation resolution (m)
  double vel_scale;
  double accel_scale;
};

// Fixed end-effector orientation: tool z-axis points straight down at the
// drawing plane, x-axis aligned with the plane's horizontal (X) axis.
geometry_msgs::msg::Quaternion penDownOrientation()
{
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0);
  return tf2::toMsg(q);
}

Pose makePose(const LetterParams & p, double x_off, double y_off, double z)
{
  Pose pose;
  pose.orientation = penDownOrientation();
  pose.position.x = p.origin_x + x_off;
  pose.position.y = p.origin_y + y_off;
  pose.position.z = z;
  return pose;
}

// Letter "K": three strokes, each a straight line (2 waypoints is enough --
// computeCartesianPath linearly interpolates between consecutive waypoints).
//   Stroke 1 - the spine: a vertical line from top to bottom.
//   Stroke 2 - the upper arm: from the spine's midpoint up to the top-right
//              corner.
//   Stroke 3 - the lower arm: from the spine's midpoint down to the
//              bottom-right corner.
// Between strokes the pen lifts and returns to the spine's midpoint, mirroring
// how a person writes "K": down-stroke, lift, upper diagonal, lift, lower
// diagonal.
std::vector<Stroke> buildLetterK(const LetterParams & p)
{
  const double z = p.draw_height;
  const double H = p.letter_height;
  const double mid = H / 2.0;
  const double W = 0.7 * H;  // how far the arms extend to the right

  Stroke spine;
  spine.push_back(makePose(p, 0.0, H, z));
  spine.push_back(makePose(p, 0.0, 0.0, z));

  Stroke upper_arm;
  upper_arm.push_back(makePose(p, 0.0, mid, z));
  upper_arm.push_back(makePose(p, W, H, z));

  Stroke lower_arm;
  lower_arm.push_back(makePose(p, 0.0, mid, z));
  lower_arm.push_back(makePose(p, W, 0.0, z));

  return {spine, upper_arm, lower_arm};
}

// wrist_3_joint has no position limits (it's a continuous/unbounded
// revolute joint), and our fixed downward tool orientation is
// rotationally symmetric about its axis: IK has infinitely many
// equally-valid solutions for it, differing by whole 2*pi turns. Solved
// independently per Cartesian waypoint (and independently of whatever
// value the *previous* move happened to leave it at), consecutive
// trajectory points can end up numerically a full turn apart even though
// physically identical -- the controller then tries to spin the wrist
// all the way around to "catch up", which trips the path tolerance
// almost instantly. This walks the trajectory and, for the given joint,
// shifts each point by whole 2*pi increments so it never differs from
// the previous (already-unwrapped) point -- or from the robot's actual
// current state, for the very first point -- by more than pi. A standard
// "phase unwrap", applied here instead of constraining the joint's range
// because constraining the range doesn't stop the wrap-around itself: a
// value near one edge of the allowed range and a value near the other
// edge are still nearly 2*pi apart numerically.
void unwrapContinuousJoint(
  moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::string & joint_name,
  const moveit::core::RobotState & current_state)
{
  auto & jt = trajectory.joint_trajectory;
  const auto it = std::find(jt.joint_names.begin(), jt.joint_names.end(), joint_name);
  if (it == jt.joint_names.end() || jt.points.empty()) {
    return;
  }
  const size_t idx = std::distance(jt.joint_names.begin(), it);

  double reference = current_state.getVariablePosition(joint_name);
  for (auto & point : jt.points) {
    double & value = point.positions[idx];
    while (value - reference > M_PI) {value -= 2.0 * M_PI;}
    while (value - reference < -M_PI) {value += 2.0 * M_PI;}
    reference = value;
  }
}

bool executeCartesianStroke(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const Stroke & waypoints,
  double eef_step,
  double vel_scale,
  double accel_scale,
  const rclcpp::Logger & logger,
  const std::string & label)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;  // disabled, as recommended by MoveIt docs
  const double fraction =
    move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(logger, "%s: Cartesian path planned (%.1f%% achieved)", label.c_str(), fraction * 100.0);

  if (fraction < 0.95) {
    RCLCPP_ERROR(
      logger, "%s: only %.1f%% of the path could be planned, aborting execution",
      label.c_str(), fraction * 100.0);
    return false;
  }

  unwrapContinuousJoint(trajectory, "wrist_3_joint", *move_group.getCurrentState());

  // computeCartesianPath() only interpolates poses; it does not apply
  // setMaxVelocityScalingFactor()/AccelerationScalingFactor() the way a
  // normal OMPL plan does. Without an explicit retime pass here, the
  // trajectory's per-point timestamps can end up nearly degenerate (points
  // effectively back-to-back in time), which the controller then tries to
  // track by crawling through the path far slower than intended. Retime it
  // the same way move_group retimes OMPL plans internally.
  robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt, vel_scale, accel_scale)) {
    RCLCPP_ERROR(logger, "%s: time parameterization of the Cartesian path failed", label.c_str());
    return false;
  }
  rt.getRobotTrajectoryMsg(trajectory);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  const auto result = move_group.execute(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "%s: execution failed (error code %d)", label.c_str(), result.val);
    return false;
  }
  return true;
}

bool moveTo(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const Pose & pose,
  const rclcpp::Logger & logger,
  const std::string & label)
{
  move_group.setPoseTarget(pose);
  const auto result = move_group.move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "%s: point-to-point move failed (error code %d)", label.c_str(), result.val);
    return false;
  }
  return true;
}

// Simulated joint tracking can occasionally (and non-deterministically,
// depending on host CPU load) lag enough to trip the controller's path
// tolerance even for a correctly-planned move; a short retry is the
// simplest robust way to ride that out rather than aborting the whole
// letter on a one-off simulation hiccup.
bool withRetries(
  const std::function<bool()> & attempt,
  int max_attempts,
  const rclcpp::Logger & logger,
  const std::string & label)
{
  for (int attempt_num = 1; attempt_num <= max_attempts; ++attempt_num) {
    if (attempt()) {
      return true;
    }
    if (attempt_num < max_attempts) {
      RCLCPP_WARN(logger, "%s: attempt %d/%d failed, retrying...", label.c_str(), attempt_num, max_attempts);
    }
  }
  return false;
}

// Publishes a persistent LINE_STRIP marker per stroke, tracing the
// end-effector's actual executed path, so the letter's shape stays
// visible in RViz as it is drawn (Gazebo has no usable GUI on macOS, so
// RViz is the only live view on that platform -- see README). Each
// successfully-drawn stroke gets its own marker id: strokes are only
// connected where the letter's design actually connects them (see
// buildLetterK), and a single marker per stroke keeps pen-up transitions
// between strokes from being drawn as a spurious connecting line.
class LetterTracePublisher
{
public:
  explicit LetterTracePublisher(const rclcpp::Node::SharedPtr & node, std::string frame_id)
  : node_(node), frame_id_(std::move(frame_id))
  {
    publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(
      "letter_trace", rclcpp::QoS(10).transient_local());
  }

  void addStroke(const Stroke & stroke)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "letter_trace";
    marker.id = next_id_++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.004;  // line width, meters
    marker.color.r = 1.0;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    for (const auto & pose : stroke) {
      marker.points.push_back(pose.position);
    }
    publisher_->publish(marker);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  std::string frame_id_;
  int next_id_ = 0;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("letter_writer_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  LetterParams p;
  p.planning_group = node->declare_parameter<std::string>("planning_group", "ur_manipulator");
  p.origin_x = node->declare_parameter<double>("origin_x", 0.22);
  p.origin_y = node->declare_parameter<double>("origin_y", 0.08);
  p.draw_height = node->declare_parameter<double>("draw_height", 0.28);
  p.lift_height = node->declare_parameter<double>("lift_height", 0.05);
  p.letter_height = node->declare_parameter<double>("letter_height", 0.20);
  p.eef_step = node->declare_parameter<double>("eef_step", 0.005);
  p.vel_scale = node->declare_parameter<double>("velocity_scaling", 0.3);
  p.accel_scale = node->declare_parameter<double>("acceleration_scaling", 0.3);

  moveit::planning_interface::MoveGroupInterface move_group(node, p.planning_group);
  move_group.setMaxVelocityScalingFactor(p.vel_scale);
  move_group.setMaxAccelerationScalingFactor(p.accel_scale);
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(5);


  const auto logger = node->get_logger();
  const auto strokes = buildLetterK(p);
  const double z_lift = p.draw_height + p.lift_height;
  LetterTracePublisher trace(node, move_group.getPlanningFrame());

  bool ok = true;

  // "up" (wrist folded upward) rather than "home" (wrist level, arm
  // extended toward the table) as the safe retreat/start pose: the robot
  // base sits directly at table/ground height in this simulation (no
  // pedestal), and "home" keeps the forearm low enough to graze the floor.
  constexpr int kMaxAttempts = 15;

  RCLCPP_INFO(logger, "Moving to 'up' named target");
  ok = ok && withRetries(
    [&]() {return move_group.setNamedTarget("up"), move_group.move() == moveit::core::MoveItErrorCode::SUCCESS;},
    kMaxAttempts, logger, "Move to 'up'");

  // Per stroke: [approach (pen up, at this stroke's start)] -> pen down ->
  // draw. There is deliberately no separate "retreat" move after each
  // stroke: the *next* stroke's "approach" already has to lift the pen
  // and travel to its own start position, so retreating first (straight
  // up at the current spot) and then approaching (sideways at height)
  // would just be the same trip broken into two stops. Folding them into
  // one move still ends with the pen lifted before the next pen-down --
  // the "pen up / move / pen down" behaviour the assignment allows -- it
  // just skips the redundant intermediate stop. The pen is only lifted
  // once more at the very end, after the last stroke.
  for (size_t s = 0; ok && s < strokes.size(); ++s) {
    const auto & stroke = strokes[s];
    const std::string label = "Stroke " + std::to_string(s + 1);

    const Pose start = stroke.front();
    const Pose pen_up_start = makePose(
      p, start.position.x - p.origin_x, start.position.y - p.origin_y, z_lift);

    const std::string approach_label = label + " approach (pen up)";
    const std::string pen_down_label = label + " pen down";
    const std::string draw_label = label + " draw";

    ok = ok && withRetries(
      [&]() {return moveTo(move_group, pen_up_start, logger, approach_label);},
      kMaxAttempts, logger, approach_label);
    ok = ok && withRetries(
      [&]() {return moveTo(move_group, start, logger, pen_down_label);},
      kMaxAttempts, logger, pen_down_label);
    ok = ok && withRetries(
      [&]() {
        return executeCartesianStroke(
          move_group, stroke, p.eef_step, p.vel_scale, p.accel_scale, logger, draw_label);
      },
      kMaxAttempts, logger, draw_label);
    if (ok) {
      trace.addStroke(stroke);  // reveal this stroke in RViz once it's actually drawn
    }
  }

  if (ok && !strokes.empty()) {
    const Pose end = strokes.back().back();
    const Pose pen_up_end = makePose(
      p, end.position.x - p.origin_x, end.position.y - p.origin_y, z_lift);
    ok = ok && withRetries(
      [&]() {return moveTo(move_group, pen_up_end, logger, "Final retreat (pen up)");},
      kMaxAttempts, logger, "Final retreat (pen up)");
  }

  if (ok) {
    RCLCPP_INFO(logger, "Letter drawing complete.");
  } else {
    RCLCPP_ERROR(logger, "Letter drawing aborted due to a planning/execution failure.");
  }

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
