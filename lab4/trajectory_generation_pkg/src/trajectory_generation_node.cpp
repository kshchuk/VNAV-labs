#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>

#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class WaypointFollower : public rclcpp::Node {

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr currentStateSub;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poseArraySub;

  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::
      SharedPtr desiredStatePub;

  // Current state
  Eigen::Vector3d x; // current position of the UAV's c.o.m. in the world frame

  rclcpp::TimerBase::SharedPtr desiredStateTimer;

  rclcpp::Time trajectoryStartTime;
  bool trajectory_time_initialized_{false};
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory yaw_trajectory;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void onCurrentState(nav_msgs::msg::Odometry const &cur_state) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.1 |  16.485 - Fall 2024  - Lab 4 coding assignment (5 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  Populate the variable x, which encodes the current world position of the
    //  UAV
    // ~~~~ begin solution

    x = Eigen::Vector3d(cur_state.pose.pose.position.x,
                        cur_state.pose.pose.position.y,
                        cur_state.pose.pose.position.z);

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.1
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void
  generateOptimizedTrajectory(geometry_msgs::msg::PoseArray const &poseArray) {
    if (poseArray.poses.size() < 1) {
      RCLCPP_ERROR(get_logger(),
                   "Must have at least one pose to generate trajectory!");
      trajectory.clear();
      yaw_trajectory.clear();
      return;
    }

    if (!trajectory.empty())
      return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.2 |  16.485 - Fall 2024  - Lab 4 coding assignment (35 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  We are using the mav_trajectory_generation library
    //  (https://github.com/ethz-asl/mav_trajectory_generation) to perform
    //  trajectory optimization given the waypoints (based on the position and
    //  orientation of the gates on the race course).
    //  We will be finding the trajectory for the position and the trajectory
    //  for the yaw in a decoupled manner.
    //  In this section:
    //  1. Fill in the correct number for D, the dimension we should apply to
    //  the solver to find the positional trajectory
    //  2. Correctly populate the Vertex::Vector structure below (vertices,
    //  yaw_vertices) using the position of the waypoints and the yaw of the
    //  waypoints respectively
    //
    //  Hints:
    //  1. Use vertex.addConstraint(POSITION, position) where position is of
    //  type Eigen::Vector3d to enforce a waypoint position.
    //  2. Use vertex.addConstraint(ORIENTATION, yaw) where yaw is a double
    //  to enforce a waypoint yaw.
    //  3. Remember angle wraps around 2 pi. Be careful!
    //  4. For the ending waypoint for position use .makeStartOrEnd as seen with
    //  the starting waypoint instead of .addConstraint as you would do for the
    //  other waypoints.
    //
    // ~~~~ begin solution

    // for access to SNAP
    using namespace mav_trajectory_generation::derivative_order;
    
    const int D = 3; // dimension of each vertex in the trajectory (x, y, z)
    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex::Vector yaw_vertices;

    size_t num_waypoints = poseArray.poses.size();
    vertices.reserve(num_waypoints);
    yaw_vertices.reserve(num_waypoints);

    double prev_yaw = 0.0;

    for (size_t i = 0; i < num_waypoints; ++i) {
      const auto &pose = poseArray.poses[i];

      // Position constraint
      Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
      mav_trajectory_generation::Vertex v(D);

      if (i == 0 || i == num_waypoints - 1) {
        v.makeStartOrEnd(position, SNAP);
      } else {
        v.addConstraint(POSITION, position);
      }
      vertices.push_back(v);

      // Yaw constraint
      tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      // Unroll yaw to prevent 2*pi phase jumps between waypoints
      if (i == 0) {
        prev_yaw = yaw;
      } else {
        double diff = yaw - prev_yaw;
        while (diff > M_PI)  diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        yaw = prev_yaw + diff;
        prev_yaw = yaw;
      }

      mav_trajectory_generation::Vertex yaw_v(1);
      if (i == 0 || i == num_waypoints - 1) {
        yaw_v.makeStartOrEnd(yaw, SNAP);
      } else {
        yaw_v.addConstraint(ORIENTATION, yaw);
      }
      yaw_vertices.push_back(yaw_v);
    }

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.2
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // ============================================================
    // Estimate the time to complete each segment of the trajectory
    // ============================================================

    std::vector<double> segment_times;
    const double v_max = 8.0;
    const double a_max = 4.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    // =====================================================
    // Solve for the optimized trajectory (linear optimizer)
    // =====================================================
    // Position
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(D);
    opt.setupFromVertices(vertices, segment_times, SNAP);
    opt.solveLinear();

    // Yaw
    mav_trajectory_generation::PolynomialOptimization<N> yaw_opt(1);
    yaw_opt.setupFromVertices(yaw_vertices, segment_times, SNAP);
    yaw_opt.solveLinear();

    // ============================
    // Get the optimized trajectory
    // ============================
    mav_trajectory_generation::Segment::Vector segments;
    //        opt.getSegments(&segments); // Unnecessary?
    opt.getTrajectory(&trajectory);
    yaw_opt.getTrajectory(&yaw_trajectory);
    trajectory_time_initialized_ = false;

    RCLCPP_INFO(get_logger(),
                "Generated optimizes trajectory from %zu waypoints (duration %.2fs)",
                vertices.size(), trajectory.getMaxTime());
  }

  void publishDesiredState() {
    if (trajectory.empty())
      return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.3 |  16.485 - Fall 2024  - Lab 4 coding assignment (15 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  Finally we get to send commands to our controller! First fill in
    //  properly the value for 'nex_point.time_from_start' and 'sampling_time'
    //  (hint: not 0) and after extracting the state information from our
    //  optimized trajectory, finish populating next_point.
    //
    // ~~~~ begin solution

    using namespace mav_trajectory_generation::derivative_order;

    // Start the trajectory clock on the first publish after optimization, once
    // Gazebo sim time (/clock) is advancing. Setting this in generateOptimizedTrajectory
    // can pin sampling_time to 0 if waypoints arrive before sim starts.
    if (!trajectory_time_initialized_) {
      trajectoryStartTime = now();
      trajectory_time_initialized_ = true;
    }

    double sampling_time = (now() - trajectoryStartTime).seconds();
    if (sampling_time < 0.0) {
      sampling_time = 0.0;
    }
    double max_time = trajectory.getMaxTime();

    // Clamp sampling time to final time if trajectory is finished
    if (sampling_time > max_time) {
      sampling_time = max_time;
    }

    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint next_point;
    next_point.time_from_start = rclcpp::Duration::from_seconds(sampling_time);

    // Evaluate position, velocity, and acceleration at current trajectory time
    Eigen::Vector3d pos = trajectory.evaluate(sampling_time, POSITION);
    Eigen::Vector3d vel = trajectory.evaluate(sampling_time, VELOCITY);
    Eigen::Vector3d acc = trajectory.evaluate(sampling_time, ACCELERATION);

    // Evaluate yaw, yaw rate, and yaw acceleration
    double yaw      = yaw_trajectory.evaluate(sampling_time, POSITION)[0];
    double yaw_dot  = yaw_trajectory.evaluate(sampling_time, VELOCITY)[0];
    double yaw_ddot = yaw_trajectory.evaluate(sampling_time, ACCELERATION)[0];

    // Build transform
    geometry_msgs::msg::Transform transform;
    transform.translation.x = pos.x();
    transform.translation.y = pos.y();
    transform.translation.z = pos.z();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    transform.rotation = tf2::toMsg(q);

    // Build velocity (Twist)
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = vel.x();
    velocity.linear.y = vel.y();
    velocity.linear.z = vel.z();
    velocity.angular.z = yaw_dot;

    // Build acceleration (Twist)
    geometry_msgs::msg::Twist acceleration;
    acceleration.linear.x = acc.x();
    acceleration.linear.y = acc.y();
    acceleration.linear.z = acc.z();
    acceleration.angular.z = yaw_ddot;

    next_point.transforms.push_back(transform);
    next_point.velocities.push_back(velocity);
    next_point.accelerations.push_back(acceleration);

    desiredStatePub->publish(next_point);

    geometry_msgs::msg::TransformStamped desired_tf;
    desired_tf.header.stamp = this->now();
    desired_tf.header.frame_id = "world";
    desired_tf.child_frame_id = "av-desired";
    desired_tf.transform = transform;
    tf_broadcaster_->sendTransform(desired_tf);

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.3
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

public:
  explicit WaypointFollower() : Node("waypoint_follower_node") {
    currentStateSub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/current_state", 1,
        std::bind(&WaypointFollower::onCurrentState, this,
                  std::placeholders::_1));

    poseArraySub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/desired_traj_vertices", 1,
        std::bind(&WaypointFollower::generateOptimizedTrajectory, this,
                  std::placeholders::_1));

    desiredStatePub = this->create_publisher<
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>("/desired_state",
                                                            1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    desiredStateTimer =
        create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.2),
                     std::bind(&WaypointFollower::publishDesiredState, this));
    desiredStateTimer->reset();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollower>());
  return 0;
}
