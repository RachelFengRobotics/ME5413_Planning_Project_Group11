#include "me5413_world/math_utils.hpp"
#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world
{

// Dynamic Parameters
double SPEED_TARGET;
double PID_Kp, PID_Ki, PID_Kd;
double STANLEY_K;
bool PARAMS_UPDATED;

// Look ahead distance
double LAD; 
// Angular velocity proportional coefficient
double kp_yaw_error = 2; 


void dynamicParamCallback(const me5413_world::path_trackerConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;
  // PID
  PID_Kp = config.PID_Kp;
  PID_Ki = config.PID_Ki;
  PID_Kd = config.PID_Kd;
  // Disable Stanley
  // STANLEY_K = config.stanley_K;

  //Introduce look ahead distance
  LAD = config.LAD;

  PARAMS_UPDATED = true;
}

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
}

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Calculate absolute errors (wrt to world frame)
  // this->pose_world_goal_ = path->poses[11].pose;
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, path));
  return;
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
}

// Function to find a goal point on the path based on lookahead distance
geometry_msgs::Point PathTrackerNode::findGoalPoint(const tf2::Vector3& point_robot, const nav_msgs::Path::ConstPtr& path, double LAD)
{
  // Iterate over the path
  for(int i = 0; i < path->poses.size(); i++)
  {
    tf2::Vector3 point_path;
    tf2::fromMsg(path->poses[i].pose.position, point_path);
    double distance = tf2::tf2Distance(point_robot, point_path); // Calculate the distance between robot and path point
    // Determine whether distance is greater than set threshold
    if(distance >= LAD)
    {
      return path->poses[i].pose.position;
    }
  }
  return path->poses.back().pose.position;  // No point found within the LAD, return the last point
}

// Function to normalize an angle in the range [-pi, pi)
double PathTrackerNode::normalizeAngle(double angle)
{
  double a = fmod(angle + M_PI, 2.0 * M_PI);
  if(a < 0)
    a += 2.0 * M_PI;
  return a - M_PI;
}

// Disable Stanley
// double PathTrackerNode::computeStanelyControl(const double heading_error, const double cross_track_error, const double velocity)
// {
//   const double stanley_output = -1.0*(heading_error + std::atan2(STANLEY_K*cross_track_error, std::max(velocity, 0.3)));

//   return std::min(std::max(stanley_output, -2.2), 2.2);
// }

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const nav_msgs::Path::ConstPtr& path)
{
  // Heading Error
  tf2::Quaternion q_robot;
  tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
  // tf2::fromMsg(pose_goal.orientation, q_goal);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  // const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);
  double roll, pitch, yaw_robot;
  m_robot.getRPY(roll, pitch, yaw_robot);
  // m_goal.getRPY(roll, pitch, yaw_goal);
  // const double heading_error = unifyAngleRange(yaw_robot - yaw_goal);
  // // Lateral Error
  tf2::Vector3 point_robot, point_goal;
  tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
  // tf2::fromMsg(pose_goal.position, point_goal);
  // const tf2::Vector3 V_goal_robot = point_robot - point_goal;
  // const double angle_goal_robot = std::atan2(V_goal_robot.getY(), V_goal_robot.getX());
  // const double angle_diff = angle_goal_robot - yaw_goal;
  // const double lat_error = V_goal_robot.length()*std::sin(angle_diff);

  // Velocity
  tf2::Vector3 robot_vel;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
  const double velocity = robot_vel.length();

  geometry_msgs::Twist cmd_vel;
  if (PARAMS_UPDATED)
  {
    this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
    PARAMS_UPDATED = false;
  }
  cmd_vel.linear.x = this->pid_.calculate(SPEED_TARGET, velocity);
  // cmd_vel.angular.z = computeStanelyControl(heading_error, lat_error, velocity);

  // std::cout << "robot velocity is " << velocity << " throttle is " << cmd_vel.linear.x << std::endl;
  // std::cout << "lateral error is " << lat_error << " heading_error is " << heading_error << " steering is " << cmd_vel.angular.z << std::endl;

  // Find the goal point and calculate the error
  geometry_msgs::Point goal_point = findGoalPoint(point_robot, path, LAD);
  double yaw_goal = atan2(goal_point.y - point_robot.y(), goal_point.x - point_robot.x());
  double yaw_error = normalizeAngle(yaw_goal - yaw_robot); // normalize it to [-pi, pi)

  // Controller output angle control
  cmd_vel.angular.z = kp_yaw_error * yaw_error;    // kp_yaw_error is proportional coefficient
  return cmd_vel;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}
