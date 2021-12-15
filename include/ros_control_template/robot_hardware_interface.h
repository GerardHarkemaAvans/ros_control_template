#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>


class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(ros::NodeHandle& nh);
  ~MyRobot();
	void init();
  void update(const ros::TimerEvent& e);
	void read();
	void write(ros::Duration elapsed_time);

private:

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface position_joint_interface;

	int num_joints;
	std::string joint_names[1];
	double joint_position[1];
	double joint_velocity[1];
	double joint_effort[1];
	double joint_position_command[1];
  joint_limits_interface::JointLimits limits[1];

	ros::NodeHandle nh;
	ros::Timer non_realtime_loop;
	ros::Duration elapsed_time;
	double loop_hz;
	boost::shared_ptr<controller_manager::ControllerManager> controller_manager;

  ros::Publisher setpoint_value_pub;

};
