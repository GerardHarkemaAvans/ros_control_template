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
  //hardware_interface::JointStateInterface jnt_state_interface;
  //hardware_interface::PositionJointInterface jnt_pos_interface;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;



	int num_joints_;
	std::string joint_names_[1];
	double joint_position_[1];
	double joint_velocity_[1];
	double joint_effort_[1];
	double joint_position_command_[1];
  joint_limits_interface::JointLimits limits[1];

	ros::NodeHandle nh_;
	ros::Timer non_realtime_loop_;
	ros::Duration elapsed_time_;
	double loop_hz_;
	boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};
