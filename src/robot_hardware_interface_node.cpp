#include "ros_control_template/robot_hardware_interface.h"
#include "std_msgs/Float64.h"


MyRobot::MyRobot(ros::NodeHandle& nh) : nh(nh) {
    init();
    controller_manager.reset(new controller_manager::ControllerManager(this, nh));
    loop_hz=4;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz);

	//pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino",10);
	//client = nh_.serviceClient<three_dof_planar_manipulator::Floats_array>("/read_joint_state");

    non_realtime_loop = nh.createTimer(update_freq, &MyRobot::update, this);
}

MyRobot::~MyRobot(){
}

void MyRobot::init() {

    num_joints=1;
	  joint_names[0]="my_joint";

    for (int i = 0; i < num_joints; ++i) {

         // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names[i], &joint_position[i], &joint_velocity[i], &joint_effort[i]);
        joint_state_interface.registerHandle(jointStateHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command[i]);

        position_joint_interface.registerHandle(jointPositionHandle);

        joint_limits_interface::getJointLimits(joint_names[i], nh, limits[i]);

    }
    registerInterface(&joint_state_interface);
    registerInterface(&position_joint_interface);

    setpoint_value_pub = nh.advertise<std_msgs::Float64>("my_controller/setpoint_value", 1000);
}

void MyRobot::read() {

  std_msgs::Float64::ConstPtr actual_value = ros::topic::waitForMessage<std_msgs::Float64>("my_controller/actual_value", ros::Duration(1));
  if (actual_value == NULL){
      ROS_INFO("No actual value messages received");
  }
  else{
    joint_position[0] = actual_value->data;
    joint_velocity[0] = 0;
    joint_effort[0] = 0;
  }

}

void MyRobot::write(ros::Duration elapsed_time) {

  std_msgs::Float64 setpoint_value;
  setpoint_value.data = joint_position_command[0];

	setpoint_value_pub.publish(setpoint_value);

}

void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager->update(ros::Time::now(), elapsed_time);
    write(elapsed_time);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_actuator_hardware_interface");
  ros::NodeHandle nh;
  //ros::AsyncSpinner spinner(2);
  ros::MultiThreadedSpinner spinner(2);// 2 threads for controller service and for the Service client used to get the feedback from ardiuno
  //spinner.start();
  MyRobot ROBOT(nh);
  spinner.spin();
  //ros::spin();
  return 0;
}
