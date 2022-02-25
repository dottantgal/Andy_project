#include "include/andy_ros1_pid_core.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "andy_ros1_pid_listener");
  ros::NodeHandle nh;

  int rate;

  ros::NodeHandle pnh("~");
  pnh.param("rate", rate, int(40));

  AndyRos1PID *andy_ros1_pid = new AndyRos1PID();

  ros::Subscriber sub_message = nh.subscribe("pid", 1000, &AndyRos1PID::messageCallback, andy_ros1_pid);

  ros::Rate r(rate);

  // Main loop.
  while (nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
