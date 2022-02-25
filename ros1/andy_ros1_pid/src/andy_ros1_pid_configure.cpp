#include "include/andy_ros1_pid_core.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "andy_ros1_pid_configure");
  ros::NodeHandle nh;

  AndyRos1PID *andy_ros1_pid = new AndyRos1PID();

  dynamic_reconfigure::Server<andy_ros1_pid::andy_ros1_pidConfig> dr_srv;
  dynamic_reconfigure::Server<andy_ros1_pid::andy_ros1_pidConfig>::CallbackType cb;
  cb = boost::bind(&AndyRos1PID::configCallback, andy_ros1_pid, _1, _2);
  dr_srv.setCallback(cb);

  double p;
  double d;
  double i;
  int rate;

  ros::NodeHandle pnh("~");
  pnh.param("p", p, 0.6);
  pnh.param("d", d, 0.3);
  pnh.param("i", i, 0.5);
  pnh.param("rate", rate, 1);

  ros::Publisher pub_message = nh.advertise<andy_ros1_pid ::PID>("pid", 10);

  ros::Rate r(rate);

  while (nh.ok())
  {
    andy_ros1_pid->publishMessage(&pub_message);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
