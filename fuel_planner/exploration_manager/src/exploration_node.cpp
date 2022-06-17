#include <ros/ros.h>
#include <exploration_manager/fast_exploration_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_public;

  FastExplorationFSM expl_fsm;
  expl_fsm.init(nh, nh_public);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
