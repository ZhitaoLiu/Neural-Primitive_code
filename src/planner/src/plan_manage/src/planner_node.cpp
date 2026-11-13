#include "plan_manage/replan_fsm.h"

#include <ros/ros.h>

using namespace npe2e_planner;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh_priv("~");

    npe2eReplanFSM npe2e_replan;
    npe2e_replan.init(nh_priv);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
