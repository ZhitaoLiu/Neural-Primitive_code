#include "map_generator/map_gen.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_generator_node");
  ros::NodeHandle nh, nh_priv("~");

  int obs_num;
  nh_priv.param("obs_num", obs_num, 80);

  randomMapGenerator map_generator(nh, nh_priv);

  map_generator.generateRandomSimulatedMap(obs_num);

  ros::spin();
  return 0;
}
