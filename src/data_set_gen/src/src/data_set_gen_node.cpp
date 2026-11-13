#include "data_set_gen/data_set_generator.hpp"

using namespace dataSet_gen;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "data_set_gen_node");
    ros::NodeHandle nh, nh_priv("~");

    DataSetGenerator generator(nh, nh_priv);
    generator.run(nh);

    ros::shutdown();
    return 0;
}
