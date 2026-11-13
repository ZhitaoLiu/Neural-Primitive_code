#ifndef DATA_SET_STORAGE_HPP
#define DATA_SET_STORAGE_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>

namespace dataSet_gen
{

    struct DataRecord
    {
        int record_id;
        Eigen::Vector3d pos_worldFrame;
        double vel_magnitude;
        Eigen::Vector3d acc_velFrame;
        Eigen::Vector3d target_pos_worldFrame;
        Eigen::Vector3d dir_target_from_drone;
        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_sensedCloud_velFrame;
        Eigen::Quaterniond quaterniond;
        double duration;
        Eigen::MatrixXd coeffs;
    };

    class DataSetStorage
    {
    public:
        DataSetStorage(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, int records_per_file);
        ~DataSetStorage();

        void addRecord(const DataRecord &record);

    private:
        void initializeDataDirectory();
        void writerLoop();

        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;
        std::string base_path_;
        int records_per_file_;
        std::queue<DataRecord> data_queue_;
        std::mutex mutex_;
        std::condition_variable cond_var_;
        std::thread writer_thread_;
        std::atomic<bool> running_;
    };

}

#endif