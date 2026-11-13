#include "data_set_gen/data_set_storage.hpp"

#include <ros/package.h>
#include <hdf5.h>
#include <H5Cpp.h>
#include <filesystem>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fs = std::filesystem;
using namespace H5;

namespace dataSet_gen
{

    DataSetStorage::DataSetStorage(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, int records_per_file)
        : nh_(nh), nh_priv_(nh_priv), records_per_file_(records_per_file), running_(true)
    {
        initializeDataDirectory();
        writer_thread_ = std::thread(&DataSetStorage::writerLoop, this);
    }

    DataSetStorage::~DataSetStorage()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = false;
            cond_var_.notify_all();
        }
        writer_thread_.join();
    }

    void DataSetStorage::initializeDataDirectory()
    {
        std::string package_path = ros::package::getPath("data_set_gen");
        base_path_ = package_path + "/dataSetFile";

        try
        {
            if (fs::exists(base_path_))
            {
                fs::remove_all(base_path_);
            }
            fs::create_directories(base_path_);
            std::cout << "\033[32m [save Dataset]: Dataset saving dir created: \033[0m" << base_path_ << std::endl;
        }
        catch (const fs::filesystem_error &e)
        {
            std::cerr << "Filesystem Error: " << e.what() << std::endl;
            throw;
        }
    }

    static Eigen::MatrixXd pointCloudToEigenMatrix(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        Eigen::MatrixXd matrix(cloud->points.size(), 3);
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            matrix(i, 0) = cloud->points[i].x;
            matrix(i, 1) = cloud->points[i].y;
            matrix(i, 2) = cloud->points[i].z;
        }
        return matrix;
    }

    static void saveToHDF5(const std::string &filename, const DataRecord &record)
    {
        try
        {
            H5::H5File file(filename, H5F_ACC_RDWR | H5F_ACC_CREAT);

            std::string group_name = "/record_" + std::to_string(record.record_id % 10000);
            H5::Group group = file.createGroup(group_name);

            hsize_t dronePos_dims[1] = {3};
            H5::DataSpace dronePos_space(1, dronePos_dims);
            H5::DataSet dataset_dronePos = group.createDataSet("pos_worldFrame", H5::PredType::NATIVE_DOUBLE, dronePos_space);
            dataset_dronePos.write(record.pos_worldFrame.data(), H5::PredType::NATIVE_DOUBLE);

            H5::DataSpace scalar_space(H5S_SCALAR);
            H5::DataSet dataset_vel = group.createDataSet("vel_magnitude", H5::PredType::NATIVE_DOUBLE, scalar_space);
            dataset_vel.write(&record.vel_magnitude, H5::PredType::NATIVE_DOUBLE);

            hsize_t acc_dims[1] = {3};
            H5::DataSpace acc_space(1, acc_dims);
            H5::DataSet dataset_acc = group.createDataSet("acc_velFrame", H5::PredType::NATIVE_DOUBLE, acc_space);
            dataset_acc.write(record.acc_velFrame.data(), H5::PredType::NATIVE_DOUBLE);

            hsize_t target_dims[1] = {3};
            H5::DataSpace target_space(1, target_dims);
            H5::DataSet dataset_target = group.createDataSet("target_pos_worldFrame", H5::PredType::NATIVE_DOUBLE, target_space);
            dataset_target.write(record.target_pos_worldFrame.data(), H5::PredType::NATIVE_DOUBLE);

            hsize_t dir_target_dims[1] = {3};
            H5::DataSpace dir_target_space(1, dir_target_dims);
            H5::DataSet dataset_target_dir = group.createDataSet("dir_target_from_drone", H5::PredType::NATIVE_DOUBLE, dir_target_space);
            dataset_target_dir.write(record.dir_target_from_drone.data(), H5::PredType::NATIVE_DOUBLE);

            Eigen::MatrixXd cloud_matrix = pointCloudToEigenMatrix(record.processed_sensedCloud_velFrame);
            hsize_t cloud_dims[2] = {static_cast<hsize_t>(cloud_matrix.cols()), static_cast<hsize_t>(cloud_matrix.rows())};
            H5::DataSpace cloud_space(2, cloud_dims);
            H5::DataSet dataset_cloud;
            if (cloud_matrix.rows() == 0)
            {
                dataset_cloud = group.createDataSet("processed_sensedCloud_velFrame", H5::PredType::NATIVE_DOUBLE, cloud_space);
            }
            else
            {
                H5::DSetCreatPropList prop;
                hsize_t chunk_dims[2] = {static_cast<hsize_t>(cloud_matrix.cols()), std::min<hsize_t>(1000, cloud_matrix.rows())};
                prop.setChunk(2, chunk_dims);
                prop.setDeflate(6);
                dataset_cloud = group.createDataSet("processed_sensedCloud_velFrame", H5::PredType::NATIVE_DOUBLE, cloud_space, prop);
                dataset_cloud.write(cloud_matrix.data(), H5::PredType::NATIVE_DOUBLE);
            }

            hsize_t quat_dims[1] = {4};
            H5::DataSpace quat_space(1, quat_dims);
            H5::DataSet dataset_quat = group.createDataSet("quaterniond", H5::PredType::NATIVE_DOUBLE, quat_space);
            Eigen::Vector4d quat_vec;
            quat_vec << record.quaterniond.w(), record.quaterniond.x(), record.quaterniond.y(), record.quaterniond.z();
            dataset_quat.write(quat_vec.data(), H5::PredType::NATIVE_DOUBLE);

            H5::DataSet dataset_duration = group.createDataSet("duration", H5::PredType::NATIVE_DOUBLE, scalar_space);
            dataset_duration.write(&record.duration, H5::PredType::NATIVE_DOUBLE);

            hsize_t coeffs_dims[2] = {static_cast<hsize_t>(record.coeffs.cols()), static_cast<hsize_t>(record.coeffs.rows())};
            H5::DataSpace coeffs_space(2, coeffs_dims);
            H5::DSetCreatPropList coeffs_prop;
            hsize_t coeffs_chunk_dims[2] = {static_cast<hsize_t>(record.coeffs.cols()), static_cast<hsize_t>(record.coeffs.rows())};
            coeffs_prop.setChunk(2, coeffs_chunk_dims);
            coeffs_prop.setDeflate(6);
            H5::DataSet dataset_coeffs = group.createDataSet("coeffs", H5::PredType::NATIVE_DOUBLE, coeffs_space, coeffs_prop);
            dataset_coeffs.write(record.coeffs.data(), H5::PredType::NATIVE_DOUBLE);

            dataset_dronePos.close();
            dataset_vel.close();
            dataset_acc.close();
            dataset_target.close();
            dataset_cloud.close();
            dataset_duration.close();
            dataset_coeffs.close();
            group.close();
            file.close();
        }
        catch (const H5::Exception &e)
        {
            std::cerr << "[DataSetStorage]: HDF5 Error for record_" << record.record_id << ": " << e.getCDetailMsg() << std::endl;
            std::cerr << "[DataSetStorage]: Point cloud rows: " << pointCloudToEigenMatrix(record.processed_sensedCloud_velFrame).rows() << std::endl;
            std::cerr << "[DataSetStorage]: Skipping record_" << record.record_id << " and continuing." << std::endl;
        }
        catch (...)
        {
            std::cerr << "[DataSetStorage]: Unknown error for record_" << record.record_id << ", skipping and continuing." << std::endl;
        }
    }

    void DataSetStorage::addRecord(const DataRecord &record)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_queue_.push(record);
        cond_var_.notify_one();
    }

    void DataSetStorage::writerLoop()
    {
        int current_file_index = 0;
        int records_in_current_file = 0;
        std::string current_filename = base_path_ + "/dataset_part_" + std::to_string(current_file_index) + ".h5";

        while (true)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cond_var_.wait(lock, [this]
                           { return !data_queue_.empty() || !running_; });

            if (!running_ && data_queue_.empty())
            {
                break;
            }

            while (!data_queue_.empty())
            {
                DataRecord record = data_queue_.front();
                data_queue_.pop();
                lock.unlock();

                saveToHDF5(current_filename, record);
                records_in_current_file++;

                if (records_in_current_file >= records_per_file_)
                {
                    current_file_index++;
                    current_filename = base_path_ + "/dataset_part_" + std::to_string(current_file_index) + ".h5";
                    records_in_current_file = 0;
                }

                lock.lock();
            }
        }
    }

}