#ifndef CLOUD_STREAM_H
#define CLOUD_STREAM_H

#include <fstream>
#include <limits>
#include <filesystem>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/util3d.h>

namespace fs = std::filesystem;

class CloudStream {
private:
    std::fstream file_;
    fs::path filePath_;
    size_t count_ = 0, index_;
    const size_t countDigits_ = std::floor(std::log10(std::numeric_limits<size_t>::max())) + 1;
    std::streampos countPos_;
public:
    CloudStream(const fs::path& pathCloud, const size_t index) : index_(index) {
        std::ostringstream cloudName;
        cloudName << "out_" << index << ".ply";

        filePath_ = pathCloud / cloudName.str();
        if(!fs::exists(filePath_.parent_path())) fs::create_directories(filePath_.parent_path());
        file_ = std::fstream(filePath_, std::ios::in | std::ios::out | 
            std::ios::binary | std::ios::trunc);

        file_ << "ply" << std::endl;
        file_ << "format binary_little_endian 1.0" << std::endl;
        file_ << "element vertex ";
        countPos_ = file_.tellp();
        for(size_t i = 0; i < countDigits_; ++i) file_ << " ";
        file_ << std::endl;
        file_ << "property float x" << std::endl;
        file_ << "property float y" << std::endl;
        file_ << "property float z" << std::endl;
        file_ << "property uchar red" << std::endl;
        file_ << "property uchar green" << std::endl;
        file_ << "property uchar blue" << std::endl;
        file_ << "end_header" << std::endl;
    }

    CloudStream(CloudStream&& other) noexcept : 
        filePath_(std::move(other.filePath_)),
        count_(other.count_), 
        countPos_(other.countPos_) {
        file_ = std::move(other.file_);
        other.count_ = 0;
        other.countPos_ = 0;
    }

    CloudStream& operator=(CloudStream&& other) noexcept {
        if(this == &other) return *this;

        file_.close();
        file_ = std::move(other.file_);
        filePath_ = std::move(other.filePath_);

        count_ = other.count_;
        countPos_ = other.countPos_;

        other.count_ = 0;
        other.countPos_ = 0;

        return *this;
    }

    CloudStream(const CloudStream& other) = delete;
    CloudStream& operator=(const CloudStream& other) = delete;

    ~CloudStream() {
        if(file_.is_open()) CloudStream::close();
    }

public:
    void close() {
        if(!file_.is_open()) {
            UWARN("CloudStream has already been closed.");
            return;
        }

        UINFO("Making final modifications to output point cloud [%s].", filePath_.c_str());

        file_.clear();
        file_.seekp(countPos_);
        std::string countStr = std::to_string(count_);
        if(countStr.size() < countDigits_) countStr = std::string(countDigits_ - countStr.size(), '0') + countStr;
        file_.write(countStr.c_str(), countDigits_);
        file_.close();
    }

    size_t pointCount() { return count_; }

    void operator()(const rtabmap::SensorData& sensorData, const rtabmap::Transform& pose) {
        if(!file_.is_open()) {
            UWARN("CloudStream is closed, data cannot be written to it.");
            return;
        }

        if(!sensorData.isValid()) {
            UWARN("CloudStream received invalid SensorData. Skipping frame.");
            return;
        }

        UINFO("Writing frame to output point cloud [%s].", filePath_.c_str());

        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudList = \
            rtabmap::util3d::cloudsRGBFromSensorData(sensorData);

        if(cloudList.empty()) {
            UINFO("Frame is empty, skipping.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloudList[0];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudGlobal(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloudGlobal->reserve(cloud->size());
      
        pcl::transformPointCloud(*cloud, *cloudGlobal, pose.toEigen3f());

        for(const auto& point : cloudGlobal->points) {
            file_.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
            file_.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
            file_.write(reinterpret_cast<const char*>(&point.z), sizeof(float));

            file_.write(reinterpret_cast<const char*>(&point.r), sizeof(std::uint8_t));
            file_.write(reinterpret_cast<const char*>(&point.g), sizeof(std::uint8_t));
            file_.write(reinterpret_cast<const char*>(&point.b), sizeof(std::uint8_t));
        }
        file_.flush();

        count_ += cloudGlobal->points.size();

        UINFO("Output point cloud [%s] now has [%zu] points.", filePath_.c_str(), count_);      
    }
};

#endif // CLOUD_STREAM_H