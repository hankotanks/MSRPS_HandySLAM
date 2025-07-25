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
    size_t fileIndex_;
    fs::path pathCloud_, pathFile_;
    size_t count_ = 0;
    size_t countMax_;
    const size_t countDigits_ = std::floor(std::log10(std::numeric_limits<size_t>::max())) + 1;
    std::streampos countPos_;
private:
    // TODO: Accept const Config& as a parameter,
    // include maxPoints as an optional value in the CLI arg parser
    // if not supplied, use the max representable size_t
    CloudStream(const fs::path& pathCloud, const size_t maxPoints, const size_t index) : fileIndex_(index), pathCloud_(pathCloud), countMax_(maxPoints) {
        std::ostringstream cloudName;
        cloudName << "out_" << index << ".ply";

        pathFile_ = pathCloud / cloudName.str();
        
        file_ = std::fstream(pathFile_, std::ios::in | std::ios::out | 
            std::ios::binary | std::ios::trunc);

        if(!file_.is_open()) {
            UWARN("Failed to open output PLY file [%s].", pathFile_.c_str());
            std::exit(1);
        }

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

    CloudStream& operator=(CloudStream&& other) noexcept {
        if(this == &other) return *this;

        file_.close();
        file_ = std::move(other.file_);

        fileIndex_ = other.fileIndex_;

        pathCloud_ = std::move(other.pathCloud_);
        pathFile_ = std::move(other.pathFile_);

        count_ = other.count_;
        countMax_ = other.countMax_;
        countPos_ = other.countPos_;

        return *this;
    }

public:
    CloudStream(const fs::path& pathCloud, const size_t maxPoints) : CloudStream(pathCloud, maxPoints, 0) { /* STUB */ }

    CloudStream(CloudStream&&) = delete;
    CloudStream(const CloudStream&) = delete;
    CloudStream& operator=(const CloudStream&) = delete;

    ~CloudStream() { 
        if(file_.is_open()) CloudStream::close(); 
    }

public:
    void close() {
        if(!file_.is_open()) {
            UWARN("CloudStream has already been closed.");
            return;
        }

        UINFO("Making final modifications to output point cloud [%s].", pathFile_.c_str());

        file_.clear();
        file_.seekp(countPos_);
        std::string countStr = std::to_string(count_);
        if(countStr.size() < countDigits_) countStr = std::string(countDigits_ - countStr.size(), '0') + countStr;
        file_.write(countStr.c_str(), countDigits_);
        file_.close();
    }

public:
    friend CloudStream& operator<<(CloudStream& stream, const std::pair<const rtabmap::SensorData&, const rtabmap::Transform&>& frame) {
        const auto& [sensorData, pose] = frame;

        if(!stream.file_.is_open()) {
            UWARN("CloudStream is closed, data cannot be written to it.");
            return stream;
        }

        if(!sensorData.isValid()) {
            UWARN("CloudStream received invalid SensorData. Skipping frame.");
            return stream;
        }

        UINFO("Writing frame to output point cloud [%s].", stream.pathFile_.c_str());

        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudList = \
            rtabmap::util3d::cloudsRGBFromSensorData(sensorData);

        if(cloudList.empty()) {
            UINFO("Frame is empty, skipping.");
            return stream;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloudList[0];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudGlobal(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloudGlobal->reserve(cloud->size());
      
        pcl::transformPointCloud(*cloud, *cloudGlobal, pose.toEigen3f());

        for(const auto& point : cloudGlobal->points) {
            stream.file_.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
            stream.file_.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
            stream.file_.write(reinterpret_cast<const char*>(&point.z), sizeof(float));

            stream.file_.write(reinterpret_cast<const char*>(&point.r), sizeof(std::uint8_t));
            stream.file_.write(reinterpret_cast<const char*>(&point.g), sizeof(std::uint8_t));
            stream.file_.write(reinterpret_cast<const char*>(&point.b), sizeof(std::uint8_t));
        }

        stream.file_.flush();
        stream.count_ += cloudGlobal->points.size();

        UINFO("Output point cloud [%s] now has [%zu] points.", stream.pathFile_.c_str(), stream.count_);  
        if(stream.count_ > stream.countMax_) {
            stream.close();
            stream = CloudStream(stream.pathCloud_, stream.countMax_, stream.fileIndex_ + 1);
        }  

        return stream;  
    }
};

#endif // CLOUD_STREAM_H