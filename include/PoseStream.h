#ifndef POSE_STREAM_H
#define POSE_STREAM_H

#include <filesystem>
#include <fstream>

#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/ULogger.h>

#include "Config.h"

namespace fs = std::filesystem;

class PoseStream {
private:
    fs::path filePath_;
    std::ofstream file_;
    size_t poseCount_ = 0;
public:
    PoseStream(const Config& cfg) {
        filePath_ = std::get<1>(cfg.getPaths()) / "poses.csv";
        file_ = std::ofstream(filePath_);
    }

    ~PoseStream() { if(file_.is_open()) PoseStream::close(); }
public:
    void close() {
        if(!file_.is_open()) {
            UWARN("PoseStream has already been closed.");
            return;
        }

        UINFO("Closing output pose file after writing [%zu] poses [%s].", poseCount_, filePath_.c_str());

        file_.close();
    }

    void operator()(const rtabmap::Transform& pose, const double stamp) {
        auto t = pose.translation();
        auto q = pose.getQuaterniond();
        file_ << std::setprecision(8) << std::fixed << stamp << ", ";
        file_ << std::setprecision(9) << std::fixed << t.x() << ", ";
        file_ << std::setprecision(9) << std::fixed << t.y() << ", ";
        file_ << std::setprecision(9) << std::fixed << t.z() << ", ";
        file_ << std::setprecision(9) << std::fixed << q.x() << ", ";
        file_ << std::setprecision(9) << std::fixed << q.y() << ", ";
        file_ << std::setprecision(9) << std::fixed << q.z() << ", ";
        file_ << std::setprecision(9) << std::fixed << q.w() << ", " << std::endl;
        poseCount_++;
    } 
};

#endif // POSE_STREAM_H