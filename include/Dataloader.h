#ifndef DATALOADER_H
#define DATALOADER_H

#include <filesystem>

#include <rtabmap/core/IMU.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/ULogger.h>

#include "Config.h"

namespace fs = std::filesystem;

class Dataloader {
private:
    bool rebuild_;
    bool upscale_;
    fs::path pathData_, pathTemp_;
    fs::path pathColor_, pathDepth_;
    std::vector<rtabmap::IMUEvent> events_;
public:
    Dataloader(const Config& cfg);
    // data must past validation to perform SLAM
    bool validate(const bool silent = false) const;
    virtual void process() = 0; // THE ONLY METHOD THAT SHOULD BE IMPLEMENTED ON DERIVED CLASSES
    // initializer (invokes Dataloader::process and can't be overriden)
    virtual void init() final { 
        if(!rebuild_) return;
        UINFO("Began preprocessing scene data.");
        process(); 
        UINFO("Finished preprocessing scene data."); }
    // helper methods
    cv::Size splitColorVideoAndScale(const fs::path& pathColorIn) const; // returns the original image dimensions
    void upscaleDepth(const fs::path& pathDepthIn) const; // either does dumb upscaling or PromptDA
    void writeCalibration(const rtabmap::Transform& intrinsics, const cv::Size& originalColorSize) const;
    void storeEvents(std::vector<rtabmap::IMUEvent>&& events);
    // getters
    fs::path getPathData() const { return pathData_; }
    fs::path getPathDB() const { return (pathTemp_ / "temp.db"); }
    fs::path getPathIMU() const { return (pathTemp_ / "imu.csv"); }
    fs::path getPathColor() const { return pathColor_; }
    fs::path getPathDepth() const { return pathDepth_; }
    std::pair<fs::path, std::string> getPathCalibration() const { 
        return std::make_pair(pathTemp_, "handy_camera"); } // returns calibration path and camera name
    fs::path getPathCalibrationFile() const {
        auto [pathCalibration, cameraName] = Dataloader::getPathCalibration();
        return pathCalibration / (cameraName + ".yaml"); }
    fs::path getPathStamps() const { return (pathTemp_ / "stamps.txt"); }
};

#endif // DATALOADER_H