#ifndef DATALOADER_H
#define DATALOADER_H

#include <filesystem>

#include <rtabmap/core/IMU.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/ULogger.h>

#include "Config.h"

namespace fs = std::filesystem;

#if 0
struct DataloaderMustRebuild {
    bool full = false;
    bool events = false, calibration = false;
    bool colorFrames = false, depthFrames = false;
};
#endif

class Dataloader {
private:
    bool invalid_ = false;
    bool rebuild_;
    bool upscale_;
    fs::path pathData_, pathTemp_;
    fs::path pathColor_, pathDepth_;
    std::vector<rtabmap::IMUEvent> events_;
public:
    Dataloader(const Config& cfg);
    virtual bool process() = 0; // THE ONLY METHOD THAT SHOULD BE IMPLEMENTED ON DERIVED CLASSES
    // initializer (invokes Dataloader::process and can't be overriden)
    virtual bool init() final { 
        if(!rebuild_) {
            UINFO("Using preprocessed scene data from last run.");
            return true; }
        UINFO("Began preprocessing scene data.");
        if(process()) {
            UINFO("Finished preprocessing scene data."); 
            return Dataloader::validate();
        } else UERROR("Failed to preprocess scene data.");
        return false; }
    // helper methods
    cv::Size splitColorVideoAndScale(const fs::path& pathColorIn) const; // returns the original image dimensions
    bool upscaleDepth(const fs::path& pathDepthIn) const; // either does dumb upscaling or PromptDA
    bool writeCalibration(const rtabmap::Transform& intrinsics, const cv::Size& originalColorSize) const;
    bool storeEvents(std::vector<rtabmap::IMUEvent>&& events);
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
    // if rebuilding, these events are generated with parseEvents,
    // otherwise, storeEvents should be called by the derived Dataloader class
    const std::vector<rtabmap::IMUEvent>& getEvents() const { return events_; }
private:
    // data must pass validation to perform SLAM
    bool validate(const bool silent = false) const;
    // invoked in constructor
    // error handling is punted until validation
    bool parseEvents();
};

#endif // DATALOADER_H