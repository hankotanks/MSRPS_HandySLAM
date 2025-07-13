#ifndef DATALOADER_H
#define DATALOADER_H

#include <filesystem>
#include <vector>

#include <rtabmap/utilite/ULogger.h>

namespace fs = std::filesystem;

class Dataloader {
private:
    bool rebuild_;
    fs::path pathData_, pathTemp_, pathRGB_, pathDepth_;
public:
    Dataloader(
        const fs::path& pathData, 
        const fs::path& pathTemp, 
        const bool forceRebuild = false);
    // data must past validation to perform SLAM
    bool validate(const bool silent = false) const;
    virtual void process() const = 0;
    // initializer (invokes Dataloader::process and can't be overriden)
    virtual void init() final { 
        if(!rebuild_) return;
        UINFO("Began preprocessing scene data.");
        process(); 
        UINFO("Finished preprocessing scene data."); }
    // getters
    fs::path getPathData() const { return pathData_; }
    fs::path getPathIMU() const { return (pathTemp_ / "imu.csv"); }
    fs::path getPathRGB() const { return pathRGB_; }
    fs::path getPathDepth() const { return pathDepth_; }
    std::pair<fs::path, std::string> getPathCalibration() const { 
        return std::make_pair(pathTemp_, "camera"); }
    fs::path getPathCalibrationFile() const {
        auto [pathCalibration, cameraName] = Dataloader::getPathCalibration();
        return pathCalibration / (cameraName + ".yaml"); }
    fs::path getPathStamps() const { return (pathTemp_ / "stamps.txt"); }
};

#endif // DATALOADER_H