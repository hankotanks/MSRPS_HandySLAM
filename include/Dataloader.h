#ifndef DATALOADER_H
#define DATALOADER_H

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/Transform.h>

#include "Config.h"

class Dataloader {
protected:
    const Config cfg_;
public: // IMPLEMENT THIS METHOD ON DERIVED CLASSES
    virtual bool process() = 0;
public:
    Dataloader(const Config& cfg) : cfg_(cfg) { /* STUB */ }
    bool init();
public:
    const Config& getConfig() const { return cfg_; }
public: // helper methods for derived dataloaders
    bool writeCalibration(const rtabmap::Transform& intrinsics, const cv::Size& originalColorSize) const;
    bool writeDepth(const fs::path& pathDepthIn) const;
    bool writeEvents(std::vector<rtabmap::IMUEvent>&& events) const;
    std::optional<std::vector<rtabmap::IMUEvent>> parseEvents() const;
    bool splitImagesColor(const fs::path& pathImagesColorIn) const;
    cv::Size queryImagesColor(const fs::path& pathImagesColorIn) const;
};

#endif // DATALOADER_H