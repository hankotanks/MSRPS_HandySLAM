#ifndef DATALOADER_SCANNET_H
#define DATALOADER_SCANNET_H

#include "Dataloader.h"

class DataloaderScanNet : public Dataloader {
public:
    DataloaderScanNet(const Config& cfg) : Dataloader(cfg) { /* STUB */ };
    virtual bool processImagesColor() override;
    virtual bool processImagesDepth() override;
    virtual bool processEvents() override;
    virtual bool processCalibration() override;
    // helper functions
private:
    std::optional<std::pair<rtabmap::Transform, std::vector<rtabmap::IMUEvent>>> processEventsAndCalibration();
};

#endif // DATALOADER_SCANNET_H