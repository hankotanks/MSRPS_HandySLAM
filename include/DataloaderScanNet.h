#ifndef DATALOADER_SCANNET_H
#define DATALOADER_SCANNET_H

#include "Dataloader.h"

class DataloaderScanNet : public Dataloader {
public:
    DataloaderScanNet(const Config& cfg) : Dataloader(cfg) { /* STUB */ };
    virtual bool process() override;
};

#endif // DATALOADER_SCANNET_H