#ifndef DATALOADER_STRAY_H
#define DATALOADER_STRAY_H

#include "Config.h"
#include "Dataloader.h"

class DataloaderStray : public Dataloader {
public:
    DataloaderStray(const Config& cfg) : Dataloader(cfg) { /* STUB */ };
    virtual bool process() override;
};

#endif // DATALOADER_STRAY_H