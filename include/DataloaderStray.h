#ifndef DATALOADER_STRAY_H
#define DATALOADER_STRAY_H

#include "Config.h"
#include "Dataloader.h"

class DataloaderStray : public Dataloader {
private:
    bool upscaleWithPromptDA_;
public:
    DataloaderStray(const Config& cfg) : Dataloader(
        std::get<0>(cfg.getPaths()), 
        std::get<1>(cfg.getPaths()) / "temp", 
        cfg.forceRebuild()), upscaleWithPromptDA_(cfg.upscaleWithPromptDA()) { /* STUB */ };
    virtual void process() const override;
};

#endif // DATALOADER_STRAY_H