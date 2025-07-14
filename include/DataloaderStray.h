#ifndef DATALOADER_STRAY_H
#define DATALOADER_STRAY_H

#include "Config.h"
#include "Dataloader.h"

class DataloaderStray : public Dataloader {
private:
    bool upscaleWithPromptDA_;
public:
    DataloaderStray(const Config& cfg) : 
        Dataloader(cfg), 
        upscaleWithPromptDA_(cfg.upscaleWithPromptDA()) { /* STUB */ };
    virtual void process() override;
};

#endif // DATALOADER_STRAY_H