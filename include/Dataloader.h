#ifndef DATALOADER_H
#define DATALOADER_H

#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

class Dataloader {
private:
    bool rebuild_;
    fs::path pathData_, pathTemp_, pathRGB_, pathDepth_, pathCalibration_;
public:
    Dataloader(const fs::path& pathData, const fs::path& pathTemp, const bool forceRebuild = false);
    bool validate() const;
    virtual void process() const = 0;
    // getters
    fs::path getPathRGB() const;
    fs::path getPathDepth() const;
    std::pair<fs::path, fs::path> getPathCalibration() const;
    fs::path getPathStamps() const;
    // write
    void writeStamps(const std::vector<double>& stamps) const;
};

#endif // DATALOADER_H