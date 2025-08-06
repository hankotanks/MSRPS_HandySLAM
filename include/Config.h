#ifndef CONFIG_H
#define CONFIG_H

#include <filesystem>
#include <optional>

namespace fs = std::filesystem;

class Dataloader;

enum DataSource { STRAY, SCANNET, POST };
enum UpscalingMethod { NAIVE, PROMPTDA };

struct Config {
    bool integrated = false;
    DataSource dataSource;
    UpscalingMethod upscalingMethod = NAIVE;
    // input paths
    fs::path pathData;
    fs::path pathTemp;
    fs::path pathDB;
    fs::path pathCalibration;
    fs::path pathIMU;
    fs::path pathStamps;
    fs::path pathImagesColor;
    fs::path pathImagesDepth;
    // output paths
    fs::path pathOut;
    fs::path pathPoses;
    fs::path pathStampsOut;
    std::optional<fs::path> pathCloud;
    Config(int argc, char* argv[]);
private:
    void validate();
    bool rebuildImagesColor = false;
    bool rebuildImagesDepth = false;
    bool rebuildEvents = false;
    bool rebuildCalibration = false;
private:
    friend Dataloader;
};

#endif // CONFIG_H