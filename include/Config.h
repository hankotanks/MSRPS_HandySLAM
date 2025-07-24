#ifndef CONFIG_H
#define CONFIG_H

#include <filesystem>
#include <optional>

namespace fs = std::filesystem;

enum DataSource { STRAY, SCANNET };
enum UpscalingMethod { NAIVE, PROMPTDA };

struct Config {
    bool rebuild = false;
    bool post = false;
    bool integrated = false;
    DataSource dataSource;
    UpscalingMethod upscalingMethod = NAIVE;
    fs::path pathData;
    fs::path pathOut;
    fs::path pathTemp;
    fs::path pathDB;
    fs::path pathCalibration;
    fs::path pathIMU;
    fs::path pathStamps;
    fs::path pathImagesColor;
    fs::path pathImagesDepth;
    std::optional<fs::path> pathCloud;

    Config(int argc, char* argv[]);

    bool validate() const;
};

#endif // CONFIG_H