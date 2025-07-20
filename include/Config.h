#ifndef CONFIG_H
#define CONFIG_H

#include <filesystem>
#include <optional>

namespace fs = std::filesystem;

enum ConfigDataSource { STRAY, SCANNET };
enum ConfigUpscalingMethod { NAIVE, PROMPTDA };

struct Config {
    ConfigDataSource dataSource;
    fs::path pathData;
    fs::path pathOut;
    std::optional<fs::path> pathCloud;
    ConfigUpscalingMethod upscalingMethod = NAIVE;
    bool skipSLAM = false;
    bool withIMU= false;
    bool forceRebuild = false;
    Config(int argc, char* argv[]);
};

#endif // CONFIG_H