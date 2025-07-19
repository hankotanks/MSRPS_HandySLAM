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

#if 0
class Config {
private:
    ConfigDataSource dataSource_;
    fs::path pathData_;
    fs::path pathOut_;
    bool forceRebuild_ = false;
    bool upscaleWithPromptDA_ = false;
    bool skipSLAM_ = false;
    bool withIMU_ = false;
    std::optional<fs::path> pathCloud_;
public:
    Config(int argc, char* argv[]);
    std::pair<fs::path, fs::path> getPaths() const { return std::make_pair(pathData_, pathOut_); }
    bool forceRebuild() const { return forceRebuild_; }
    bool upscaleWithPromptDA() const { return upscaleWithPromptDA_; }
    bool skipSLAM() const { return skipSLAM_; }
    bool withIMU() const { return withIMU_; }
    ConfigDataSource dataSource() { return dataSource_; }
};
#endif

#endif // CONFIG_H