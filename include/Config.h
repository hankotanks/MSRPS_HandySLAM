#ifndef CONFIG_H
#define CONFIG_H

#include <filesystem>

namespace fs = std::filesystem;

enum ConfigDataSource { STRAY, SCANNET };

class Config {
private:
    ConfigDataSource dataSource_;
    fs::path pathData_;
    fs::path pathOut_;
    bool forceRebuild_ = false;
    bool upscaleWithPromptDA_ = false;
    bool savePoints_ = false;
    bool skipSLAM_ = false;
    bool withIMU_ = false;
public:
    Config(int argc, char* argv[]);
    std::pair<fs::path, fs::path> getPaths() const { return std::make_pair(pathData_, pathOut_); }
    bool forceRebuild() const { return forceRebuild_; }
    bool upscaleWithPromptDA() const { return upscaleWithPromptDA_; }
    bool savePoints() const { return savePoints_; }
    bool skipSLAM() const { return skipSLAM_; }
    bool withIMU() const { return withIMU_; }
    ConfigDataSource dataSource() { return dataSource_; }
};

#endif // CONFIG_H