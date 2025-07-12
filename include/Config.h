#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <filesystem>

namespace fs = std::filesystem;

class Config {
private:
    fs::path pathData_;
    fs::path pathOut_;
    bool forceRebuild_ = false;
    bool upscaleWithPromptDA_ = false;
public:
    Config(int argc, char* argv[]);
    std::pair<fs::path, fs::path> getPaths() const { return std::make_pair(pathData_, pathOut_); }
    bool forceRebuild() const { return forceRebuild_; }
    bool upscaleWithPromptDA() const { return upscaleWithPromptDA_; }
};

#endif // CONFIG_H