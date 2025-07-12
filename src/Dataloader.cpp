#include "Dataloader.h"

#include <algorithm>
#include <vector>
#include <fstream>
#include <filesystem>

#include <rtabmap/utilite/ULogger.h>

namespace fs = std::filesystem;

Dataloader::Dataloader(const fs::path& pathData, const fs::path& pathTemp, const bool forceRebuild) : 
    pathData_(pathData), pathTemp_(pathTemp) {
    pathRGB_ = pathTemp_ / "rgb";
    pathDepth_ = pathTemp_ / "depth";

    rebuild_ = !Dataloader::validate(true) || forceRebuild;
    if(!rebuild_) return;

    if(fs::exists(pathTemp_)) fs::remove_all(pathTemp_);
    fs::create_directories(pathTemp_);
    fs::create_directories(pathRGB_);
    fs::create_directories(pathDepth_);
}

size_t validateImagery(const fs::path& path) {
    if(!fs::exists(path)) return 0;

    std::vector<fs::path> pathImages;

    for(const auto& entry : fs::directory_iterator(path)) {
        if(entry.path().extension() == ".png") pathImages.push_back(entry.path());
    }

    std::sort(pathImages.begin(), pathImages.end());

    size_t imageCount = pathImages.size();
    for(size_t i = 0; i < imageCount; ++i) {
        std::string imageName = pathImages[i].filename();
        std::stringstream imageNameExpected;
        imageNameExpected << std::setw(6) << std::setfill('0') << i << ".png";

        if(imageName != imageNameExpected.str()) return 0;
    }

    return imageCount;
}

size_t validateLineCount(const fs::path& path, const bool hasHeader = false) {
    std::ifstream file(path);  // Open the file
    if(!file.is_open()) {
        UERROR("Failed to open [%s].", path.c_str());
        return 0;
    }

    size_t lineCount = 0, emptyCount = 0;

    std::string line;
    bool lineHadContent = false, lineEmptyAfterContent = false;
    while(std::getline(file, line)) {
        if(std::all_of(line.begin(), line.end(), isspace)) {
            if(lineHadContent) lineEmptyAfterContent = true;
        } else {
            if(lineEmptyAfterContent) return 0;
            lineHadContent = true;
            lineCount++;
        }
    }
    file.close();

    if(lineCount == 0) return 0;

    return hasHeader ? (lineCount - 1) : lineCount;
}

// returns truthy if the data at pathTemp_ is valid
bool Dataloader::validate(const bool silent) const {
    if(!fs::exists(pathTemp_)) {
        if(!silent) UERROR("Skipping validation, temp folder does not exist [%s].", 
            pathTemp_.c_str());
        return false;
    }

    if(!silent) UINFO("Beginning data validation.");
    const size_t imageCountRGB = validateImagery(Dataloader::getPathRGB());
    if(!imageCountRGB) {
        if(!silent) UERROR("Provided RGB imagery is invalid [%s].", 
            Dataloader::getPathRGB().c_str());
        return false;
    }

    const size_t imageCountDepth = validateImagery(Dataloader::getPathDepth());
    if(!imageCountDepth) {
        if(!silent) UERROR("Provided depth imagery is invalid [%s].", 
            Dataloader::getPathDepth().c_str());
        return false;
    }

    if(imageCountRGB != imageCountDepth) {
        if(!silent) UERROR("Number of RGB [%zu] and depth images [%zu] don't match.", 
            imageCountRGB, imageCountDepth);
        return false;
    }

    const size_t stampsCount = validateLineCount(Dataloader::getPathStamps());
    if(stampsCount != imageCountRGB) {
        if(!silent) UERROR("Imagery count [%zu] does not match number of timestamps [%zu] in [%s].", 
            imageCountRGB, stampsCount, Dataloader::getPathStamps().c_str());
        return false;
    }

    if(!fs::exists(Dataloader::getPathCalibrationFile())) {
        if(!silent) UERROR("Camera calibration file does not exist [%s].", 
            Dataloader::getPathCalibrationFile().c_str());
        return false;
    }

    const size_t imuCount = validateLineCount(Dataloader::getPathStamps());
    if(imuCount != imageCountRGB) {
        if(!silent) UERROR("Imagery count [%zu] does not match number of IMU entries [%zu] in [%s].", 
            imageCountRGB, imuCount, Dataloader::getPathIMU().c_str());
        return false;
    }

    if(!silent) UINFO("Finished data validation.");
    return true;
}
