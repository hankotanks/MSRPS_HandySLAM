#include "Dataloader.h"

#include <algorithm>
#include <vector>
#include <fstream>

#include <rtabmap/utilite/ULogger.h>

Dataloader::Dataloader(const fs::path& pathData, const fs::path& pathTemp, const bool forceRebuild) : pathData_(pathData), pathTemp_(pathTemp) {
    pathRGB_ = pathTemp_ / "rgb";
    pathDepth_ = pathDepth_ / "depth";
    pathCalibration_ = pathTemp_;

    rebuild_ = !Dataloader::validate() || forceRebuild;
    if(!rebuild_) return;

    if(fs::exists(pathTemp_)) fs::remove_all(pathTemp_);
    fs::create_directories(pathTemp_);
    fs::create_directories(pathRGB_);
    fs::create_directories(pathDepth_);
}

size_t validateImagery(const fs::path& path) {
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

    size_t lineCount = 0;

    std::string line;
    while(std::getline(file, line)) ++lineCount;
    file.close();
    if(lineCount == 0) return 0;

    return hasHeader ? lineCount - 1 : lineCount;
}

// returns truthy if the data at pathTemp_ is valid
bool Dataloader::validate() const {
    UINFO("Beginning data validation.");
    size_t imageCountRGB = validateImagery(pathRGB_);
    if(!imageCountRGB) {
        UERROR("Provided RGB imagery is invalid [%s].", pathRGB_.c_str());
        return false;
    }

    size_t imageCountDepth = validateImagery(pathDepth_);
    if(!imageCountDepth) {
        UERROR("Provided depth imagery is invalid [%s].", pathDepth_.c_str());
        return false;
    }

    if(imageCountRGB != imageCountDepth) {
        UERROR("Number of RGB [%zu] and depth images [%zu] don't match.", imageCountRGB, imageCountDepth);
        return false;
    }

    UINFO("Finished data validation.");
    return true;
}

fs::path Dataloader::getPathRGB() const {
    return pathRGB_;
}

fs::path Dataloader::getPathDepth() const {
    return pathDepth_;
}

std::pair<fs::path, fs::path> Dataloader::getPathCalibration() const {
    return std::make_pair(pathCalibration_, "camera.yaml");
}

fs::path Dataloader::getPathStamps() const {
    return pathTemp_ / "stamps.txt";
}
