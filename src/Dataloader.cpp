#include "Dataloader.h"

#include <algorithm>
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rtabmap/utilite/ULogger.h>

#include "Config.h"
#include "PyScript.h"
#include "rtabmap/core/IMU.h"

namespace fs = std::filesystem;

Dataloader::Dataloader(const Config& cfg) : 
    pathData_(std::get<0>(cfg.getPaths())),
    pathTemp_(std::get<1>(cfg.getPaths()) / "temp"),
    pathColor_(std::get<1>(cfg.getPaths()) / "temp" / "rgb"),
    pathDepth_(std::get<1>(cfg.getPaths()) / "temp" / "depth"),
    upscale_(cfg.upscaleWithPromptDA()),
    rebuild_(cfg.forceRebuild()) {
    rebuild_ |= !Dataloader::validate(true); 
      
    if(fs::exists(Dataloader::getPathDB())) fs::remove(Dataloader::getPathDB());
    if(!rebuild_) return;
    if(fs::exists(pathTemp_)) fs::remove_all(pathTemp_);

    fs::create_directories(pathTemp_);
    fs::create_directories(pathColor_);
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
    const size_t imageCountRGB = validateImagery(Dataloader::getPathColor());
    if(!imageCountRGB) {
        if(!silent) UERROR("Provided RGB imagery is invalid [%s].", 
            Dataloader::getPathColor().c_str());
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

cv::Size Dataloader::splitColorVideoAndScale(const fs::path& pathColorIn) const {
    cv::VideoCapture cap(pathColorIn);
    if(!cap.isOpened()) {
        UWARN("Failed to split RGB imagery from [%s].", pathColorIn.c_str());
        return cv::Size(0, 0);
    }

    cv::Size capSize;
    capSize.width =  static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    capSize.height = static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    cv::Mat frame;
    for(int frameCount = 0; cap.read(frame); ++frameCount) {
        std::ostringstream filename;
        filename << std::setw(6) << std::setfill('0') << frameCount << ".png";

        cv::Mat frameResized;
        cv::resize(frame, frameResized, cv::Size(HANDY_W, HANDY_H));
        cv::imwrite(Dataloader::getPathColor() / filename.str(), frameResized);
    }

    return capSize;
}

void upscaleDepthNaive(const fs::path& pathDepthIn, const fs::path& pathDepthOut) {
    std::vector<fs::directory_entry> entries;
    for(const auto& entry : fs::directory_iterator(pathDepthIn)) {
        if(entry.is_regular_file() && entry.path().extension() == ".png") entries.push_back(entry);
    }

    std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
        return a.path().filename() < b.path().filename();
    });

    for(size_t i = 0; i + 1 < entries.size(); ++i) {
        const auto& entry = entries[i];

        cv::Mat frame = cv::imread(entry.path().string(), cv::IMREAD_UNCHANGED);
        if(frame.empty()) UWARN("Failed to process a frame of depth imagery [%s].", entry.path().c_str());

        cv::Mat frameResized;
        cv::resize(frame, frameResized, cv::Size(HANDY_W, HANDY_H));

        cv::imwrite(pathDepthOut / entry.path().filename().string(), frameResized);
    }
}

void upscaleDepthPromptDA(
    const fs::path& pathRGB, 
    const fs::path& pathDepthIn, 
    const fs::path& pathDepthOut
) {
    PyScript("upscale_depth_imagery").call("main", "sssi", 
        pathRGB.c_str(), 
        pathDepthIn.c_str(), 
        pathDepthOut.c_str(), HANDY_W);
}

void Dataloader::upscaleDepth(const fs::path& pathDepthIn) const {
    if(upscale_) upscaleDepthPromptDA(
        Dataloader::getPathColor(), 
        pathDepthIn, 
        Dataloader::getPathDepth());
    else upscaleDepthNaive(pathDepthIn, Dataloader::getPathDepth());
}

void Dataloader::writeCalibration(
    const rtabmap::Transform& intrinsics, 
    const cv::Size& originalColorSize
) const {
    double width_scalar = \
        static_cast<double>(HANDY_W) / static_cast<double>(originalColorSize.width);
    double height_scalar = \
        static_cast<double>(HANDY_H) / static_cast<double>(originalColorSize.height);

    double fx, fy, cx, cy;
    fx = intrinsics(0, 0) * width_scalar;
    fy = intrinsics(1, 1) * height_scalar;
    cx = intrinsics(0, 2) * width_scalar;
    cy = intrinsics(1, 2) * height_scalar;

    std::ofstream file(Dataloader::getPathCalibrationFile());
    file << "%YAML:1.0" << std::endl;
    file << "---" << std::endl;
    file << "camera_name: handy_camera" << std::endl;
    file << "image_width: " << HANDY_W << std::endl;
    file << "image_height: " << HANDY_H << std::endl;
    file << "camera_matrix:" << std::endl;
    file << "   rows: 3" << std::endl;
    file << "   cols: 3" << std::endl;
    file << "   data: [ ";
    file << fx  << ", " << 0.0 << ", " << cx  << ", ";
    file << 0.0 << ", " << fy  << ", " << cy  << ", ";
    file << 0.0 << ", " << 0.0 << ", " << 0.0 << " ]" << std::endl;
    file << "local_transform:" << std::endl;
    file << "   rows: 3" << std::endl;
    file << "   cols: 4" << std::endl;
    file << "   data: [  1.0,  0.0,  0.0, 0.0, " << std::endl;
    file << "            0.0,  1.0,  0.0, 0.0, " << std::endl;
    file << "            0.0,  0.0,  1.0, 0.0 ]" << std::endl;
    file << "distortion_coefficients:" << std::endl;
    file << "   rows: 1" << std::endl;
    file << "   cols: 5" << std::endl;
    file << "   data: [ 0., 0., 0., 0., 0. ]" << std::endl;
    file << "distortion_model: plumb_bob" << std::endl;
    file << "rectification_matrix:" << std::endl;
    file << "   rows: 3" << std::endl;
    file << "   cols: 3" << std::endl;
    file << "   data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1. ]" << std::endl;
    file << "projection_matrix:" << std::endl;
    file << "   rows: 3" << std::endl;
    file << "   cols: 4" << std::endl;
    file << "   data: [ ";
    file << fx  << ", " << 0.0 << ", " << cx  << ", 0.0, ";
    file << 0.0 << ", " << fy  << ", " << cy  << ", 0.0, ";
    file << 0.0 << ", " << 0.0 << ", " << 0.0 << ", 0.0 ]" << std::endl;
    file.close();
}

void Dataloader::storeEvents(std::vector<rtabmap::IMUEvent>&& events) {
    events_ = std::move(events);
    std::ofstream streamIMU(Dataloader::getPathIMU());
    std::ofstream streamStamps(Dataloader::getPathStamps());

    rtabmap::IMU sensorData;
    for(const rtabmap::IMUEvent& event : events_) {
        sensorData = event.getData();

        streamIMU << std::fixed << std::setprecision(6) << sensorData.linearAcceleration()[0] << ", ";
        streamIMU << std::fixed << std::setprecision(6) << sensorData.linearAcceleration()[1] << ", ";
        streamIMU << std::fixed << std::setprecision(6) << sensorData.linearAcceleration()[2] << ", ";
        streamIMU << std::fixed << std::setprecision(6) << sensorData.angularVelocity()[0] << ", ";
        streamIMU << std::fixed << std::setprecision(6) << sensorData.angularVelocity()[1] << ", ";
        streamIMU << std::fixed << std::setprecision(6) << sensorData.angularVelocity()[2] << std::endl;

        streamStamps << std::fixed << std::setprecision(6) << event.getStamp() << std::endl;
    }

    streamIMU.close();
    streamStamps.close();
}