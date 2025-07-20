#include "Dataloader.h"

#include <algorithm>
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rtabmap/core/IMU.h>
#include <rtabmap/utilite/ULogger.h>

#include "ext/lazycsv.h"

#include "Config.h"
#include "PyScript.h"

namespace fs = std::filesystem;

Dataloader::Dataloader(const Config& cfg) : 
    pathData_(cfg.pathData),
    pathTemp_(cfg.pathData / "temp"),
    pathDB_(cfg.pathOut / "temp.db"),
    pathColor_(cfg.pathData / "temp" / "rgb"),
    pathDepth_(cfg.pathData / "temp" / "depth"),
    upscale_(cfg.upscalingMethod == PROMPTDA),
    rebuild_(cfg.forceRebuild) {
    if(cfg.skipSLAM) {
        if(fs::exists(Dataloader::getPathDB())) {
            UINFO("Skipping SLAM, beginning postprocessing.");
            skip_ = true;
            return;
        } else UWARN("Can't skip SLAM without prebuilt database [%s].", pathDB_.c_str());
    }

    if(rebuild_ || !fs::exists(pathTemp_)) {
        if(fs::exists(pathTemp_)) fs::remove_all(pathTemp_);
        fs::create_directories(pathTemp_);
        fs::create_directories(pathColor_);
        fs::create_directories(pathDepth_);
    }

    validation_ = Dataloader::validate(true);

      
    if(fs::exists(pathDB_)) fs::remove(pathDB_);

    rebuild_ |= !(validation_.valid());
    if(!rebuild_) {
        if(!Dataloader::parseEvents()) invalid_ = true;
        return;
    }
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
    if(!fs::exists(path)) return 0;

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
DataloaderValidation Dataloader::validate(const bool silent) const {
    DataloaderValidation validation;

    if(!fs::exists(pathTemp_)) {
        if(!silent) UERROR("Skipping validation, temp folder does not exist [%s].", 
            pathTemp_.c_str());
        validation.fullRebuild = true;
        return validation;
    }

    if(invalid_) {
        if(!silent) UERROR("Failed to parse IMU data [%s] and timestamps [%s].",
            Dataloader::getPathIMU().c_str(), Dataloader::getPathStamps().c_str());
        validation.events = true;
    }

    if(!silent) UINFO("Beginning data validation.");

    validation.colorFrameCount = validateImagery(Dataloader::getPathColor());
    if(!validation.colorFrameCount) {
        if(!silent) UERROR("Provided RGB imagery is invalid [%s].", 
            Dataloader::getPathColor().c_str());
        validation.colorFrames = true;
    }

    validation.depthFrameCount = validateImagery(Dataloader::getPathDepth());
    if(!validation.depthFrameCount) {
        if(!silent) UERROR("Provided depth imagery is invalid [%s].", 
            Dataloader::getPathDepth().c_str());
        validation.depthFrames = true;
    }

    size_t frameCount = std::min(validation.colorFrameCount, validation.depthFrameCount);
    if(validation.colorFrameCount != validation.depthFrameCount) {
        if(!silent) UWARN("Number of RGB [%zu] and depth images [%zu] don't match.", 
            validation.colorFrameCount, validation.depthFrameCount);
    }

    validation.stampCount = validateLineCount(Dataloader::getPathStamps());

    if(validation.stampCount < frameCount) {
        if(!silent) UERROR("Imagery count [%zu color, %zu depth] is greater than the number of timestamps [%zu] in [%s].", 
            validation.colorFrameCount, validation.depthFrameCount, validation.stampCount, Dataloader::getPathStamps().c_str());
        validation.events = true;
    }

    if(!fs::exists(Dataloader::getPathCalibrationFile())) {
        if(!silent) UERROR("Camera calibration file does not exist [%s].", 
            Dataloader::getPathCalibrationFile().c_str());
        validation.calibration = true;
    }

    validation.sensorDataCount = validateLineCount(Dataloader::getPathIMU());
    if(validation.sensorDataCount < frameCount) {
        if(!silent) UERROR("Imagery count [%zu] is greater than the number of IMU entries [%zu] in [%s].", 
            validation.colorFrameCount, validation.sensorDataCount, Dataloader::getPathIMU().c_str());
        validation.events = true;
    }

    if(!silent) UINFO("Finished data validation.");

    return validation;
}

cv::Size Dataloader::queryColorVideoSize(const fs::path& pathColorIn) const {
    UINFO("Querying dimensions of color imagery [%s].", pathColorIn.c_str());

    cv::VideoCapture cap(pathColorIn);
    if(!cap.isOpened()) {
        UWARN("Failed to split RGB imagery from [%s].", pathColorIn.c_str());
        return cv::Size(0, 0);
    }

    cv::Size size;
    size.width =  static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    size.height = static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    cap.release();

    return size;
}

cv::Size Dataloader::splitColorVideoAndScale(const fs::path& pathColorIn) const {
    UINFO("Began splitting color imagery [%s].", pathColorIn.c_str());

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
        fs::path pathFrame = Dataloader::getPathColor() / filename.str();

        cv::Mat frameResized;
        cv::resize(frame, frameResized, cv::Size(HANDY_W, HANDY_H));
        cv::imwrite(pathFrame, frameResized);

        UINFO("Saved color frame [%d] to [%s].", frameCount, pathFrame.c_str());
    }
    
    cap.release();

    UINFO("Completed splitting color imagery.");

    return capSize;
}

void upscaleDepthNaive(const fs::path& pathDepthIn, const fs::path& pathDepthOut) {
    UINFO("Began depth upscaling WITHOUT PromptDA [%s].", pathDepthIn.c_str());

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
        fs::path pathFrame = pathDepthOut / entry.path().filename().string();

        cv::Mat frameResized;
        cv::resize(frame, frameResized, cv::Size(HANDY_W, HANDY_H));

        cv::imwrite(pathFrame, frameResized);

        UINFO("Saved depth frame [%zu] to [%s].", i, pathFrame.c_str());
    }

    UINFO("Completed depth upscaling WITHOUT PromptDA.");
}

bool upscaleDepthPromptDA(
    const fs::path& pathRGB, 
    const fs::path& pathDepthIn, 
    const fs::path& pathDepthOut
) {
    UINFO("Began depth upscaling WITH PromptDA.");
    UINFO("pathRGB: [%s].", pathRGB.c_str());
    UINFO("pathDepthIn: [%s].", pathDepthIn.c_str());
    UINFO("pathDepthOut: [%s].", pathDepthOut.c_str());

    const bool result = PyScript::get().call("upscale_depth", "sssi", 
        pathRGB.c_str(), pathDepthIn.c_str(), pathDepthOut.c_str(), HANDY_W);

    if(result) UINFO("Completed depth upscaling WITH PromptDA.");
    else {
        UERROR("Failed to upscale depth imagery WITH PromptDA.");
        UINFO("pathRGB: [%s].", pathRGB.c_str());
        UINFO("pathDepthIn: [%s].", pathDepthIn.c_str());
        UINFO("pathDepthOut: [%s].", pathDepthOut.c_str());
    }

    return result;
}

bool Dataloader::upscaleDepth(const fs::path& pathDepthIn) const {
    if(upscale_) return upscaleDepthPromptDA(
        Dataloader::getPathColor(), 
        pathDepthIn, 
        Dataloader::getPathDepth());
    else upscaleDepthNaive(pathDepthIn, Dataloader::getPathDepth());
    return true;
}

bool Dataloader::writeCalibration(
    const rtabmap::Transform& intrinsics, 
    const cv::Size& originalColorSize
) const {
    double fx, fy, cx, cy;
    fx = intrinsics(0, 0);
    fy = intrinsics(1, 1);
    cx = intrinsics(0, 2);
    cy = intrinsics(1, 2);

    UINFO("Received camera intrinsics for imagery with size [%dx%d].", originalColorSize.width, originalColorSize.height);
    UINFO("fx: [%lf].", fx);
    UINFO("fy: [%lf].", fy);
    UINFO("cx: [%lf].", cx);
    UINFO("cy: [%lf].", cy);

    double width_scalar = \
        static_cast<double>(HANDY_W) / static_cast<double>(originalColorSize.width);
    double height_scalar = \
        static_cast<double>(HANDY_H) / static_cast<double>(originalColorSize.height);

    fx *= width_scalar;
    fy *= height_scalar;
    cx *= width_scalar;
    cy *= height_scalar;

    UINFO("Rescaled camera intrinsics to [%dx%d].", HANDY_W, HANDY_H);
    UINFO("fx: [%lf].", fx);
    UINFO("fy: [%lf].", fy);
    UINFO("cx: [%lf].", cx);
    UINFO("cy: [%lf].", cy);

    UINFO("Began writing camera calibration [%s].", Dataloader::getPathCalibrationFile().c_str());

    std::ofstream file(Dataloader::getPathCalibrationFile());
    if(!file.is_open()) {
        UERROR("Failed to open camera calibration output file [%s].", Dataloader::getPathCalibrationFile().c_str());
        return false;
    }
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
#if 0
    file << "local_transform:" << std::endl;
    file << "   rows: 3" << std::endl;
    file << "   cols: 4" << std::endl;
    file << "   data: [  1.0,  0.0,  0.0, 0.0, " << std::endl;
    file << "            0.0,  1.0,  0.0, 0.0, " << std::endl;
    file << "            0.0,  0.0,  1.0, 0.0 ]" << std::endl;
#endif
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

    UINFO("Finished writing camera calibration.");

    return true;
}

bool Dataloader::storeEvents(std::vector<rtabmap::IMUEvent>&& events) {
    UINFO("Began storing [%zu] IMU events.", events.size());

    events_ = std::move(events);
    std::ofstream streamIMU(Dataloader::getPathIMU());
    if(!streamIMU.is_open()) {
        UERROR("Failed to write to IMU output file [%s].", Dataloader::getPathIMU().c_str());
        return false;
    }
    std::ofstream streamStamps(Dataloader::getPathStamps());
    if(!streamStamps.is_open()) {
        UERROR("Failed to write to timestamps output file [%s].", Dataloader::getPathStamps().c_str());
        return false;
    }

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

    UINFO("Finished writing IMU and timestamp data to temp folder.");

    return true;
}

bool Dataloader::parseEvents() {
    lazycsv::parser<lazycsv::mmap_source,
        lazycsv::has_header<false>,
        lazycsv::delimiter<','>,
        lazycsv::quote_char<'"'>,
        lazycsv::trim_chars<' ', '\t'>> imuParser { Dataloader::getPathIMU() };
    rtabmap::IMU imuCurrent;

    std::ifstream stampsStream(Dataloader::getPathStamps());
    double stampCurrent;

    char* temp;

    double lx, ly, lz, ax, ay, az, qx, qy, qz, qw;
    for(const auto row : imuParser) {
        if(!(stampsStream >> stampCurrent)) return false;

        const auto raw = row.cells(0, 1, 2, 3, 4, 5);
        lx = std::strtof(raw[0].raw().data(), &temp);
        if(temp == raw[0].raw().data()) return false;
        ly = std::strtof(raw[1].raw().data(), &temp);
        if(temp == raw[1].raw().data()) return false;
        lz = std::strtof(raw[2].raw().data(), &temp);
        if(temp == raw[2].raw().data()) return false;
        ax = std::strtof(raw[3].raw().data(), &temp);
        if(temp == raw[3].raw().data()) return false;
        ay = std::strtof(raw[4].raw().data(), &temp);
        if(temp == raw[4].raw().data()) return false;
        az = std::strtof(raw[5].raw().data(), &temp);
        if(temp == raw[5].raw().data()) return false;

        imuCurrent = rtabmap::IMU(
            cv::Vec3d(ax, ay, az),
            cv::Mat::eye(3, 3, CV_64F),
            cv::Vec3d(lx, ly, lz),
            cv::Mat::eye(3, 3, CV_64F)
        );

        events_.emplace_back(imuCurrent, stampCurrent); 
    }
    
    return true;
}