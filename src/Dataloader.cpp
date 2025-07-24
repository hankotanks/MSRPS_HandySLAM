#include "Dataloader.h"

#include <fstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/IMU.h>

#include "ext/lazycsv.h"

#include "PyScript.h"

bool Dataloader::init() {
        if(cfg_.post) return true;
        if(!process()) {
            UERROR("Loading scene data failed [%s].", cfg_.pathData.c_str());
            return false;
        }
        if(!cfg_.validate()) {
            UERROR("Data validation failed.");
            return false;
        }
        return true;
    }


cv::Size Dataloader::queryImagesColor(const fs::path& pathImagesColorIn) const {
    UINFO("Querying dimensions of color imagery [%s].", pathImagesColorIn.c_str());

    cv::VideoCapture cap(pathImagesColorIn);
    if(!cap.isOpened()) {
        UWARN("Failed to split RGB imagery from [%s].", pathImagesColorIn.c_str());
        return cv::Size(0, 0);
    }

    cv::Size size;
    size.width =  static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    size.height = static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    cap.release();

    return size;
}

bool Dataloader::splitImagesColor(const fs::path& pathImagesColorIn) const {
    UINFO("Began splitting color imagery [%s].", pathImagesColorIn.c_str());

    cv::VideoCapture cap(pathImagesColorIn);
    if(!cap.isOpened()) {
        UWARN("Failed to split RGB imagery from [%s].", pathImagesColorIn.c_str());
        return false;
    }

    cv::Mat frame;
    for(int frameCount = 0; cap.read(frame); ++frameCount) {
        std::ostringstream filename;
        filename << std::setw(6) << std::setfill('0') << frameCount << ".png";
        fs::path pathFrame = cfg_.pathImagesColor / filename.str();

        cv::Mat frameResized;
        cv::resize(frame, frameResized, cv::Size(HANDY_W, HANDY_H));
        cv::imwrite(pathFrame, frameResized);

        UINFO("Saved color frame [%d] to [%s].", frameCount, pathFrame.c_str());
    }
    
    cap.release();

    UINFO("Completed splitting color imagery.");

    return true;
}

void upscaleDepthNaive(const fs::path& pathImagesDepthIn, const fs::path& pathImagesDepthOut) {
    UINFO("Began depth upscaling WITHOUT PromptDA [%s].", pathImagesDepthIn.c_str());

    std::vector<fs::directory_entry> entries;
    for(const auto& entry : fs::directory_iterator(pathImagesDepthIn)) {
        if(entry.is_regular_file() && entry.path().extension() == ".png") entries.push_back(entry);
    }

    std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
        return a.path().filename() < b.path().filename();
    });

    for(size_t i = 0; i + 1 < entries.size(); ++i) {
        const auto& entry = entries[i];

        cv::Mat frame = cv::imread(entry.path().string(), cv::IMREAD_UNCHANGED);
        if(frame.empty()) UWARN("Failed to process a frame of depth imagery [%s].", entry.path().c_str());
        fs::path pathFrame = pathImagesDepthOut / entry.path().filename().string();

        cv::Mat frameResized;
        cv::resize(frame, frameResized, cv::Size(HANDY_W, HANDY_H));

        cv::imwrite(pathFrame, frameResized);

        UINFO("Saved depth frame [%zu] to [%s].", i, pathFrame.c_str());
    }

    UINFO("Completed depth upscaling WITHOUT PromptDA.");
}

bool upscaleDepthPromptDA(
    const fs::path& pathImagesColor, 
    const fs::path& pathImagesDepthIn, 
    const fs::path& pathImagesDepthOut
) {
    UINFO("Began depth upscaling WITH PromptDA.");
    UINFO("pathRGB: [%s].", pathImagesColor.c_str());
    UINFO("pathDepthIn: [%s].", pathImagesDepthIn.c_str());
    UINFO("pathDepthOut: [%s].", pathImagesDepthOut.c_str());

    const bool result = PyScript::get().call("upscale_depth", "sssi", 
        pathImagesColor.c_str(), pathImagesDepthIn.c_str(), pathImagesDepthOut.c_str(), HANDY_W);

    if(result) UINFO("Completed depth upscaling WITH PromptDA.");
    else {
        UERROR("Failed to upscale depth imagery WITH PromptDA.");
        UINFO("pathRGB: [%s].", pathImagesColor.c_str());
        UINFO("pathDepthIn: [%s].", pathImagesDepthIn.c_str());
        UINFO("pathDepthOut: [%s].", pathImagesDepthOut.c_str());
    }

    return result;
}

bool Dataloader::writeDepth(const fs::path& pathImagesDepthIn) const {
    if(cfg_.upscalingMethod == PROMPTDA) return upscaleDepthPromptDA(
        cfg_.pathImagesColor, 
        pathImagesDepthIn, 
        cfg_.pathImagesDepth);
    else upscaleDepthNaive(pathImagesDepthIn, cfg_.pathImagesDepth);
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

    UINFO("Began writing camera calibration [%s].", cfg_.pathCalibration.c_str());

    std::ofstream file(cfg_.pathCalibration);
    if(!file.is_open()) {
        UERROR("Failed to open camera calibration output file [%s].", cfg_.pathCalibration.c_str());
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

bool Dataloader::writeEvents(std::vector<rtabmap::IMUEvent>&& events) const {
    UINFO("Began storing [%zu] IMU events.", events.size());

    std::ofstream streamIMU(cfg_.pathIMU);
    if(!streamIMU.is_open()) {
        UERROR("Failed to write to IMU output file [%s].", cfg_.pathIMU.c_str());
        return false;
    }
    std::ofstream streamStamps(cfg_.pathStamps);
    if(!streamStamps.is_open()) {
        UERROR("Failed to write to timestamps output file [%s].", cfg_.pathStamps.c_str());
        return false;
    }

    rtabmap::IMU sensorData;
    for(const rtabmap::IMUEvent& event : events) {
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

std::optional<std::vector<rtabmap::IMUEvent>> Dataloader::parseEvents() const {
    std::vector<rtabmap::IMUEvent> events;

    lazycsv::parser<lazycsv::mmap_source,
        lazycsv::has_header<false>,
        lazycsv::delimiter<','>,
        lazycsv::quote_char<'"'>,
        lazycsv::trim_chars<' ', '\t'>> imuParser { cfg_.pathIMU };
    rtabmap::IMU imuCurrent;

    std::ifstream stampsStream(cfg_.pathStamps);
    double stampCurrent;

    char* temp;

    double lx, ly, lz, ax, ay, az, qx, qy, qz, qw;
    for(const auto row : imuParser) {
        if(!(stampsStream >> stampCurrent)) return std::nullopt;

        const auto raw = row.cells(0, 1, 2, 3, 4, 5);
        lx = std::strtof(raw[0].raw().data(), &temp);
        if(temp == raw[0].raw().data()) return std::nullopt;
        ly = std::strtof(raw[1].raw().data(), &temp);
        if(temp == raw[1].raw().data()) return std::nullopt;
        lz = std::strtof(raw[2].raw().data(), &temp);
        if(temp == raw[2].raw().data()) return std::nullopt;
        ax = std::strtof(raw[3].raw().data(), &temp);
        if(temp == raw[3].raw().data()) return std::nullopt;
        ay = std::strtof(raw[4].raw().data(), &temp);
        if(temp == raw[4].raw().data()) return std::nullopt;
        az = std::strtof(raw[5].raw().data(), &temp);
        if(temp == raw[5].raw().data()) return std::nullopt;

        imuCurrent = rtabmap::IMU(
            cv::Vec3d(ax, ay, az),
            cv::Mat::eye(3, 3, CV_64F),
            cv::Vec3d(lx, ly, lz),
            cv::Mat::eye(3, 3, CV_64F)
        );

        events.emplace_back(imuCurrent, stampCurrent); 
    }
    
    return events;
}