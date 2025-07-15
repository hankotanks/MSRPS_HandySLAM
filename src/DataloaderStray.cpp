#include "DataloaderStray.h"

#include <filesystem>
#include <optional>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rtabmap/core/IMU.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/ULogger.h>

#include "ext/lazycsv.h"

namespace fs = std::filesystem;

std::optional<rtabmap::Transform> parseCameraMatrix(
    const fs::path& pathCameraMatrix
) {
    lazycsv::parser<lazycsv::mmap_source,   
        lazycsv::has_header<false>,
        lazycsv::delimiter<','>,
        lazycsv::quote_char<'"'>,
        lazycsv::trim_chars<' ', '\t'>> parser { pathCameraMatrix };

    int rowCount = 0;
    char* temp;

    rtabmap::Transform intrinsics{};
    for(const auto row : parser) {
        const auto [fst, snd, thd] = row.cells(0, 1, 2);
        intrinsics(rowCount, 0) = std::strtof(fst.raw().data(), &temp);
        if(temp == fst.raw().data()) return std::nullopt;
        intrinsics(rowCount, 1) = std::strtof(snd.raw().data(), &temp);
        if(temp == snd.raw().data()) return std::nullopt;
        intrinsics(rowCount, 2) = std::strtof(thd.raw().data(), &temp); 
        if(temp == thd.raw().data()) return std::nullopt;
        rowCount++;
    }

    return intrinsics;
}

std::vector<rtabmap::IMUEvent> parseStrayEvents(
    const fs::path& pathOdometry, 
    const fs::path& pathIMU
) {
    lazycsv::parser<> parserStamps { pathOdometry };

    std::vector<float> stamps;

    char* temp;
    for(const auto row : parserStamps) {
        const auto [t] = row.cells(0);
        stamps.emplace_back(std::strtof(t.raw().data(), &temp));
    }
    stamps.pop_back();

    lazycsv::parser<> parserIMU { pathIMU };
    std::vector<rtabmap::IMU> sensorData;

    size_t stampIdx = 0;
    float t0 = std::numeric_limits<float>::max() * -1.f, tf;
    for(const auto row : parserStamps) {
        if(stampIdx >= stamps.size()) break;
        const auto raw = row.cells(0, 1, 2, 3, 4, 5, 6);
        tf = std::strtof(raw[0].raw().data(), &temp);
        if(stamps[stampIdx] < tf) {
            sensorData.emplace_back(
                cv::Vec3d(
                    std::strtof(raw[4].raw().data(), &temp), 
                    std::strtof(raw[5].raw().data(), &temp), 
                    std::strtof(raw[6].raw().data(), &temp)),
                cv::Mat::eye(3, 3, CV_64F),
                cv::Vec3d(
                    std::strtof(raw[1].raw().data(), &temp), 
                    std::strtof(raw[2].raw().data(), &temp), 
                    std::strtof(raw[3].raw().data(), &temp)),
                cv::Mat::eye(3, 3, CV_64F)
            ); stampIdx++;
        } t0 = tf;
    }

    std::vector<rtabmap::IMUEvent> events;

    size_t eventCount = std::min(stamps.size(), sensorData.size());
    events.reserve(eventCount);

    for(size_t i = 0; i < eventCount; ++i) events.emplace_back(sensorData[i], stamps[i]);

    return events;
}

bool DataloaderStray::process() {
    // RGB
    const fs::path pathColorIn = Dataloader::getPathData() / "rgb.mp4";
    const cv::Size originalColorSize = Dataloader::splitColorVideoAndScale(pathColorIn);
    if(originalColorSize.empty()) {
        UERROR("Failed to split RGB frames [%s].", pathColorIn.c_str());
        return false;
    }
    // DEPTH
    const fs::path pathDepthIn = Dataloader::getPathData() / "depth";
    if(!Dataloader::upscaleDepth(pathDepthIn)) {
        UERROR("Failed to upscale depth [%s].", pathDepthIn.c_str());
        return false;
    }

    // CALIBRATION
    const fs::path pathCameraMatrix = Dataloader::getPathData() / "camera_matrix.csv";
    auto intrinsics = parseCameraMatrix(pathCameraMatrix);
    if(!intrinsics) {
        UERROR("Failed to parse camera matrix [%s].", pathCameraMatrix.c_str());
        return false;
    }
    Dataloader::writeCalibration(*intrinsics, originalColorSize);

    // TIMESTAMPS AND IMU
    const fs::path pathOdometry = Dataloader::getPathData() / "odometry.csv";
    const fs::path pathIMU = Dataloader::getPathData() / "imu.csv";
    std::vector<rtabmap::IMUEvent> events = parseStrayEvents(pathOdometry, pathIMU);
    if(events.empty()) {
        UERROR("Failed to parse odometry [%s] or IMU data [%s].", pathOdometry.c_str(), pathIMU.c_str());
        return false;
    }
    Dataloader::storeEvents(std::move(events));

    return true;
}
