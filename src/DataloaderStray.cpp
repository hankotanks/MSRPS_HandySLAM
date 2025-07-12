#include "DataloaderStray.h"

#include <filesystem>

#include <fstream>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rtabmap/utilite/ULogger.h>

#include "ext/lazycsv.h"

#include "PyScript.h"
#include "rtabmap/core/IMU.h"

namespace fs = std::filesystem;

std::pair<size_t, size_t> splitImagesRGB(
    const fs::path& pathIn, 
    const fs::path& pathOut
) {
    cv::VideoCapture cap(pathIn);
    if(!cap.isOpened()) {
        UWARN("Failed to split RGB imagery from [%s].", pathIn.c_str());
        return std::make_pair(0, 0);
    }

    size_t cap_w, cap_h;
    cap_w = static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    cap_h = static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    cv::Mat frame;
    for(int frameCount = 0; cap.read(frame); ++frameCount) {
        std::ostringstream filename;
        filename << std::setw(6) << std::setfill('0') << frameCount << ".png";

        cv::Mat frameResized;
        cv::resize(frame, frameResized, cv::Size(HANDY_W, HANDY_H));
        cv::imwrite(pathOut / filename.str(), frameResized);
    }

    return std::make_pair(cap_w, cap_h);
}

void upscaleImagesDepth(
    const fs::path& pathDepthIn, 
    const fs::path& pathDepthOut
) {
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

void upscaleImagesDepthPromptDA(
    const fs::path& pathRGB, 
    const fs::path& pathDepthIn, 
    const fs::path& pathDepthOut
) {
    PyScript("upscale_depth_imagery").call("main", "sssi", 
        pathRGB.c_str(), 
        pathDepthIn.c_str(), 
        pathDepthOut.c_str(), HANDY_W);
}

std::vector<float> writeTimestamps(
    const fs::path& pathOdometry, 
    const fs::path& pathStamps
) {
    lazycsv::parser<> parser { pathOdometry };

    std::vector<float> stamps;

    char* temp;
    for(const auto row : parser) {
        const auto [t] = row.cells(0);
        stamps.emplace_back(std::strtof(t.raw().data(), &temp));
    }
    stamps.pop_back();

    std::ofstream file(pathStamps);
    for(const float stamp : stamps) 
        file << std::fixed << std::setprecision(6) << stamp << std::endl;
    file.close();

    return stamps;
}

void writeCalibration(
    const fs::path& pathCameraMatrix, 
    const fs::path& pathCalibration,
    const size_t width,
    const size_t height
) {
    lazycsv::parser<lazycsv::mmap_source,
        lazycsv::has_header<false>,
        lazycsv::delimiter<','>,
        lazycsv::quote_char<'"'>,
        lazycsv::trim_chars<' ', '\t'>> parser { pathCameraMatrix };
    float mat[3][3];
    int rowCount = 0;
    char* temp;
    for(const auto row : parser) {
        const auto [fst, snd, thd] = row.cells(0, 1, 2);
        mat[rowCount][0] = std::strtof(fst.raw().data(), &temp);
        mat[rowCount][1] = std::strtof(snd.raw().data(), &temp);
        mat[rowCount][2] = std::strtof(thd.raw().data(), &temp); rowCount++;
    }

    float width_scalar = \
        static_cast<float>(HANDY_W) / static_cast<float>(width);
    float height_scalar = \
        static_cast<float>(HANDY_H) / static_cast<float>(height);

    mat[0][0] *= width_scalar;
    mat[0][2] *= width_scalar;
    mat[1][1] *= height_scalar;
    mat[1][2] *= height_scalar;

    std::ofstream file(pathCalibration);
    file << "%YAML:1.0" << std::endl;
    file << "---" << std::endl;
    file << "camera_name: iphone16pro" << std::endl;
    file << "image_width: " << HANDY_W << std::endl;
    file << "image_height: " << HANDY_H << std::endl;
    file << "camera_matrix:" << std::endl;
    file << "   rows: 3" << std::endl;
    file << "   cols: 3" << std::endl;
    file << "   data: [ ";
    file << mat[0][0] << ", " << mat[0][1] << ", " << mat[0][2] << ", ";
    file << mat[1][0] << ", " << mat[1][1] << ", " << mat[1][2] << ", ";
    file << mat[2][0] << ", " << mat[2][1] << ", " << mat[2][2] << " ]" << std::endl;
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
    file << mat[0][0] << ", " << mat[0][1] << ", " << mat[0][2] << ", 0.0, ";
    file << mat[1][0] << ", " << mat[1][1] << ", " << mat[1][2] << ", 0.0, ";
    file << mat[2][0] << ", " << mat[2][1] << ", " << mat[2][2] << ", 0.0 ]" << std::endl;
    file.close();
}

void writeIMU(
    const fs::path& pathIMUIn,
    const fs::path& pathIMUOut,
    const std::vector<float>& stamps
) {
    lazycsv::parser<> parser { pathIMUIn };
    std::vector<rtabmap::IMU> sensorData;

    size_t stampIdx = 0;
    float t0 = std::numeric_limits<float>::max() * -1.f, tf;
    char* temp;
    for(const auto row : parser) {
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

    std::ofstream file(pathIMUOut);
    for(const rtabmap::IMU& data : sensorData) {
        file << std::fixed << std::setprecision(6) << data.linearAcceleration()[0] << ", ";
        file << std::fixed << std::setprecision(6) << data.linearAcceleration()[1] << ", ";
        file << std::fixed << std::setprecision(6) << data.linearAcceleration()[2] << ", ";
        file << std::fixed << std::setprecision(6) << data.angularVelocity()[0] << ", ";
        file << std::fixed << std::setprecision(6) << data.angularVelocity()[1] << ", ";
        file << std::fixed << std::setprecision(6) << data.angularVelocity()[2] << std::endl;
    }
    file.close();
}

void DataloaderStray::process() const {
    const auto [width, height] = splitImagesRGB(
        Dataloader::getPathData() / "rgb.mp4", 
        Dataloader::getPathRGB()
    );

    if(upscaleWithPromptDA_) upscaleImagesDepthPromptDA(
        Dataloader::getPathRGB(), 
        Dataloader::getPathData() / "depth", 
        Dataloader::getPathDepth()
    ); else upscaleImagesDepth(
        Dataloader::getPathData() / "depth", 
        Dataloader::getPathDepth()
    );

    const std::vector<float> stamps = writeTimestamps(
        Dataloader::getPathData() / "odometry.csv", 
        Dataloader::getPathStamps()
    );

    writeCalibration(
        Dataloader::getPathData() / "camera_matrix.csv", 
        Dataloader::getPathCalibrationFile(), width, height
    );

    writeIMU(
        Dataloader::getPathData() / "imu.csv",
        Dataloader::getPathIMU(),
        stamps
    );
}