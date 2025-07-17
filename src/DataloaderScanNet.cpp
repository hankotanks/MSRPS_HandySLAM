#include "DataloaderScanNet.h"

#include <filesystem>
#include <fstream>
#include <optional>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rtabmap/core/IMU.h>

#include "Dataloader.h"
#include "ext/json.h"

#include "PyScript.h"
#include "MadgwickFilter.h"

namespace fs = std::filesystem;

// TODO: Make this a member function so I don't have to pass withIMU
std::optional<std::pair<rtabmap::Transform, rtabmap::IMUEvent>> parseFrame(
    const nlohmann::detail::iteration_proxy_value<nlohmann::detail::iter_impl<nlohmann::basic_json<>>>& frame,
    const std::optional<float> prevStamp = std::nullopt
) {
    const auto frameObject = frame.value();

    // parse camera intrinsic
    if(!frameObject.contains("intrinsic")) 
        return std::nullopt;
    if(!frameObject["intrinsic"].is_array()) 
        return std::nullopt;
    std::vector<std::vector<float>> raw = frameObject["intrinsic"].get<std::vector<std::vector<float>>>();
    if(raw.size() != 3) return std::nullopt;
    for(const std::vector<float>& rawRow : raw) if(rawRow.size() != 3) 
        return std::nullopt;
    // construct intrinsic
    rtabmap::Transform intrinsics{
        raw[0][0], raw[0][1], raw[0][2], 0.f, 
        raw[1][0], raw[1][1], raw[1][2], 0.f, 
        raw[2][0], raw[2][1], raw[2][2], 0.f
    };

    // parse timestamp
    if(!frameObject.contains("timestamp")) 
        return std::nullopt;
    double stamp = frameObject["timestamp"].get<double>();

    // parse imu
    if(!frameObject.contains("imu")) 
        return std::nullopt;
    const auto sensorObject = frameObject["imu"];
    if(!sensorObject.contains("rotate_rate") || !sensorObject.contains("acceleration")) 
        return std::nullopt;
    if(!sensorObject["rotate_rate"].is_array() || !sensorObject["acceleration"].is_array()) 
        return std::nullopt;
    std::vector<float> gyro = sensorObject["rotate_rate"].get<std::vector<float>>();
    std::vector<float> acc = sensorObject["acceleration"].get<std::vector<float>>();
    if(gyro.size() != 3 || acc.size() != 3)
        return std::nullopt;

    if(prevStamp) imu_filter(acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], stamp - (*prevStamp));

    // put it all together
    rtabmap::IMU sensorData{
        cv::Vec4d(q_est.q2, q_est.q3, q_est.q4, q_est.q1),
        cv::Mat::eye(4, 4, CV_64F), // * 0.001,
        cv::Vec3d(gyro[0], gyro[1], gyro[2]),
        cv::Mat::eye(3, 3, CV_64F), // * 0.00225,
        cv::Vec3d(acc[0], acc[1], acc[2]),
        cv::Mat::eye(3, 3, CV_64F) // * 0.000225
    };

    return std::make_pair(intrinsics, rtabmap::IMUEvent(sensorData, stamp));
}

// TODO: Make this a member function so I don't have to pass withIMU
std::pair<rtabmap::Transform, std::vector<rtabmap::IMUEvent>> parseIntrinsicsAndIMU(const fs::path& pathJSON) {
    rtabmap::Transform intrinsics;
    std::vector<rtabmap::IMUEvent> events;

    std::ifstream jsonStream(pathJSON);
    if(!jsonStream.is_open()) return std::make_pair(intrinsics, events);

    nlohmann::json jsonParser;
    jsonStream >> jsonParser;

    q_est.q1 = 1.f;
    q_est.q2 = 0.f;
    q_est.q3 = 0.f;
    q_est.q4 = 0.f;

    bool first = true;
    for(const auto& frame : jsonParser.items()) {
        const auto frameData = first ? parseFrame(frame) : parseFrame(frame, events[events.size() - 1].getStamp());
        if(frameData) {
            events.push_back(std::get<1>(*frameData));
            if(first) {
                intrinsics = std::get<0>(*frameData);
                // TODO: Decide if its worth keeping each frame's intrinsic 
                // and writing a calibration file for each
                first = false; 
            }
        }
    }
    
    return std::make_pair(intrinsics, events);
}

bool unpackDepthBinary(
    const fs::path& pathDepthBinary, 
    const fs::path& pathDepthOut
) {
    UINFO("Began unpacking depth binary [%s].", pathDepthBinary.c_str());
    const bool result = PyScript::get().call("unpack_depth_binary", "ss", 
        pathDepthBinary.c_str(), pathDepthOut.c_str());
    if(result) UINFO("Finished unpacking depth binary.");
    return result;
}

bool DataloaderScanNet::process(const DataloaderValidation& validation) {
    const fs::path pathData = Dataloader::getPathData() / "iphone";
    const fs::path pathColorIn = pathData / "rgb.mkv";
    
    // RGB & IMU & CALIBRATION & TIMESTAMPS
    cv::Size originalColorSize;
    if(validation.colorFrames || validation.fullRebuild) {
        originalColorSize = Dataloader::splitColorVideoAndScale(pathColorIn);
        if(originalColorSize.empty()) {
            UERROR("Failed to split RGB frames [%s].", pathColorIn.c_str());
            return false;
        }
    }

    // IMU & CALIBRATION
    if(validation.events || validation.calibration || validation.fullRebuild) {
        if(originalColorSize.empty()) originalColorSize = Dataloader::queryColorVideoSize(pathColorIn);
        if(originalColorSize.empty()) {
            UERROR("Failed to retrieve dimensions of color imagery [%s]. ", pathColorIn.c_str());
            return false;
        }

        const fs::path pathJSON = pathData / "pose_intrinsic_imu.json";
        auto [intrinsics, events] = parseIntrinsicsAndIMU(pathJSON);
        if(events.empty()) {
            UERROR("Failed to parse IMU data or camera intrinsics [%s].", pathJSON.c_str());
            return false;
        }
        Dataloader::writeCalibration(intrinsics, originalColorSize);
        Dataloader::storeEvents(std::move(events));
    }

    // DEPTH
    if(validation.depthFrames || validation.fullRebuild) {
        const fs::path pathDepthTemp = Dataloader::getPathDepth().parent_path() / (Dataloader::getPathDepth().filename().string() + "_unpacked");
        const fs::path pathDepthBinary = pathData / "depth.bin";
        if(fs::exists(pathDepthTemp)) {
            UINFO("Using previously unpacked depth frames for upscaling [%s].", pathDepthTemp.c_str());
            if(!Dataloader::upscaleDepth(pathDepthTemp)) return false;
        } else {
            fs::create_directories(pathDepthTemp);
            if(unpackDepthBinary(pathDepthBinary, pathDepthTemp)) 
                Dataloader::upscaleDepth(pathDepthTemp);
            else {
                UERROR("Failed to unpack depth binary [%s].", pathDepthBinary.c_str());
                return false;
            }
        }
    }

    return true;
}