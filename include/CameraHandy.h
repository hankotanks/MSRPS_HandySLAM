#ifndef CAMERA_HANDY_H
#define CAMERA_HANDY_H

#include <filesystem>

#include <rtabmap/core/camera/CameraImages.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/camera/CameraRGBDImages.h>

#include "ext/lazycsv.h"

#include "Dataloader.h"

namespace rtabmap {
    class CameraHandy: public CameraRGBDImages {
    private:
        std::vector<IMU> sensorData_;
    public:
        CameraHandy(const Dataloader& data) : CameraRGBDImages(
            data.getPathRGB().string(), 
            data.getPathDepth().string()) {
            const auto [pathCalibration, cameraName] = data.getPathCalibration();
            CameraRGBDImages::init(pathCalibration.string(), cameraName);
            CameraImages::setTimestamps(false, data.getPathStamps().string());

            lazycsv::parser<lazycsv::mmap_source,
                lazycsv::has_header<false>,
                lazycsv::delimiter<','>,
                lazycsv::quote_char<'"'>,
                lazycsv::trim_chars<' ', '\t'>> parser { data.getPathIMU() };
                
            char* temp;
            for(const auto row : parser) {
                const auto raw = row.cells(0, 1, 2, 3, 4, 5);
                sensorData_.emplace_back(
                    cv::Vec3d(
                        std::strtof(raw[3].raw().data(), &temp), 
                        std::strtof(raw[4].raw().data(), &temp), 
                        std::strtof(raw[5].raw().data(), &temp)),
                    cv::Mat::eye(3, 3, CV_64F),
                    cv::Vec3d(
                        std::strtof(raw[0].raw().data(), &temp), 
                        std::strtof(raw[1].raw().data(), &temp), 
                        std::strtof(raw[2].raw().data(), &temp)),
                    cv::Mat::eye(3, 3, CV_64F)
                ); 
            }
        }

        SensorData takeImage(SensorCaptureInfo* info = 0) {
            SensorData data = Camera::takeImage();
            if(data.isValid() && data.id() < sensorData_.size()) data.setIMU(sensorData_[data.id()]);
            return data;
        }
    };
}


#endif // CAMERA_HANDY_H