#ifndef CAMERA_HANDY_H
#define CAMERA_HANDY_H

#include <filesystem>

#include <rtabmap/core/camera/CameraImages.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/camera/CameraRGBDImages.h>
#include <rtabmap/core/Transform.h>

#include "Config.h"
#include "Dataloader.h"

namespace rtabmap {
    class CameraHandy: public CameraRGBDImages {
    private:
        const std::vector<IMUEvent>& sensorData_;
        bool withIMU_;
        size_t frameCount_;
    public:
        CameraHandy(const Config& cfg, const Dataloader& data) : 
            CameraRGBDImages(data.getPathColor().string(), data.getPathDepth().string()),
            sensorData_(data.getEvents()),
            withIMU_(cfg.withIMU()),
            frameCount_(data.getFrameCount()) {
            const auto [pathCalibration, cameraName] = data.getPathCalibration();
            CameraRGBDImages::init(pathCalibration.string(), cameraName);
            CameraImages::setTimestamps(false, data.getPathStamps().string());
        }

        SensorData takeImage(SensorCaptureInfo* info = 0) {
            SensorData data = Camera::takeImage(info);
            if(data.isValid() && data.id() < frameCount_) {
                if(withIMU_) data.setIMU(sensorData_[data.id()].getData());
                else data.setIMU(rtabmap::IMU{});
            }   
            return data;
        }
    };
}


#endif // CAMERA_HANDY_H