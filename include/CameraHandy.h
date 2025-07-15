#ifndef CAMERA_HANDY_H
#define CAMERA_HANDY_H

#include <filesystem>

#include <rtabmap/core/camera/CameraImages.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/camera/CameraRGBDImages.h>

#include "Dataloader.h"

namespace rtabmap {
    class CameraHandy: public CameraRGBDImages {
    private:
        const std::vector<IMUEvent>& sensorData_;
    public:
        CameraHandy(const Dataloader& data) : 
            CameraRGBDImages(data.getPathColor().string(), data.getPathDepth().string()),
            sensorData_(data.getEvents()) {
            const auto [pathCalibration, cameraName] = data.getPathCalibration();
            CameraRGBDImages::init(pathCalibration.string(), cameraName);
            CameraImages::setTimestamps(false, data.getPathStamps().string());
        }

        SensorData takeImage(SensorCaptureInfo* info = 0) {
            SensorData data = Camera::takeImage();
            if(data.isValid() && data.id() < sensorData_.size()) 
                data.setIMU(sensorData_[data.id()].getData());
            return data;
        }
    };
}


#endif // CAMERA_HANDY_H