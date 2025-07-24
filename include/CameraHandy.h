#ifndef CAMERA_HANDY_H
#define CAMERA_HANDY_H

#include <filesystem>

#include <rtabmap/core/camera/CameraImages.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/camera/CameraRGBDImages.h>
#include <rtabmap/core/Transform.h>

#include "Dataloader.h"

namespace rtabmap {
    class CameraHandy: public CameraRGBDImages {
    public:
        CameraHandy(const Dataloader& data) : 
            CameraRGBDImages(data.getConfig().pathImagesColor.string(), data.getConfig().pathImagesDepth.string()) {
            CameraRGBDImages::init(data.getConfig().pathCalibration.parent_path(), "handy_camera");
            CameraImages::setTimestamps(false, data.getConfig().pathStamps.string());
        }

        SensorData takeImage(SensorCaptureInfo* info = 0) {
            return Camera::takeImage(info);
        }
    };
}


#endif // CAMERA_HANDY_H