#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/ULogger.h>

#include "Config.h"
#include "DataloaderStray.h"
#include "CameraHandy.h"

int main(int argc, char* argv[]) {
    ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

    Config cfg(argc, argv);

    const auto [pathData, pathOut] = cfg.getPaths();

    DataloaderStray data(cfg);
    data.init();
    data.validate();

    rtabmap::CameraHandy camera(data);

    rtabmap::ParametersMap params;
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
    // TODO: add other appropriate parameters

    rtabmap::Odometry* odom = rtabmap::Odometry::create(params);
    rtabmap::OdometryInfo info;

    rtabmap::Rtabmap rtabmap;
    rtabmap.init(params);

    rtabmap::SensorData cameraData = camera.takeImage();
    while(cameraData.isValid()) {
        rtabmap::Transform pose = odom->process(cameraData, &info);

        if(rtabmap.process(cameraData, pose)) {
            if(rtabmap.getLoopClosureId() > 0) UINFO("Loop closure detected!");
        }

        cameraData = camera.takeImage();
    }
    delete odom;
    
    return 0;
}