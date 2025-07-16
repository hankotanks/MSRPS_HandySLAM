#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/ULogger.h>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/DBDriverSqlite3.h>

#include "Config.h"
#include "CameraHandy.h"
#include "DataloaderStray.h"
#include "DataloaderScanNet.h"
#include "CloudStream.h"

#define CLOUD_NAME "out"

#define MAX_POINTS 20000000

void run(const Config&, const Dataloader&);

int main(int argc, char* argv[]) {
    ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

    Config cfg(argc, argv);

    const auto [pathData, pathOut] = cfg.getPaths();

    switch(cfg.dataSource()) {
        case STRAY: {
            DataloaderStray dataStray(cfg);
            if(dataStray.init()) run(cfg, dataStray);
            break;
        }
        case SCANNET: {
            DataloaderScanNet dataScanNet(cfg);
            if(dataScanNet.init()) run(cfg, dataScanNet);
            break;
        }
        default: UERROR("Unreachable!");
    }
    
    return 0;
}

void run(const Config& cfg, const Dataloader& data) {
    rtabmap::CameraHandy camera(data);

    rtabmap::ParametersMap params;
  	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDEnabled(), "true"));
  	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
  	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomGuessMotion(), "true"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomStrategy(), "0"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRegStrategy(), "0"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDNeighborLinkRefining(), "true"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), "6"));
    
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemNotLinkedNodesKept(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemInitWMWithAllNodes(), "false"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemImageKept(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemBinDataKept(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemSaveDepth16Format(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIntermediateNodeDataKept(), "true"));

    rtabmap::Odometry* odom = rtabmap::Odometry::create(params);
    rtabmap::OdometryInfo info;

    rtabmap::Rtabmap rtabmap;
    rtabmap.init(params, data.getPathDB());

    rtabmap::Transform pose;

    rtabmap::SensorData cameraData = camera.takeImage();
    while(cameraData.isValid()) {
        pose = odom->process(cameraData, &info);
        if(rtabmap.process(cameraData, pose)) {
            if(rtabmap.getLoopClosureId() > 0) UINFO("Loop closure detected!");
        }

        cameraData = camera.takeImage();
    }
    rtabmap.close();

    delete odom;

    if(cfg.savePoints()) {
        size_t cloudIdx = 0;
        std::ostringstream cloudName;
        cloudName << CLOUD_NAME << cloudIdx;

        CloudStream writeFrame(cfg, cloudName.str());

        rtabmap::DBDriver* driver = new rtabmap::DBDriverSqlite3();
        if(driver->openConnection(data.getPathDB(), false)) {
            std::map<int, rtabmap::Transform> poses = driver->loadOptimizedPoses();

            rtabmap::Signature* node;
            for(const auto& [id, pose] : poses) {
                node = driver->loadSignature(id);
                if(node == nullptr) {
                    UWARN("Failed to load node [%d] from database, skipping frame.", id);
                    continue;
                }

                driver->loadNodeData(node);

                cameraData = node->sensorData();
                cameraData.uncompressData();

                if(cameraData.imageRaw().empty()) {
                    UWARN("Failed to retrieve data from node [%d], skipping frame.", id);
                    continue;
                }

                UINFO("Writing node [%d].", id);

                writeFrame(cameraData, pose);

                if(writeFrame.pointCount() > MAX_POINTS) {
                    writeFrame.close();
                    cloudName.str("");
                    cloudName.clear();
                    cloudName << CLOUD_NAME << (++cloudIdx);
                    writeFrame = CloudStream(cfg, cloudName.str());
                }
                
                delete node;
            }

            driver->closeConnection();
        } else UERROR("Failed to open database, cannot write point cloud [%s].", data.getPathDB().c_str());

        delete driver;
    }
   
    UINFO("Finished processing [%d] frames.", cameraData.id());
}