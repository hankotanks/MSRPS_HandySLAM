#include <optional>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/DBDriverSqlite3.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/ULogger.h>

#include "Config.h"
#include "CameraHandy.h"
#include "DataloaderStray.h"
#include "DataloaderScanNet.h"
#include "PoseStream.h"
#include "CloudStream.h"

#define CLOUD_NAME "out"

#define MAX_POINTS 20000000

void run(const Config&, const Dataloader&);

int main(int argc, char* argv[]) {
    ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

    Config cfg(argc, argv);
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

void post(const Config&, const Dataloader&);

void run(const Config& cfg, const Dataloader& data) {
    if(data.skipSLAM()) {
        post(cfg, data);
        return;
    }

    rtabmap::CameraHandy camera(cfg, data);

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
        if(!(info.lost || pose.isNull())) {
            if(rtabmap.process(cameraData, pose)) if(rtabmap.getLoopClosureId() > 0) UINFO("Loop closure detected!");
        }
        
        cameraData = camera.takeImage();
    }
    rtabmap.close();

    delete odom;

    post(cfg, data);
   
    UINFO("Finished SLAM with [%d] frames.", cameraData.id());
}

std::map<int, int> components(const std::map<int, rtabmap::Transform>& poses, const std::multimap<int, rtabmap::Link>& links) {
    std::map<int, int> nodeGroups; // nodeId -> groupId
    std::set<int> visitedNodes;

    int groupId = 0;
    for(const auto& [nodeId, _] : poses) {
        if(visitedNodes.count(nodeId) == 0) {
            std::stack<int> stack;
            stack.push(nodeId);

            while(!stack.empty()) {
                int current = stack.top();
                stack.pop();

                if(visitedNodes.count(current)) continue;

                visitedNodes.insert(current);
                nodeGroups[current] = groupId;

                auto range = links.equal_range(current);
                for(auto it = range.first; it != range.second; ++it) {
                    int neighbor = it->second.to(); // or .from() depending on Link struct
                    if(poses.count(neighbor) && !visitedNodes.count(neighbor)) stack.push(neighbor);
                }

                // Also check reverse links (undirected graph)
                for(const auto& [fromId, link] : links) {
                    if(link.to() == current) {
                        if(poses.count(fromId) && !visitedNodes.count(fromId)) stack.push(fromId);
                    }
                }
            }

            groupId++;
        }
    }

    return nodeGroups;
}

void post(const Config& cfg, const Dataloader& data) {
    rtabmap::DBDriver* driver = new rtabmap::DBDriverSqlite3();

    rtabmap::SensorData cameraData;
    if(driver->openConnection(data.getPathDB(), false)) {
        std::map<int, rtabmap::Transform> poses = driver->loadOptimizedPoses();

        std::multimap<int, rtabmap::Link> links;
        driver->getAllLinks(links);

        std::map<int, int> nodeGroups = components(poses, links);

        std::map<int, size_t> groupSizes;
        for(const auto& [id, group] : nodeGroups) groupSizes[group]++;

        int mainGroup; size_t mainGroupSize = 0;
        for (const auto& [group, groupSize] : groupSizes) {
            if (groupSize > mainGroupSize) {
                mainGroup = group;
                mainGroupSize = groupSize;
            }
        }

        size_t cloudIdx = 0;
        std::ostringstream cloudName;
        cloudName << CLOUD_NAME << cloudIdx;

        std::optional<CloudStream> writeFrame = std::nullopt;
        if(cfg.savePoints()) writeFrame = CloudStream(cfg, cloudName.str());

        PoseStream writePose(cfg);

        rtabmap::Signature* node;
        for(const auto& [id, pose] : poses) {
            node = driver->loadSignature(id);
            if(node == nullptr) {
                UWARN("Failed to load node [%d] from database, skipping frame.", id);
                continue;
            }

            driver->loadNodeData(node);

            if(!nodeGroups.count(id) || nodeGroups[id] != mainGroup) {
                UWARN("Node [%d] not in largest connected graph, skipping frame.", id);
                continue;
            }

            cameraData = node->sensorData();
            if(!cameraData.isValid()) {
                UWARN("Retrieved sensor data is invalid [%d], skipping frame.", id);
                continue;
            }

            cameraData.uncompressData();
            writePose(pose, cameraData.stamp());

            if(cameraData.imageRaw().empty()) {
                UWARN("Failed to retrieve data from node [%d], skipping frame.", id);
                continue;
            }

            UINFO("Writing node [%d].", id);

            if(writeFrame) {
                CloudStream& writeFrameTemp = *writeFrame;
                writeFrameTemp(cameraData, pose);
                if(writeFrameTemp.pointCount() > MAX_POINTS) {
                    writeFrameTemp.close();
                    cloudName.str("");
                    cloudName.clear();
                    cloudName << CLOUD_NAME << (++cloudIdx);
                    writeFrame = CloudStream(cfg, cloudName.str());
                }
            }
            
            delete node;
        }

        driver->closeConnection();
    } else UERROR("Failed to open database, cannot write point cloud [%s].", data.getPathDB().c_str());

    delete driver;

    UINFO("Finished processing [%d] frames.", cameraData.id());
}