#include <optional>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/DBDriverSqlite3.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/utilite/ULogger.h>

#include "Config.h"
#include "CameraHandy.h"
#include "DataloaderStray.h"
#include "DataloaderScanNet.h"
#include "PoseStream.h"
#include "CloudStream.h"
#include "MadgwickFilter.h"

#define CLOUD_NAME "out"

#define MAX_POINTS 20000000

void run(const Config&, const Dataloader&);

int main(int argc, char* argv[]) {
    ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

    Config cfg(argc, argv);
    switch(cfg.dataSource) {
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

    rtabmap::CameraHandy camera(data);

    rtabmap::ParametersMap params;
  	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDEnabled(), "true"));
  	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
  	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomGuessMotion(), "true"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomStrategy(), "1"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRegStrategy(), "0"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDNeighborLinkRefining(), "true"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), "6"));
    
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemNotLinkedNodesKept(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemInitWMWithAllNodes(), "false"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemImageKept(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemBinDataKept(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemSaveDepth16Format(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIntermediateNodeDataKept(), "true"));

    const std::vector<rtabmap::IMUEvent> events = data.getEvents();

    rtabmap::Odometry* odom = rtabmap::Odometry::create(params);
    odom->reset();
    rtabmap::OdometryInfo info;

    rtabmap::Rtabmap rtabmap;
    rtabmap.init(params, data.getPathDB());

    rtabmap::Transform pose, posePrev = odom->getPose();

    rtabmap::SensorData cameraData = camera.takeImage(), cameraDataProcessed;

    std::ofstream stamps(cfg.pathOut / "stamps.csv");

    int nodeIdPrev = 0;

    size_t currIdx = 0;
    while(cameraData.isValid()) {
        rtabmap::IMUEvent eventCurr = events[currIdx];
        if(cfg.withIMU) {
            q_est.q1 = posePrev.getQuaternionf().w();
            q_est.q2 = posePrev.getQuaternionf().x();
            q_est.q3 = posePrev.getQuaternionf().y();
            q_est.q4 = posePrev.getQuaternionf().z();

            const cv::Vec3d acc = eventCurr.getData().linearAcceleration();
            const cv::Vec3d gyr = eventCurr.getData().angularVelocity();

            if(currIdx > 0) {
                double stampCurr = events[currIdx].getStamp();
                double stampPrev = events[currIdx - 1].getStamp();

                UINFO("Integrating orientation over [%lf] seconds.", stampCurr - stampPrev);
                float rPrev, pPrev, yPrev, rCurr, pCurr, yCurr;
                eulerAngles(q_est, &rPrev, &pPrev, &yPrev);

                imu_filter(acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], static_cast<float>(stampCurr - stampPrev));

                eulerAngles(q_est, &rCurr, &pCurr, &yCurr);
                UINFO("r: [%.4f] to [%.4f]", rPrev, rCurr);
                UINFO("p: [%.4f] to [%.4f]", pPrev, pCurr);
                UINFO("y: [%.4f] to [%.4f]", yPrev, yCurr);
            }

            cameraDataProcessed = cameraData;
            cameraDataProcessed.setIMU(rtabmap::IMU {
                cv::Vec4d(q_est.q2, q_est.q3, q_est.q4, q_est.q1),
                cv::Mat::eye(4, 4, CV_64F) * 0.003,
                gyr,
                eventCurr.getData().angularVelocityCovariance(),
                acc,
                eventCurr.getData().linearAccelerationCovariance()
            });
        } else {
            cameraDataProcessed = cameraData;
            cameraDataProcessed.setIMU(events[currIdx].getData());
        }
        
        pose = odom->process(cameraDataProcessed, &info);
        if(!(info.lost || pose.isNull())) {
            if(rtabmap.process(cameraDataProcessed, pose)) {
                if(rtabmap.getLoopClosureId() > 0) UINFO("Loop closure detected!");

                int nodeId = rtabmap.getLastLocationId();
                if(nodeId > 0 && nodeId != nodeIdPrev) {
                    stamps << nodeId << ", " << std::fixed << std::setprecision(6) << eventCurr.getStamp() << std::endl;
                    nodeId = nodeId;
                }
            }
            posePrev = pose;
        } else if(cfg.withIMU) posePrev = rtabmap::Transform(posePrev.x(), posePrev.y(), posePrev.z(), q_est.q2, q_est.q3, q_est.q4, q_est.q1);
        
        cameraData = camera.takeImage();

        currIdx++;
    }
    stamps.close();
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
        std::optional<CloudStream> writeFrame = std::nullopt;
        if(cfg.pathCloud) writeFrame = CloudStream(*cfg.pathCloud, cloudIdx);

        PoseStream writePose(cfg.pathOut);

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
            writePose(pose, node->id());

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
                    writeFrame = CloudStream(*cfg.pathCloud, ++cloudIdx);
                }
            }
            
            delete node;
        }

        driver->closeConnection();
    } else UERROR("Failed to open database, cannot write point cloud [%s].", data.getPathDB().c_str());

    delete driver;

    UINFO("Finished processing [%d] frames.", cameraData.id());
}