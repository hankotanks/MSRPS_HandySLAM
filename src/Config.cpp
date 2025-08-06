#include "Config.h"

#include <filesystem>
#include <iostream>
#include <cassert>
#include <optional>
#include <fstream>
#include <unordered_set>

#include <rtabmap/utilite/ULogger.h>

#include "ext/clipp.h"

size_t countImagery(const fs::path& pathImagery) {
    size_t count = 0;

    while(true) {
        std::ostringstream filename;
        filename << std::setw(6) << std::setfill('0') << count << ".png";
        fs::path filePath = pathImagery / filename.str();
        if(fs::exists(filePath) && fs::is_regular_file(filePath)) ++count;
        else break;
    }

    return count;
}

bool validateImagery(fs::path const& pathImagery, size_t expectedCount) {
    if(!fs::exists(pathImagery) || !fs::is_directory(pathImagery)) return false;

    std::unordered_set<std::string> expected;
    for(size_t i = 0; i < expectedCount; ++i) {
        std::ostringstream filename;
        filename << std::setw(6) << std::setfill('0') << i << ".png";
        expected.insert(filename.str());
    }

    std::unordered_set<std::string> found;
    for(const auto& entry : fs::directory_iterator(pathImagery)) {
        if(!entry.is_regular_file()) continue;

        std::string name = entry.path().filename().string();
        if(expected.find(name) != expected.end()) found.insert(name);
        else fs::remove(entry.path());
    }

    return found.size() == expected.size();
}

bool validateLines(const std::filesystem::path& pathFile, size_t expectedCount) {
    std::ifstream in(pathFile);
    if(!in) return false;
    size_t lines = 0;
    std::string line;
    while(std::getline(in, line)) ++lines;
    in.close();
    return lines >= expectedCount;
}

// truthy if validation succeeds
void Config::validate() {
    rebuildImagesColor = false;
    rebuildImagesDepth = false;
    rebuildEvents = false;
    rebuildCalibration = false;

    if(!fs::exists(pathTemp)) {
        UERROR("Validation failed. Temp folder did not exist [%s].", pathTemp.c_str());
        rebuildImagesColor = true;
        rebuildImagesDepth = true;
        rebuildEvents = true;
        rebuildCalibration = true;
        return;
    }
    
    if(!fs::exists(pathCalibration)) {
        UERROR("Validation failed. Calibration file did not exist [%s].", pathCalibration.c_str());
        rebuildCalibration = true;
    }

    if(!fs::exists(pathIMU)) {
        UERROR("Validation failed. IMU file did not exist [%s].", pathIMU.c_str());
        rebuildEvents = true;
    }

    if(!fs::exists(pathStamps)) {
        UERROR("Validation failed. Timestamps file did not exist [%s].", pathStamps.c_str());
        rebuildEvents = true;
    }

    if(!fs::exists(pathImagesColor)) {
        UERROR("Validation failed. Color imagery folder did not exist [%s].", pathImagesColor.c_str());
        rebuildImagesColor = true;
    }

    if(!fs::exists(pathImagesDepth)) {
        UERROR("Validation failed. Depth imagery folder did not exist [%s].", pathImagesDepth.c_str());
        rebuildImagesDepth = true;
    }

    size_t frames, framesColor, framesDepth;
    framesColor = countImagery(pathImagesColor);
    UINFO("Counted [%zu] frames of color imagery.", framesColor);
    if(!framesColor) rebuildImagesColor = true;

    framesDepth = countImagery(pathImagesDepth);
    UINFO("Counted [%zu] frames of depth imagery.", framesDepth);
    if(!framesDepth) rebuildImagesDepth = true;

    frames = std::min(framesColor, framesDepth);

    if(!validateLines(pathIMU, frames)) {
        UERROR("Validation failed. IMU file contains insufficient lines.");
        rebuildEvents = true;
    }

    if(!validateLines(pathStamps, frames)) {
        UERROR("Validation failed. Timestamps file contains insufficient lines.");
        rebuildEvents = true;
    }

    if(frames) {
        if(!validateImagery(pathImagesColor, frames)) {
            UERROR("Validation failed. Unable to rectify color imagery.");
            rebuildImagesColor = true;
        }

        if(!validateImagery(pathImagesDepth, frames)) {
            UERROR("Validation failed. Unable to rectify depth imagery.");
            rebuildImagesDepth = true;
        }
    }
    
}

Config::Config(int argc, char* argv[]) {
    std::string pathDataRaw, pathOutRaw;
    
    bool savePointCloud = false;
    bool rebuildAll = false;

    auto cli = (
        (
            clipp::command("stray").set(dataSource, STRAY).doc("process a scene from the Stray scanner app") | 
                clipp::command("scannet").set(dataSource, SCANNET).doc("process a scene from the ScanNet++ dataset"),
            clipp::value("data-path", pathDataRaw),
            clipp::value("out-path", pathOutRaw),
            clipp::option("-r", "--rebuild").set(rebuildAll, true).doc("force a rebuild of the temp folder"),
            clipp::option("-u", "--upscale").set(upscalingMethod, PROMPTDA).doc("upscale imagery using PromptDA"),
            clipp::option("-i", "--integration").set(integrated, true).doc("integrate orientation using IMU data")
        ) | (
            clipp::command("post").set(dataSource, POST).doc("skip SLAM and use database from previous run (if possible)"),
            clipp::value("out-path", pathOutRaw)
        ),
        clipp::option("-s", "--save").set(savePointCloud).doc("save generated point cloud as PLY")
    );

    if(!clipp::parse(argc, argv, cli)) {
        std::cout << clipp::make_man_page(cli) << std::endl;
        std::exit(1);
    }

    if(dataSource != POST) {
        pathData = fs::path(pathDataRaw);
        if(!fs::exists(pathData)) {
            UERROR("Provided data path does not exist [%s].", pathData.c_str());
            std::exit(1);
        }
    }

    pathOut = fs::path(pathOutRaw);
    if(!fs::exists(pathOut)) {
        fs::create_directories(pathOut);
        if(!fs::exists(pathOut)) {
            UERROR("Failed to create output folder [%s].", pathOut.c_str());
            std::exit(1);
        }
    }

    pathCloud = savePointCloud ? std::optional(pathOut / "cloud") : std::nullopt;
    if(pathCloud) {
        if(!fs::exists(*pathCloud)) {
            fs::create_directories(*pathCloud);
            if(!fs::exists(*pathCloud)) {
                UERROR("Failed to create output folder for point cloud [%s].", pathCloud->c_str());
                std::exit(1);
            }
        } else if(!(fs::is_directory(*pathCloud) && fs::directory_iterator(*pathCloud) == fs::end(fs::directory_iterator()))) {
            UERROR("Output point cloud directory is not empty [%s].", pathCloud->c_str());
            std::exit(1);
        }
    }

    pathTemp = pathData / "temp";
    pathDB = pathOut / "temp.db";
    pathCalibration = pathTemp / "handy_camera.yaml";
    pathIMU = pathTemp / "imu.csv";
    pathStamps = pathTemp / "stamps.txt";
    pathImagesColor = pathTemp / "color";
    pathImagesDepth = pathTemp / ((upscalingMethod == NAIVE) ? "depth" : "depth_upscaled");

    pathPoses = pathOut / "poses.csv";
    pathStampsOut = pathOut / "stamps.csv";

    if(rebuildAll) {
        rebuildImagesColor = true;
        rebuildImagesDepth = true;
        rebuildEvents = true;
        rebuildCalibration = true;
    }

    if(dataSource == POST) {
        if(!fs::exists(pathDB)) {
            UERROR("Unable to skip SLAM without prior database [%s].", pathDB.c_str());
            std::exit(1);
        }

        fs::remove(pathPoses);
    } else {
        if(fs::exists(pathDB)) fs::remove(pathDB);

        if(rebuildAll) {
            fs::remove_all(pathTemp);
            fs::create_directories(pathTemp);
        } else Config::validate();

        if(rebuildCalibration) fs::remove(pathCalibration);
        if(rebuildEvents) {
            fs::remove(pathIMU);
            fs::remove(pathStamps);
        }

        if(rebuildImagesColor && fs::exists(pathImagesColor)) fs::remove_all(pathImagesColor);
        if(rebuildImagesDepth && fs::exists(pathImagesDepth)) fs::remove_all(pathImagesDepth);
        if(!fs::exists(pathImagesColor)) fs::create_directories(pathImagesColor);
        if(!fs::exists(pathImagesDepth)) fs::create_directories(pathImagesDepth);
    }  
}