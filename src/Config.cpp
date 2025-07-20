#include "Config.h"

#include <filesystem>
#include <iostream>
#include <cassert>
#include <optional>

#include <rtabmap/utilite/ULogger.h>

#include "ext/clipp.h"

Config::Config(int argc, char* argv[]) {
    std::string pathDataRaw, pathOutRaw;
    
    bool savePointCloud = false;

    auto cli = (
        clipp::command("stray").set(dataSource, STRAY) | 
            clipp::command("scannet").set(dataSource, SCANNET),
        clipp::value("data-path", pathDataRaw),
        clipp::value("out-path", pathOutRaw),
        clipp::option("-f", "--force").set(forceRebuild).doc("force a rebuild of the temp folder"),
        clipp::option("-p", "--post").set(skipSLAM).doc("skip SLAM and use database from previous run (if possible)"),
        clipp::option("-u", "--upscale").set(upscalingMethod, PROMPTDA).doc("upscale imagery using PromptDA"),
        clipp::option("-s", "--save").set(savePointCloud).doc("save generated point cloud as PLY"),
        clipp::option("-i", "--imu").set(withIMU).doc("integrate orientation using IMU data")
    );

    if(!clipp::parse(argc, argv, cli)) {
        std::cout << clipp::make_man_page(cli) << std::endl;
        std::exit(1);
    }
    pathData = fs::path(pathDataRaw);
    if(!fs::exists(pathData)) {
        UERROR("Provided data path does not exist [%s].", pathData.c_str());
        std::exit(1);
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
}