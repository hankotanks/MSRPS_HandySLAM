#include "Config.h"

#include <filesystem>
#include <iostream>
#include <cassert>

#include <rtabmap/utilite/ULogger.h>

#include "ext/clipp.h"

Config::Config(int argc, char* argv[]) {
    std::string pathData, pathOut = "";
    auto cli = (
        clipp::value("data-path", pathData),
        clipp::option("-o", "--out").doc("species output directory, defaults to data path") &
            clipp::value("out-path", pathOut),
        clipp::option("-f", "--force").set(forceRebuild_).doc("force a rebuild of the temp folder"),
        clipp::option("-u", "--upscale").set(upscaleWithPromptDA_).doc("upscale imagery using PromptDA"),
        clipp::option("-s", "--save").set(savePoints_).doc("save generated point cloud as PLY")
    );

    if(!clipp::parse(argc, argv, cli)) {
        std::cout << clipp::make_man_page(cli) << std::endl;
        std::exit(1);
    }
    pathData_ = fs::path(pathData);
    if(!fs::exists(pathData_)) {
        UERROR("Provided data path does not exist [%s].", pathData.c_str());
        std::exit(1);
    }

    if(pathOut.size()) {
        pathOut_ = fs::path(pathOut);
        if(!fs::exists(pathOut_)) {
            fs::create_directories(pathOut_);
            if(!fs::exists(pathOut_)) {
                UERROR("Failed to create output folder [%s].", pathOut_.c_str());
                std::exit(1);
            }
        }
    } else {
        pathOut_ = pathData_;
    }
}