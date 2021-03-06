#include <cstdlib>
#include <cassert>
#include <string>  
#include <filesystem>
#include <iostream>
#include <opencv2/core.hpp>

#include "cameraModel.hpp"
#include "SLAM.h"

int main(int argc, char* argv [])
{
    const cv::String keys =
        // Argument names | defaults | help message
        "{help h usage ?  |          | print this help message}"
        "{@input          | <none>   | path to input video file}"
        "{calibrate c     |          | perform camera calibration from input video}"
        "{scenario slam s | 1        | run SLAM on input video with scenario type (1:tags, 2:points, 3:ducks)}"
        "{interactive i   | 0        | interactivity (0:none, 1:last frame, 2:all frames)}"
        "{export e        |          | export video during SLAM}";

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("MCHA4400 Assignment 1");

    if (parser.has("help"))
    {
        parser.printMessage();
        return EXIT_SUCCESS;
    }

    int scenario = parser.get<int>("scenario");
    int interactive = parser.get<int>("interactive");
    bool hasExport = parser.has("export");
    bool hasCalibrate = parser.has("calibrate");
    std::filesystem::path inputVideoPath = parser.get<std::string>("@input");

    if (!parser.check())
    {
        parser.printErrors();
        return EXIT_FAILURE;
    }

    std::cout << "Input video: " << inputVideoPath.string() << std::endl;

    std::filesystem::path outputDirectory;
    if (hasExport)
    {
        std::filesystem::path appPath = parser.getPathToApplication();
        outputDirectory = appPath / ".." / "out";

        // Create output directory if we need to
        if (!std::filesystem::exists(outputDirectory))
        {
            std::cout << "Creating directory " << outputDirectory.string() << std::endl;
            std::filesystem::create_directory(outputDirectory);
        }
        std::cout << "Output directory set to " << outputDirectory.string() << std::endl;
    }

    std::filesystem::path appPath = parser.getPathToApplication();
    std::filesystem::path dataFile = appPath / ".." / "data" / "camera.xml";
    if (hasCalibrate)
    {
        std::cout << "Calibrating camera" << std::endl;
        calibrateCameraFromVideo(inputVideoPath, dataFile);
    }
    else
    {
        // IMPORT CAMERA DATA YA FUCK
        assert(1 <= scenario && scenario <= 3);
        assert(0 <= interactive && interactive <= 2);
        std::cout << "Running SLAM" << std::endl;
        runSLAMFromVideo(inputVideoPath, dataFile, scenario, interactive, outputDirectory);
    }

    return EXIT_SUCCESS;
}
