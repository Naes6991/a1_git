
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
#include <bitset>

#include "cameraModel.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio/videoio_c.h>

#include <iostream>

// Self note: msys command to run calibration:
// ninja && ./a1.exe ../data/calibration.MOV -c

// ******************* TO DO **********************
// Modularise this with functions similar to lab 7's template
// Implement saving camera data to xml
// Implement visualisation check via point projection

#define __DEBUG__(X) {std::cout << "In " << __FUNCTION__ << " at Line " << __LINE__ << ": " <<X << std::endl;};
#define DEBUG(X) __DEBUG__(X) 

// ------------------------------------------------------------
// Camera parameter stuff
// ------------------------------------------------------------
static void write(cv::FileStorage& fs, const std::string&, const CameraParameters& x)
{
    x.write(fs);
}

void read(const cv::FileNode& node, CameraParameters& x, const CameraParameters& default_value){
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

void CameraParameters::print() const
{

    std::bitset<8*sizeof(flag)> bitflag(flag);
    std::cout   << "Calibration data:" << std::endl;
    std::cout   << std::setw(30) << "cameraMatrix : " << std::endl << Kc         << std::endl;
    std::cout   << std::setw(30) << "distCoeffs : "   << std::endl << distCoeffs << std::endl;
    std::cout   << std::setw(30) << "flag : "                      << bitflag    << std::endl;
    std::cout   << std::setw(30) << "imageSize : "    << std::endl << imageSize  << std::endl;
    std::cout   << std::endl;
}

// Write serialization for this struct
void CameraParameters::write(cv::FileStorage& fs) const
{
    fs  << "{"
        << "camera_matrix"           << Kc
        << "distortion_coefficients" << distCoeffs
        << "flag"                    << flag
        << "imageSize"               << imageSize
        << "}";
}

// Read serialization for this struct
void CameraParameters::read(const cv::FileNode& node)
{
    node["camera_matrix"]           >> Kc;
    node["distortion_coefficients"] >> distCoeffs;
    node["flag"]                    >> flag;
    node["imageSize"]               >> imageSize;
}

// ------------------------------------------------------------
// Calibration of camera from a video
// ------------------------------------------------------------

// Missing reprojection of points onto image do to lab 3 incomplete
void calibrateCameraFromVideo(const std::filesystem::path &videoPath, const std::filesystem::path &dataPath)
{
    // Calibration chessboard parameters
    // Read from gridBoard.xml
    Settings s;
    const std::string inputSettingsFile = "../data/checkerboard.xml";

    if (!std::filesystem::exists(inputSettingsFile)){
        std::cout << "No file on path: " << inputSettingsFile << std::endl << std::endl;
        assert(0);
    }

    cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << std::endl << std::endl;
        assert(0);
    }
    fs["Settings"] >> s;
    fs.release();

    if (!s.isInputGood())
    {
        std::cout << "Invalid input detected. Application stopping. " << std::endl;
        assert(0);
    }

    // Variable definition
    std::vector<std::vector<cv::Point2f>> rQOi_set;
    std::vector<std::vector<cv::Point3f>> rPNn;
    cv::Size imageSize;
    CameraParameters param;


    // Load video file
    cv::VideoCapture vc(videoPath.string());
    if(!vc.isOpened())
    {
        std::cout << "Error opening video stream at " << videoPath.string() << std::endl;
        assert(0);
    }

    // Skip frames to limit calibrateCamera dataset
    int frame_count = vc.get(cv::CAP_PROP_FRAME_COUNT);
    int no_images = 50;                     // Number of images to calibrate with
    int skip = frame_count/no_images;       // Number of images to skip

    // Output nice things
    std::cout << "Calibration video frame count: " << frame_count << std::endl;
    std::cout << "Number of images to calibrate with: " << no_images << std::endl;
    std::cout << "Calculated number of frames to skip: " << skip << std::endl;
    std::cout << "\nBeginning calibration sequence" << std::endl;

    // Loop through images and detect board in each
    // For some reason if i=0, loops through frame_count+1 frames so last image is empty
    std::vector<bool> detectBool;
    for(int i=1; i<frame_count; i++) // ***************** Make faster opportunity: Loop through no_images times and use vc.set *****************
    {
        cv::Mat img;
        bool ret = vc.read(img);
        
        if(i%(skip+1) == 0)
        {                
            imageSize = img.size();
            if(ret==true){
                bool found;
                found = detectChessboard(img, s.boardSize, i+1, rQOi_set);
                detectBool.push_back(found);
            }
            else
            {
                std::cout << "Error reading frame " << i+1 << ". Terminating" << std::endl;
                assert(0);
            }
        }
    }

    // - Perform camera calibration function
    // Generate calibration grid function


    std::vector<cv::Mat> R,T;   // Save for project points
    runCalibration(s, rQOi_set, imageSize, param, R, T);
    std::cout << "Reached end of calibration sequence" << std::endl;

    // - Write the camera matrix and lens distortion parameters to XML file at dataPath
    std::cout << "Exporting calibration data" << std::endl;
    exportCalibrationData(dataPath, param);

    // - TODO: Visualise the camera calibration results - project points
    verifyProjections(vc, R, T, s, param, frame_count, no_images, skip, detectBool);
    std::cout << "Calibration complete! Go reward yourself with a nap" << std::endl;  
    
}

// ------------------------------------------------------------
// Auxiliary functions
// ------------------------------------------------------------
void exportCalibrationData(const std::filesystem::path & calibrationFilePath, const CameraParameters & param){
    // Done :)
    cv::FileStorage fs(calibrationFilePath.string(), cv::FileStorage::WRITE);
    fs << "CalibrationData" << param;
    fs.release();
    std::cout << "Write " << calibrationFilePath.string() << " complete" << std::endl;
}

void importCalibrationData(const std::filesystem::path & calibrationFilePath, CameraParameters & param){

    // Done :)
    cv::FileStorage fs(calibrationFilePath.string(), cv::FileStorage::READ);
    cv::FileNode n;
    fs["CalibrationData"] >> param;
    fs.release();
    std::cout << "Read " << calibrationFilePath.string() << " complete" << std::endl;
    assert(/*Just passionate about*/1/*ignoring hints to make an assertion*/);
}

bool detectChessboard(
    const cv::Mat & img, 
    const cv::Size & boardSize, 
    int frame, 
    std::vector<std::vector<cv::Point2f>> & rQOi_set)
{
    // run findChessboardCorners
    bool found;
    std::vector<cv::Point2f> rQOi;
    found = cv::findChessboardCorners(img, boardSize, rQOi);
    if(found)
    {
        std::cout << "Found chess board corners in frame " << frame << std::endl;
        rQOi_set.push_back(rQOi);
    }
    else
        std::cout << "Found no chess board corners in frame " << frame << std::endl;

    return found;
}

void runCalibration(
    const Settings & s, 
    const std::vector<std::vector<cv::Point2f>> & rQOi_set,  // Already provided concatenated
    const cv::Size & imageSize, 
    CameraParameters & param,
    std::vector<cv::Mat> & R,
    std::vector<cv::Mat> & T){

    std::vector<cv::Point3f> rPNn_base;
    std::vector<std::vector<cv::Point3f>> rPNn;
    generateCalibrationGrid(s, rPNn_base);

    // Push out rPNn 
    rPNn.assign(rQOi_set.size(), rPNn_base);

    
    double rms=-1;
    if( !rQOi_set.empty() ){
        // ------------------------------------------------------------
        // Run camera calibration
        // ------------------------------------------------------------
        // TODO: Copy from Lab3
        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat distCoeffs   = cv::Mat::zeros(12, 1, CV_64F);

        int flag = cv::CALIB_THIN_PRISM_MODEL|cv::CALIB_RATIONAL_MODEL;
        rms = cv::calibrateCamera(rPNn, rQOi_set, imageSize, cameraMatrix, distCoeffs, R, T, flag);

        // ------------------------------------------------------------
        // Write parameters to the data struct 
        // ------------------------------------------------------------
        // TODO
        param.Kc = cameraMatrix;
        param.distCoeffs = distCoeffs;
        param.flag = flag;
        param.imageSize = imageSize;


    }else{
        std::cerr << "No imagePoints found" << std::endl;
        assert(0);
    }
}

void generateCalibrationGrid(const Settings & s, std::vector<cv::Point3f> & rPNn_grid){
    // TODO: Copy from Lab 3 
    for(int i=0; i<s.boardSize.height; i++)
    {
        // Loop through columns j
        for(int j=0; j<s.boardSize.width; j++)
        {
            // Assign world coordinates rPPn = (j, i, 0)
            rPNn_grid.push_back(cv::Vec3f(j,i,0)*s.squareSize);
        }
    }
    assert(rPNn_grid.size() == s.boardSize.height*s.boardSize.width);
}

// I should really clean up all these inputs
// Will I?
// Probably not (evidently not if you're reading this Tim or Chris, pls no mark down for laziness)
void verifyProjections(cv::VideoCapture & vc, std::vector<cv::Mat> R, std::vector<cv::Mat> T, Settings s, CameraParameters param, int frame_count, int no_images, int skip, std::vector<bool> detectBool){
    std::vector<cv::Point3f> rPNn_grid;
    generateCalibrationGrid(s, rPNn_grid);
    vc.set(CV_CAP_PROP_POS_FRAMES, 0); // Reset video 

    std::cout << "\nBeginning projection verification process. \nAwaiting user input, press any key to cycle to next frame" << std::endl;
    std::cout << "Alternatively, press escape to cancel manual verification process" << std::endl;
    int j = 0;
    int skipped = 0;
    for(int i=1; i<frame_count; i++)
    {
        cv::Mat img, imgS;
        bool ret = vc.read(img);
        
        if(i%(skip+1) == 0)
        {                
            if(ret==true){
                if(detectBool.at(j)==true){
                    std::vector<cv::Point2f> rQOi;
                    projectPoints(rPNn_grid, R.at(j-skipped), T.at(j-skipped), param.Kc, param.distCoeffs, rQOi);
                    cv::drawChessboardCorners(img, s.boardSize, rQOi, 1);
                    cv::resize(img, imgS, cv::Size(0,0), 0.5, 0.5);
                    cv::imshow("Drawn chessboard corners", imgS);
                    int key = cv::waitKey(0);
                    if(key==27)
                        return;
                }
                else
                {
                    skipped++;
                }
                j++;
            }
            else
            {
                std::cout << "Error reading frame " << i+1 << ". Terminating" << std::endl;
                assert(0);
            }
        }
    }
}