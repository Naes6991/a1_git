#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <filesystem>

#include "settings.h"

#include <opencv2/opencv.hpp>

struct CameraParameters{
    cv::Mat Kc;                 // Camera Matrix
    cv::Mat distCoeffs;         // Distortion coefficients
    int flag            = 0;    // Calibration flag
    double fieldOfView  = 150;  // Describe the arc of the view cone
    cv::Size imageSize;         // Image size
    // Read and write methods required for class serialisation
    void read(const cv::FileNode & node);
    void write(cv::FileStorage& fs) const;
    // Convenience function
    void print() const;
};

void exportCalibrationData(const std::filesystem::path & calibrationFilePath, const CameraParameters & param);
void importCalibrationData(const std::filesystem::path & calibrationFilePath, CameraParameters & param);
void calibrateCameraFromVideo(const std::filesystem::path &videoPath, const std::filesystem::path &dataPath);
bool detectChessboard(const cv::Mat & img, const cv::Size & boardSize, int frame, std::vector<std::vector<cv::Point2f>> & rQOi_set);
void runCalibration(const Settings & s, const std::vector<std::vector<cv::Point2f>> & rQOi_set,  const cv::Size & imageSize, CameraParameters & param, std::vector<cv::Mat> & R, std::vector<cv::Mat> & T);
void generateCalibrationGrid(const Settings & s, std::vector<cv::Point3f> & rPNn_grid);
void verifyProjections(cv::VideoCapture & vc, std::vector<cv::Mat> R, std::vector<cv::Mat> T, Settings s, CameraParameters param, int frame_count, int no_images, int skip, std::vector<bool> detectBool);

#endif