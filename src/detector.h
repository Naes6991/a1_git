#ifndef DETECTOR_H
#define DETECTOR_H

#include <opencv2/core.hpp>

void detectAruco(cv::Mat img, const int i, const int verbosity, std::vector<int> markerIds, std::vector<std::vector<cv::Point2f>> markerCorners);

#endif