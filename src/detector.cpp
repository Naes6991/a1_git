#include <iostream>
#include <string>  
#include <cstdlib>
#include <iomanip>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void detectAruco(
	cv::Mat img, 
	const int i, 
	const int verbosity, 
	std::vector<int> markerIds, 
	std::vector<std::vector<cv::Point2f>> markerCorners)
{
	// Detector parameters
	std::vector<std::vector<cv::Point2f>> rejectedCandidates;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	// Detect the tags
	cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

	// Print stuff if so desired
	if (verbosity == 1)
	{
		std::cout << "Frame: " << i+1 << std::endl;
		std::cout << "Number of detected ID's: " << markerIds.size() << std::endl;
		std::cout << "Id's found: ";
		for(int j=0; j<markerIds.size(); j++)
	    	std::cout << markerIds.at(j) << ", ";
	    std::cout << std::endl;
	}
	
	
}
