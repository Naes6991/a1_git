#ifndef SLAM_H
#define SLAM_H

#include <Eigen/Core>

#include <filesystem>

struct marker_struct{
    // Marker location stored in x state vector
    int ID;
    int nMissed;

};

void runSLAMFromVideo(const std::filesystem::path &videoPath, const std::filesystem::path &cameraDataPath, int scenario = 2, int interactive = 0, const std::filesystem::path &outputDirectory = "");
int findMarkID(const std::vector<marker_struct> marker, const int findID);
std::vector<int> compareMarkers(std::vector<marker_struct> & savedMarkers, std::vector<int> & detectedMarkers);
void updateMarkerStates(Eigen::MatrixXd & mu, Eigen::MatrixXd & S, std::vector<marker_struct> & savedMarkers, const std::vector<int> newMarkers, const int kappa);
void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove, unsigned int numToRemove);
void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove, unsigned int numToRemove);

#endif