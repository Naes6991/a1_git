#include <filesystem>
#include <string>
#include <cassert>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <iostream>


#include "SLAM.h"
#include "detector.h"
#include "gaussian.hpp"

#define PI 3.14159265359


void runSLAMFromVideo(const std::filesystem::path &videoPath, const std::filesystem::path &cameraDataPath, int scenario, int interactive, const std::filesystem::path &outputDirectory)
{
    std::filesystem::path outputPath;
    bool doExport = !outputDirectory.empty();

    if (doExport)
    {
        std::string outputFilename = videoPath.stem().string()
                                   + "_out"
                                   + videoPath.extension().string();
        outputPath = outputDirectory / outputFilename;
    }

    // TODO
    // Good idea to sort out video export first
    // Run through each video file
    cv::VideoCapture vc(videoPath.string());
    if(!vc.isOpened())
    {
        std::cout << "Error opening video stream at " << videoPath.string() << std::endl;
        assert(0);
    }

    

    // Output nice things
    std::cout << "\nBeginning SLAM algorithm" << std::endl;

    // Simulation parameters
    double timestep;
    int nsteps = vc.get(cv::CAP_PROP_FRAME_COUNT), 
        nx     = 12, // [x, y, z, theta, psi, phi, dx, dy, dz, dtheta, dpsi, dphi]
        ny;

    // Models
    //StateProcessModel pm;
    //StateParameters pmparam;

    // Initialise filter
    Eigen::MatrixXd muhist(nx, nsteps);
    Eigen::MatrixXd sigmahist(nx, nsteps);

    Eigen::MatrixXd S(nx, nx);
    Eigen::VectorXd mu(nx);

    Eigen::VectorXd u;

    std::vector<marker_struct> savedMarkers;

    S.fill(0);           
    S.diagonal() << 1, 1, 1,        // Velocity noise 
                    1, 1, 1,        // Angular velocity noise
                    0, 0, 0,        // Position noise
                    0, 0, 0;        // Rotation noise

    mu <<   0, 0, 0,                // Initial velocity
            0, 0, 0,                // Initial angular velocity
            0, 0, 1.8,              // Initial positions (Tim is a tall boi)
            -PI, PI, 0;                // Initial rotations

    // initial mu that Tom used: muEKF <<
    //                          0, 0, 0
    //                          0, 0, 0
    //                          0, 0, -1.8
    //                          -pi, pi, 0

    std::cout << "Initial state estimate" << std::endl;
    std::cout << "mu = \n" << mu << std::endl;
    std::cout << "S = \n" << S << std::endl;

    std::cout << "Running filter with " << nsteps << " frames. " << std::endl;
    int kappa = 10;
    for(int i=0; i<nsteps; i++)
    {
        if (i%100==0)
            std::cout << "Frame " << i+1 << "/" << nsteps+1 << std::endl;

        cv::Mat img;
        bool ret = vc.read(img);
        if(ret==true)
        {
            // ---- Process mod update ----
            //timeUpdateContinuous(mu, S, u, pm, pmparam, timestep, muk, Sk);

            
            // ---- Measurement update ----
            // Detect marker
            std::vector<int> detectedMarkers;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            detectAruco(img, i, 0, detectedMarkers, markerCorners);

            // Find new markers/delete old markers/set up measurement vector
            //newMarkers = compareMarkers(savedMarkers, detectedMarkers);

            // Save new markers to x state
            //updateMarkerStates(mu, S, savedMarkers, newMarkers, kappa);
            //Eigen::MatrixXd & mu, 
    //Eigen::MatrixXd & S, 
    //std::vector<marker_struct> & savedMarkers, 
    //const std::vector<int> newMarkers, 
    //const int kappa

            // Conduct measurement update

            // ---- Check for nans ---- 

            // Save state and sigma history

        }
        else
        {
            std::cout << "Error opening video frame at frame " << i+1 << std::endl;
            assert(0);
        }
    }

    // End of - export video

}


// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// 
// Utility Functions
// 
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
int findMarkID(const std::vector<marker_struct> marker, const int findID)
{
    // Find marker
    for(int i=0; i<marker.size(); i++)
    {
        if(marker.at(i).ID == findID)
            return(i);
    }

    // Else no match
    return(-1);
}


std::vector<int> compareMarkers(std::vector<marker_struct> & savedMarkers, std::vector<int> & detectedMarkers)
{
    std::vector<int> savedDetected(savedMarkers.size());            // For comparing if old markers have been seen
    std::fill(savedDetected.begin(), savedDetected.end(), 0);       // 0's if marker hasn't been seen this frame
    
    // Sort through marker detections
    std::vector<int> newMarkers;
    for(int i = 0; i<detectedMarkers.size(); i++)
    {
        // Check if marker has a saved ID state
        int idx = findMarkID(savedMarkers, detectedMarkers.at(i));
        
        if(idx != -1) // Marker is already saved
        {
            savedDetected.at(idx) = 1;
            savedMarkers.at(idx).nMissed = 0;
        }
        else // New marker detected
        {
            // Add marker ID to savedMarkers, this marker is then added to state vectors in updateMarkerStates
            newMarkers.push_back(detectedMarkers.at(i));

        }

    }

    // Sort through savedMarkers
    for(int i = 0; i<savedMarkers.size(); i++)
    {
        if(savedDetected.at(i) == 0) // This marker not detected
            savedMarkers.at(i).nMissed++;
    }


    return newMarkers;
}

void updateMarkerStates(Eigen::MatrixXd & mu, 
    Eigen::MatrixXd & S, 
    std::vector<marker_struct> & savedMarkers, 
    const std::vector<int> newMarkers, 
    const int kappa)
{
    int nx = 12;        // Number of camera states
    int nmkr = 6;       // Number of marker states

    // Remove outdated markers
    // Loop through each savedMarker
    int nRemoved = 0;
    for(int i = 0; i<savedMarkers.size(); i++)
    {
        int idx = i - nRemoved;
        // Check if marker.nMissed is >10
        if(savedMarkers.at(idx).nMissed > 10)
        {
            // Remove rows from mu and S
            removeRow(mu, nx+nmkr*idx, nmkr);
            removeRow(S, nx+nmkr*idx, nmkr);
            // Remove columns from mu and S
            removeColumn(S, nx+nmkr*idx, nmkr);

            // Remove marker from savedMarkers
            savedMarkers.erase(savedMarkers.begin()+idx);
            nRemoved++;
        }
    }

    // Add new markers
    int newSize = nx + savedMarkers.size()*nmkr + newMarkers.size()*nmkr;
    int curSize = nx + savedMarkers.size()*nmkr;
    mu.conservativeResizeLike(Eigen::MatrixXd::Zero(newSize,1)); // CORRECT LATER WITH cv::aruco::estimatePoseSingleMarkers
    S.conservativeResizeLike(Eigen::MatrixXd::Zero(newSize,newSize));
    S.block(curSize, curSize, newSize-curSize, newSize-curSize) = Eigen::MatrixXd::Identity(newSize-curSize, newSize-curSize)*kappa;
    for(int i = 0; i<newMarkers.size(); i++)
    {
        marker_struct tmp{newMarkers.at(i), 0};     // tmp{markerID, nMissed}
        savedMarkers.push_back(tmp);
        
    }
}

// https://stackoverflow.com/questions/13290395/how-to-remove-a-certain-row-or-column-while-using-eigen-library-c
// I don't understand it but I like your funny words magic man
void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove, unsigned int numToRemove)
{
    // Number of rows after operation
    unsigned int numRows = matrix.rows()-numToRemove;
    unsigned int numCols = matrix.cols();

    // Ensure we're not trying to remove more rows than the matrix size
    assert(numToRemove<numRows);

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.bottomRows(numRows-rowToRemove);
    else
        assert(0);

    matrix.conservativeResize(numRows,numCols);
}

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove, unsigned int numToRemove)
{
    // Number of columns after operation
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-numToRemove;

    // Ensure we're not trying to remove more cols than the matrix size
    assert(numToRemove<numCols);

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.rightCols(numCols-colToRemove);
    else
        assert(0);

    matrix.conservativeResize(numRows,numCols);
}