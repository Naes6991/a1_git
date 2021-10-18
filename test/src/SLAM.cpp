#include <catch2/catch.hpp>
#include <Eigen/Core>
#include "../../src/SLAM.h"

#include <iostream>


// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// 
// compareMarkers tests
// 
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// Still need to implement: Multiple markers, adding new markers
// Check nMissed in savedMarkers increments
SCENARIO("compareMarkers with savedMarkers = m1, detectedMarkers = empty")
{
    std::vector<marker_struct> savedMarkers;
    std::vector<int> detectedMarkers;

    marker_struct mark1 {1, 0};

    savedMarkers.push_back(mark1);

    WHEN("compareMarkers is called")
    {
        compareMarkers(savedMarkers, detectedMarkers);

        THEN("m1's nMissed counter is incremented by 1")
        {
            REQUIRE(savedMarkers.at(0).nMissed == 1);
        }
    }
}

// Check nMissed is reset to 0 when m1 is detected again
SCENARIO("compareMarkers with savedMarkers = m1, detectedMarkers = m1")
{
    std::vector<marker_struct> savedMarkers;
    std::vector<int> detectedMarkers;

    marker_struct mark1 {1, 5};

    savedMarkers.push_back(mark1);
    detectedMarkers.push_back(1);

    WHEN("compareMarkers is called")
    {
        compareMarkers(savedMarkers, detectedMarkers);

        THEN("m1's nMissed counter is incremented by 1")
        {
            REQUIRE(savedMarkers.at(0).nMissed == 0);
        }
    }
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// 
// Row/Column removal tests
// 
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
SCENARIO("Remove the 3rd row and 3rd column from a 5x5 matrix")
{
    Eigen::MatrixXd x(5,5);
    x << 1, 2, 3, 4, 5,
         2, 3, 4, 5, 6,
         3, 4, 5, 6, 7,
         4, 5, 6, 7, 8,
         5, 6, 7, 8, 9;

    int nRowToRemove = 1, nColToRemove = 1;

    WHEN("removeRow and removeColumn is called")
    {
        removeRow(x, 2, nRowToRemove);
        removeColumn(x, 2, nColToRemove);

        THEN("x is the correct shape")
        {
            REQUIRE(x.cols() == 4);
            REQUIRE(x.rows() == 4);

            AND_THEN("x has the correct values")
            {
                // x(:,0)
                CHECK(x(0,0)==1);
                CHECK(x(1,0)==2);
                CHECK(x(2,0)==4);
                CHECK(x(3,0)==5);

                // x(:,1)
                CHECK(x(0,1)==2);
                CHECK(x(1,1)==3);
                CHECK(x(2,1)==5);
                CHECK(x(3,1)==6);

                // x(:,2)
                CHECK(x(0,2)==4);
                CHECK(x(1,2)==5);
                CHECK(x(2,2)==7);
                CHECK(x(3,2)==8);

                // x(:,3)
                CHECK(x(0,3)==5);
                CHECK(x(1,3)==6);
                CHECK(x(2,3)==8);
                CHECK(x(3,3)==9);
            }
        }
    }
}

SCENARIO("Remove the 3rd and 4th rows and columns from a 5x5 matrix")
{
    Eigen::MatrixXd x(5,5);
    x << 1, 2, 3, 4, 5,
         2, 3, 4, 5, 6,
         3, 4, 5, 6, 7,
         4, 5, 6, 7, 8,
         5, 6, 7, 8, 9;

    int nRowToRemove = 2, nColToRemove = 2;

    WHEN("removeRow and removeColumn is called")
    {
        removeRow(x, 2, nRowToRemove);
        removeColumn(x, 2, nColToRemove);
        
        THEN("x is the correct shape")
        {
            REQUIRE(x.cols() == 3);
            REQUIRE(x.rows() == 3);

            AND_THEN("x has the correct values")
            {
                // x(:,0)
                CHECK(x(0,0)==1);
                CHECK(x(1,0)==2);
                CHECK(x(2,0)==5);

                // x(:,1)
                CHECK(x(0,1)==2);
                CHECK(x(1,1)==3);
                CHECK(x(2,1)==6);

                // x(:,2)
                CHECK(x(0,2)==5);
                CHECK(x(1,2)==6);
                CHECK(x(2,2)==9);
            }
        }
    }
}

SCENARIO("updateMarkerStates with m1.nMissed=0, m2.nMissed=11, m3.nMissed=15, m4.nMissed = 5 with no new markers")
{
    int nx = 12;
    int nmkr = 6;
    int nm1 = 4;
    int sizeAfter = nx + nmkr*nm1 - nmkr*2;

    Eigen::MatrixXd mu(nx+nmkr*nm1,1);
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nx+nmkr*nm1, nx+nmkr*nm1);
    std::vector<marker_struct> savedMarkers;
    std::vector<int> newMarkers;
    int kappa = 10;

    mu << 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,           // Camera states
            1, 1, 1, 1, 1, 1,                           // Marker 1 states
            2, 2, 2, 2, 2, 2,                           // Marker 2 states
            3, 3, 3, 3, 3, 3,                           // Marker 3 states
            4, 4, 4, 4, 4, 4;                           // Marker 4 states
    S.diagonal() << 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,       
                    1, 1, 1, 1, 1, 1,                           
                    2, 2, 2, 2, 2, 2,                        
                    3, 3, 3, 3, 3, 3,                           
                    4, 4, 4, 4, 4, 4;      

    marker_struct tmp_m;
    tmp_m = {1, 0};
    savedMarkers.push_back(tmp_m);
    tmp_m = {2, 11};
    savedMarkers.push_back(tmp_m);
    tmp_m = {3, 15};
    savedMarkers.push_back(tmp_m);
    tmp_m = {4, 5};        
    savedMarkers.push_back(tmp_m);

    WHEN("updateMarkerStates is called")
    {
        updateMarkerStates(mu, S, savedMarkers, newMarkers, kappa);
        
        THEN("mu, S and savedMarkers are the correct shapes")
        {
            REQUIRE(mu.rows() == sizeAfter);
            REQUIRE(mu.cols() == 1);
            REQUIRE(S.rows() == sizeAfter);
            REQUIRE(S.cols() == sizeAfter);
            REQUIRE(savedMarkers.size() == 2);

            AND_THEN("mu, S and savedMarkers have the correct values")
            {
                // Camera states
                CHECK(mu(0) == 9);
                CHECK(mu(1) == 9);
                CHECK(mu(2) == 9);
                CHECK(mu(3) == 9);
                CHECK(mu(4) == 9);
                CHECK(mu(5) == 9);
                CHECK(mu(6) == 9);
                CHECK(mu(7) == 9);
                CHECK(mu(8) == 9);
                CHECK(mu(9) == 9);
                CHECK(mu(10) == 9);
                CHECK(mu(11) == 9);

                CHECK(S.diagonal()(0) == 9);
                CHECK(S.diagonal()(1) == 9);
                CHECK(S.diagonal()(2) == 9);
                CHECK(S.diagonal()(3) == 9);
                CHECK(S.diagonal()(4) == 9);
                CHECK(S.diagonal()(5) == 9);
                CHECK(S.diagonal()(6) == 9);
                CHECK(S.diagonal()(7) == 9);
                CHECK(S.diagonal()(8) == 9);
                CHECK(S.diagonal()(9) == 9);
                CHECK(S.diagonal()(10) == 9);
                CHECK(S.diagonal()(11) == 9);

                // Marker states
                // m1
                CHECK(mu(12) == 1);
                CHECK(mu(13) == 1);
                CHECK(mu(14) == 1);
                CHECK(mu(15) == 1);
                CHECK(mu(16) == 1);
                CHECK(mu(17) == 1);
                CHECK(S.diagonal()(12) == 1);
                CHECK(S.diagonal()(13) == 1);
                CHECK(S.diagonal()(14) == 1);
                CHECK(S.diagonal()(15) == 1);
                CHECK(S.diagonal()(16) == 1);
                CHECK(S.diagonal()(17) == 1);

                // m2
                CHECK(mu(18) == 4);
                CHECK(mu(19) == 4);
                CHECK(mu(20) == 4);
                CHECK(mu(21) == 4);
                CHECK(mu(22) == 4);
                CHECK(mu(23) == 4);
                CHECK(S.diagonal()(18) == 4);
                CHECK(S.diagonal()(19) == 4);
                CHECK(S.diagonal()(20) == 4);
                CHECK(S.diagonal()(21) == 4);
                CHECK(S.diagonal()(22) == 4);
                CHECK(S.diagonal()(23) == 4);

                // savedMarkers 
                CHECK(savedMarkers.at(0).ID == 1);
                CHECK(savedMarkers.at(0).nMissed == 0);
                CHECK(savedMarkers.at(1).ID == 4);
                CHECK(savedMarkers.at(1).nMissed == 5);

                
            }
        }
    }
}

SCENARIO("updateMarkerStates with no saved markers and 2 new markers")
{
    int nx = 12;         // Number of camera states
    int nmkr = 6;        // Number of marker states
    int nm1 = 0;         // Number of markers before
    int nm2 = 2;         // Number of markers after
    int sizeAfter = nx + nmkr*nm1 + nmkr*nm2;

    Eigen::MatrixXd mu(nx+nmkr*nm1,1);
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nx+nmkr*nm1, nx+nmkr*nm1);
    std::vector<marker_struct> savedMarkers;
    std::vector<int> newMarkers;
    int kappa = 10;

    mu << 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9;           // Camera states
    S.diagonal() << 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9;   

    // Push back new marker ID's
    newMarkers.push_back(4);
    newMarkers.push_back(6);

    WHEN("updateMarkerStates is called")
    {
        updateMarkerStates(mu, S, savedMarkers, newMarkers, kappa);

        THEN("mu and S are the correct shapes")
        {
            REQUIRE(mu.rows() == sizeAfter);
            REQUIRE(mu.cols() == 1);
            REQUIRE(S.rows() == sizeAfter);
            REQUIRE(S.cols() == sizeAfter);
            REQUIRE(savedMarkers.size() == nm2);

            AND_THEN("mu and S have the correct values")
            {
                // Camera states
                CHECK(mu(0) == 9);
                CHECK(mu(1) == 9);
                CHECK(mu(2) == 9);
                CHECK(mu(3) == 9);
                CHECK(mu(4) == 9);
                CHECK(mu(5) == 9);
                CHECK(mu(6) == 9);
                CHECK(mu(7) == 9);
                CHECK(mu(8) == 9);
                CHECK(mu(9) == 9);
                CHECK(mu(10) == 9);
                CHECK(mu(11) == 9);

                CHECK(S.diagonal()(0) == 9);
                CHECK(S.diagonal()(1) == 9);
                CHECK(S.diagonal()(2) == 9);
                CHECK(S.diagonal()(3) == 9);
                CHECK(S.diagonal()(4) == 9);
                CHECK(S.diagonal()(5) == 9);
                CHECK(S.diagonal()(6) == 9);
                CHECK(S.diagonal()(7) == 9);
                CHECK(S.diagonal()(8) == 9);
                CHECK(S.diagonal()(9) == 9);
                CHECK(S.diagonal()(10) == 9);
                CHECK(S.diagonal()(11) == 9);

                // Marker states
                // m1
                CHECK(mu(12) == 0);
                CHECK(mu(13) == 0);
                CHECK(mu(14) == 0);
                CHECK(mu(15) == 0);
                CHECK(mu(16) == 0);
                CHECK(mu(17) == 0);
                CHECK(S.diagonal()(12) == 10);
                CHECK(S.diagonal()(13) == 10);
                CHECK(S.diagonal()(14) == 10);
                CHECK(S.diagonal()(15) == 10);
                CHECK(S.diagonal()(16) == 10);
                CHECK(S.diagonal()(17) == 10);

                // m2
                CHECK(mu(18) == 0);
                CHECK(mu(19) == 0);
                CHECK(mu(20) == 0);
                CHECK(mu(21) == 0);
                CHECK(mu(22) == 0);
                CHECK(mu(23) == 0);
                CHECK(S.diagonal()(18) == 10);
                CHECK(S.diagonal()(19) == 10);
                CHECK(S.diagonal()(20) == 10);
                CHECK(S.diagonal()(21) == 10);
                CHECK(S.diagonal()(22) == 10);
                CHECK(S.diagonal()(23) == 10);

                // savedMarkers 
                CHECK(savedMarkers.at(0).ID == 4);
                CHECK(savedMarkers.at(0).nMissed == 0);
                CHECK(savedMarkers.at(1).ID == 6);
                CHECK(savedMarkers.at(1).nMissed == 0);
            }
        }
    }
}


// TODO: TEST NEW MARKERS NOT IMPLEMENTED
SCENARIO("updateMarkerStates with m1.nMissed=0, m2.nMissed=11, m3.nMissed=15, m4.nMissed = 5 with 2 new markers")
{
    int nx = 12;
    int nmkr = 6;
    int nm1 = 4;
    int sizeAfter = nx + nmkr*nm1 - nmkr*2;

    Eigen::MatrixXd mu(nx+nmkr*nm1,1);
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nx+nmkr*nm1, nx+nmkr*nm1);
    std::vector<marker_struct> savedMarkers;
    std::vector<int> newMarkers;
    int kappa = 10;

    mu << 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,           // Camera states
            1, 1, 1, 1, 1, 1,                           // Marker 1 states
            2, 2, 2, 2, 2, 2,                           // Marker 2 states
            3, 3, 3, 3, 3, 3,                           // Marker 3 states
            4, 4, 4, 4, 4, 4;                           // Marker 4 states
    S.diagonal() << 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,       
                    1, 1, 1, 1, 1, 1,                           
                    2, 2, 2, 2, 2, 2,                        
                    3, 3, 3, 3, 3, 3,                           
                    4, 4, 4, 4, 4, 4;      

    marker_struct tmp_m;
    tmp_m = {1, 0};
    savedMarkers.push_back(tmp_m);
    tmp_m = {2, 11};
    savedMarkers.push_back(tmp_m);
    tmp_m = {3, 15};
    savedMarkers.push_back(tmp_m);
    tmp_m = {4, 5};        
    savedMarkers.push_back(tmp_m);

    WHEN("updateMarkerStates is called")
    {
        updateMarkerStates(mu, S, savedMarkers, newMarkers, kappa);
        
        THEN("mu, S and savedMarkers are the correct shapes")
        {
            REQUIRE(mu.rows() == sizeAfter);
            REQUIRE(mu.cols() == 1);
            REQUIRE(S.rows() == sizeAfter);
            REQUIRE(S.cols() == sizeAfter);
            REQUIRE(savedMarkers.size() == 2);

            AND_THEN("mu, S and savedMarkers have the correct values")
            {
                // Camera states
                CHECK(mu(0) == 9);
                CHECK(mu(1) == 9);
                CHECK(mu(2) == 9);
                CHECK(mu(3) == 9);
                CHECK(mu(4) == 9);
                CHECK(mu(5) == 9);
                CHECK(mu(6) == 9);
                CHECK(mu(7) == 9);
                CHECK(mu(8) == 9);
                CHECK(mu(9) == 9);
                CHECK(mu(10) == 9);
                CHECK(mu(11) == 9);

                CHECK(S.diagonal()(0) == 9);
                CHECK(S.diagonal()(1) == 9);
                CHECK(S.diagonal()(2) == 9);
                CHECK(S.diagonal()(3) == 9);
                CHECK(S.diagonal()(4) == 9);
                CHECK(S.diagonal()(5) == 9);
                CHECK(S.diagonal()(6) == 9);
                CHECK(S.diagonal()(7) == 9);
                CHECK(S.diagonal()(8) == 9);
                CHECK(S.diagonal()(9) == 9);
                CHECK(S.diagonal()(10) == 9);
                CHECK(S.diagonal()(11) == 9);

                // Marker states
                // m1
                CHECK(mu(12) == 1);
                CHECK(mu(13) == 1);
                CHECK(mu(14) == 1);
                CHECK(mu(15) == 1);
                CHECK(mu(16) == 1);
                CHECK(mu(17) == 1);
                CHECK(S.diagonal()(12) == 1);
                CHECK(S.diagonal()(13) == 1);
                CHECK(S.diagonal()(14) == 1);
                CHECK(S.diagonal()(15) == 1);
                CHECK(S.diagonal()(16) == 1);
                CHECK(S.diagonal()(17) == 1);

                // m2
                CHECK(mu(18) == 4);
                CHECK(mu(19) == 4);
                CHECK(mu(20) == 4);
                CHECK(mu(21) == 4);
                CHECK(mu(22) == 4);
                CHECK(mu(23) == 4);
                CHECK(S.diagonal()(18) == 4);
                CHECK(S.diagonal()(19) == 4);
                CHECK(S.diagonal()(20) == 4);
                CHECK(S.diagonal()(21) == 4);
                CHECK(S.diagonal()(22) == 4);
                CHECK(S.diagonal()(23) == 4);

                // savedMarkers 
                CHECK(savedMarkers.at(0).ID == 1);
                CHECK(savedMarkers.at(0).nMissed == 0);
                CHECK(savedMarkers.at(1).ID == 4);
                CHECK(savedMarkers.at(1).nMissed == 5);

                
            }
        }
    }
}