#include <Eigen/Core>
#include <cassert>

#include "gaussian.hpp"


// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// 
// conditionGaussianOnMarginal
// 
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------

void conditionGaussianOnMarginal(const Eigen::VectorXd & muyxjoint, const Eigen::MatrixXd & Syxjoint, const Eigen::VectorXd & y, Eigen::VectorXd & muxcond, Eigen::MatrixXd & Sxcond)
{
    // TODO: Copy from Lab 4
    // Trying to find x state mean and covariance based on previous distribution and new measurements
    // Derive state and measurement numbers
    int ny = y.rows(),
        nx = muyxjoint.rows() - y.rows();

    // Extract the useful things
    Eigen::MatrixXd mux = muyxjoint.tail(nx),
                    muy = muyxjoint.head(ny),
                    S1  = Syxjoint.topLeftCorner(ny, ny),
                    S2  = Syxjoint.topRightCorner(ny, nx),
                    S3  = Syxjoint.bottomRightCorner(nx, nx);


    // Calculate the new state mean and covariance
    muxcond = mux + S2.transpose()*S1.triangularView<Eigen::Upper>().transpose().solve((y-muy));
    Sxcond  = S3;
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// 
// pythagoreanQR
// 
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------

void pythagoreanQR(const Eigen::MatrixXd & S1, const Eigen::MatrixXd & S2, Eigen::MatrixXd & S){

    // Implement some checks to ensure inputs are appropriate

    // A is vertical concatenation of S1 and S2
    int n = S1.cols();
    Eigen::MatrixXd A(S1.rows()+S2.rows(), n);
    A << S1, S2;

    // I don't understand how I implemented the HouseholderQR? It's like a variable type? idk ask Time
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
    qr.compute(A);

    // Q Seemingly not needed?
    Eigen::MatrixXd Q = qr.householderQ();

    Eigen::MatrixXd QR = qr.matrixQR();

    Eigen::MatrixXd R = qr.matrixQR().template triangularView<Eigen::Upper>();

    S = R.block(0,0,n,n);

}







