
#include "ballistic.h"
#include <iostream>
#include <cmath>

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// 
// BallisticProcessModel
// 
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void BallisticProcessModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & f)
{
    const double & p0   = param.p0;
    const double & M    = param.M;
    const double & R    = param.R;
    const double & L    = param.L;
    const double & T0   = param.T0;
    const double & g    = param.g;

    double e1, e2, e3, exp, d, a;
    Eigen::VectorXd mu(3);

    f.resize(3);

    // TODO: mean function
    e1 = M*p0/R;
    e2 = 1/(T0-L*x(0));
    e3 = 1 - L*x(0)/T0;
    exp = g*M/R/L;
    d = 0.5*e1*e2*pow(e3,exp)*pow(x(1),2)*x(2);
    a = d - g;
    f(0) = x(1);
    f(1) = a;
    f(2) = 0;
}

void BallisticProcessModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & f, Eigen::MatrixXd & SQ)
{
    // TODO: mean function
    BallisticProcessModel pm;   // Call the already implemented one!
    pm(x, u, param, f);

    // TODO: upper Cholesky factor of process covariance
    SQ.resize(3,3);
    SQ.fill(0.);
    SQ.diagonal() << 0.0, sqrt(1.0e-20), sqrt(25.0e-12);

}

void BallisticProcessModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & f, Eigen::MatrixXd & SQ, Eigen::MatrixXd & dfdx)
{
    const double & p0   = param.p0;
    const double & M    = param.M;
    const double & R    = param.R;
    const double & L    = param.L;
    const double & T0   = param.T0;
    const double & g    = param.g;

    double p, T, dpdz, dzdh, dddT, dTdh, dddp, dpdh;

    // TODO: mean function
    // TODO: upper Cholesky factor of process covariance
    BallisticProcessModel pm;
    pm(x, u, param, f, SQ);

    // TODO: Jacobian of mean dynamics w.r.t x.   
    // Note: x = [h, v, c] 
    // Pressure and temp calc
    p = p0*pow(1 - L*x(0)/T0, g*M/R/L);
    T = T0 - L*x(0);

    // Chain rule of pressure function
    dpdz = g*M/R/L*p0*pow(1-L*x(0)/T0, g*M/R/L - 1);
    dzdh = -L/T0;

    // Chain rule elements for J(1,0)
    dddT = -0.5*p*M/R/pow(T,2)*pow(x(1),2)*x(2);
    dTdh = -L;
    dddp = 0.5*M/R/T*pow(x(1),2)*x(2);
    dpdh =  dpdz*dzdh;

    dfdx.resize(3,3);

    dfdx(0,0) = 0;
    dfdx(0,1) = 1;
    dfdx(0,2) = 0;

    dfdx(1,0) = dddT*dTdh + dddp*dpdh;
    dfdx(1,1) = p*M/R/T*x(1)*x(2);
    dfdx(1,2) = 0.5*p*M/R/T*pow(x(1),2);

    dfdx(2,0) = 0; 
    dfdx(2,1) = 0;
    dfdx(2,2) = 0;

}


// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// 
// BallisticMeasurementModel
// 
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------

void BallisticMeasurementModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & h)
{
    const double & r1   = param.r1;
    const double & r2   = param.r2;

    h.resize(1);
    // TODO: mean function
    h(0) = sqrt(pow(r1,2) + pow(x(0) - r2,2));
}

void BallisticMeasurementModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & h, Eigen::MatrixXd & SR)
{
    // TODO: mean function
    BallisticMeasurementModel mm;
    mm(x, u, param, h);

    // TODO: upper Cholesky factor of measurement covariance
    SR.resize(1,1);
    SR << 50;
}

void BallisticMeasurementModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & h, Eigen::MatrixXd & SR, Eigen::MatrixXd & dhdx)
{
    const double & r1   = param.r1;
    const double & r2   = param.r2;

    double a, b, dhda, dadb, dbdx;

    // TODO: mean function
    // TODO: upper Cholesky factor of measurement covariance
    BallisticMeasurementModel mm;
    mm(x, u, param, h, SR);

    // TODO: Jacobian of mean measurement w.r.t x.
    dhda = 0.5*pow(pow(r1,2) + pow(x(0)-r2,2),-0.5);
    dadb = 2*(x(0)-r2);
    dbdx = 1.0;

    dhdx.resize(1,3);
    dhdx(0,0) = dhda*dadb*dbdx;
    dhdx(0,1) = 0;
    dhdx(0,2) = 0;
}







