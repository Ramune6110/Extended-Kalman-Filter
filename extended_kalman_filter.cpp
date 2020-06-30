#include <iostream>
#include <Eigen/Dense>
#include <cstdio>
#include <cmath>
#include <random>
#include "extended_kalman_filter.h"

#define PI 3.14159265359

using namespace std;
using namespace Eigen;

Extended_Kalman_Filter::Extended_Kalman_Filter() 
{
    // Time 
    ts = 0.0;
    dt = 0.1; 
    tf = 60.0;
    // State Vector [x y yaw]'
    xEst << 0, 0, 0;
    xTrue << 0, 0, 0;
    // Observation vector [x y yaw]'
    z << 0, 0, 0;
    // Covariance Matrix for motion
    Q << pow(0.1, 2.0), 0, 0,
         0, pow(0.1, 2.0), 0,
         0, 0, pow(toRadian(1), 2.0);
    // Covariance Matrix for observation 
    R << pow(1.5, 2.0), 0, 0,
         0, pow(1.5, 2.0), 0,
         0, 0, pow(toRadian(3), 2.0);
    // Simulation parameter
    Qsigma << pow(0.1, 2.0), 0,
              0, pow(toRadian(20), 2.0);
    Rsigma << pow(1.5, 2.0), 0, 0,
              0, pow(1.5, 2.0), 0,
              0, 0, pow(toRadian(3), 2.0);
    PEst << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;   
}

Extended_Kalman_Filter::~Extended_Kalman_Filter()
{
    cout << "Finish" << endl;
}

double Extended_Kalman_Filter::toRadian(double degree)
{
    // degree to radian
    double radian = degree / 180.0 * PI;
    return radian;
}

MatrixXd Extended_Kalman_Filter::doControl(double time) 
{
    // Calc Input Parameter

    double T = 10.0;
    double V = 1.0; 
    double yawrate = 5.0; 
    // Input
    u << V * (1 - exp(-time / T)), toRadian(yawrate) * (1 - exp(-time / T));
    
    return u;
}

MatrixXd Extended_Kalman_Filter::noise_distribution(MatrixXd u) 
{
    // gaussian distribution
    random_device rd{};
    mt19937 gen{rd()};
    normal_distribution<> gaussian_d{0, 1};

    u(0, 0) = u(0, 0) + gaussian_d(gen) * Qsigma(0, 0);
    u(1, 0) = u(1, 0) + gaussian_d(gen) * Qsigma(1, 1);

    return u;
}

MatrixXd Extended_Kalman_Filter::Observation(MatrixXd x) 
{
    // gaussian distribution
    random_device rd{};
    mt19937 gen{rd()};
    normal_distribution<> gaussian_d{0, 1};

    z(0, 0) = x(0, 0) + gaussian_d(gen) * Rsigma(0, 0);
    z(1, 0) = x(1, 0) + gaussian_d(gen) * Rsigma(1, 1);
    z(2, 0) = x(2, 0) + gaussian_d(gen) * Rsigma(2, 2);

    return z;
}

MatrixXd Extended_Kalman_Filter::model(MatrixXd x, MatrixXd u)
{
    
    A << 1.0, 0, 0,
         0, 1.0, 0,
         0, 0, 1.0;
    
    B << dt * cos(x(2, 0)), 0,
         dt * sin(x(2, 0)), 0,
         0, dt;
    
    return x = A * x + B * u;
}

MatrixXd Extended_Kalman_Filter::jacobF(MatrixXd x, MatrixXd u)
{
    jF << 1, 0, -dt * u(0, 0) * sin(x(2, 0)),
          0, 1, dt * u(0, 0) * cos(x(2, 0)),
          0, 0, 1;
    
    return jF;
}

MatrixXd Extended_Kalman_Filter::measuement(MatrixXd x) 
{
    C << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    return H * x;
}


MatrixXd Extended_Kalman_Filter::jacobH()
{
    jF << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;
    
    return jF;
}

void Extended_Kalman_Filter::simulation()
{
    // save data
    FILE *fp;
    if ((fp = fopen("data.txt", "w")) == NULL) {
        printf("Error\n");
        exit(1);
    }
    fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\n", ts, xTrue(0, 0), xTrue(1, 0), xEst(0, 0), xEst(1, 0));
    // main loop
    for (ts; ts <= tf; ts += dt) {
        // Input
        u     = doControl(ts);
        xTrue = model(xTrue, u);
        // noise distribution
        u     = noise_distribution(u);
        z     = Observation(xTrue);
        // ------Extended Kalman Filter --------
        // Prediction step
        xPred = model(xEst, u);   
        F     = jacobF(xPred, u);  
        PPred = F * PEst * F.transpose() + Q;
        // Filtering step
        H     = jacobH(); 
        K     = PPred * H.transpose() * (H * PPred * H.transpose() + R).inverse();
        xEst  = xPred + K * (z - measuement(xPred)); 
        PEst  = (Matrix3d::Identity() - K * H) * PPred;

        fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\n", ts, xTrue(0, 0), xTrue(1, 0), xEst(0, 0), xEst(1, 0));
    }

    fclose(fp);
}
