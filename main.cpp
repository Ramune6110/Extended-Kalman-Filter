#include <iostream>
#include <Eigen/Dense>
#include <cstdio>
#include "extended_kalman_filter.h"

using namespace std;
using namespace Eigen;

int main() 
{
    Extended_Kalman_Filter EKF;

    EKF.simulation();

    return 0;
}
