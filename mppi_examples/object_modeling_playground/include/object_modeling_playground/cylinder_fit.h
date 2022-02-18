#pragma once
#include <Eigen/Core>
#include <cmath>

using namespace Eigen;

int pre_process(int n, MatrixXd& points, MatrixXd& std_points)
{  
    // n: number of point
    // points: raw points,  shape = n x #ptr
    // std_points: processed points

    Vector3d avg;
    avg = points.colwise().mean();
    
    points.rowwise() += avg;



    return (n+1);
}

