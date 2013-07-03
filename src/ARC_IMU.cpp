// =====================================================================================
//
//       Filename:  ARC_IMU.cpp
//
//    Description:  Calculates slopes and transforms using IMU data.
//
//        Version:  1.0
//        Created:  07/02/2013 03:28:40 PM
//       Revision:  none
//       Compiler:  gcc
//
//         Author:  Martin Miller (), miller7@illinois.edu
//   Organization:  Aerospace Robotics and Control Lab
//
// =====================================================================================

#include "ARC_IMU.hpp"
#include "config.hpp"
#include <iostream>

    Mat
ARC_IMU::calc_rotation_matrix ( Point3f imu_data )
{
    double phi, theta, psi;
    phi = imu_data.x;
    theta = imu_data.y;
    psi = imu_data.z;
    Mat rotation_matrix = ( Mat_<double>(3,3)
            << cosf(psi)*cosf(theta), sinf(psi)*cosf(theta), -sinf(theta),
               -sinf(psi)*cosf(phi)+cosf(psi)*sinf(theta)*sinf(phi), cosf(psi)*cosf(phi)+sinf(psi)*sinf(theta)*sinf(phi),
               cosf(theta)*sinf(phi),
               sinf(psi)*sinf(phi)+cosf(psi)*sinf(theta)*cosf(phi), -cosf(psi)*sinf(phi)+sinf(psi)*sinf(theta)*cosf(phi),
               cosf(theta)*cos(phi) );

    return rotation_matrix;
}		// -----  end of method ARC_IMU::calc_rotation_matrix  ----- 


    Point3f
ARC_IMU::poToCr ( Point2f pt2d, Mat rot_mat )
{
    Mat mat2d = ( Mat_<double>(3, 1) << pt2d.x, pt2d.y, 1 ) ;
    Mat hCi = A*mat2d;                           // 3d wrt current camera orientation.

    Mat inRc = rot_mat * hCi;
    //TODO convert to Cr.
    Point3f ptRc = (Point3f) inRc;
    

    return ptRc;
}		// -----  end of method ARC_IMU::2dToCr  ----- 


/*
float calc_slope( Point2f obj2d, Mat rot_mat )
{

}
*/

