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

    void
ARC_IMU::poEndpoints ( Point2f obj2d, Mat rot_mat, Point2f* out_line )
{
    Mat rot_mat_transpose;
    transpose( rot_mat, rot_mat_transpose );
    Point2f obj_end( obj2d.x, 480 );
    Point3f Cr = poToCr( obj2d, rot_mat );
    Point3f Cr_end_estimate = poToCr( obj_end, rot_mat );
    Point3f Cr_start(Cr.x, 0, 1 );
    Point3f Cr_end(Cr.x, Cr_end_estimate.y, 1 );

    Point2f po_start = rot_mat_transpose * Cr_start;
    Point2f po_end = rot_mat_transpose * Cr_end;
    return ;
}		// -----  end of method ARC_IMU::poEndpoints  ----- 

    Mat
ARC_IMU::calc_rotation_matrix ( Point3f imu_data )
{
    double phi, theta, psi;
    double hphi, hthe, hpsi;
    phi = imu_data.x;
    theta = imu_data.y;
    psi = imu_data.z;
    hphi = phi/2;
    hthe = theta/2;
    hpsi = psi/2;
    /*
    Mat rotation_matrix = ( Mat_<double>(3,3)
            << cos(psi)*cos(theta), sin(psi)*cos(theta), -sin(theta),
               -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi),
               cos(theta)*sin(phi),
               sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi), -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi),
               cos(theta)*cos(phi) );
               */
    double q[4] = {
                   sin(hphi)*cos(hthe)*cos(hpsi) - cos(hphi)*sin(hthe)*sin(hpsi),
                   cos(hphi)*sin(hthe)*cos(hpsi) + sin(hphi)*cos(hthe)*sin(hpsi),
                   cos(hphi)*cos(hthe)*sin(hpsi) - sin(hphi)*sin(hthe)*cos(hpsi),
                   cos(hphi)*cos(hthe)*cos(hpsi) + sin(hphi)*sin(hthe)*sin(hpsi)
    };
    Mat rotation_matrix = ( Mat_<double>(3,3)
            << pow( q[3], 2 )+pow( q[0], 2 )-pow( q[1], 2 )-pow( q[2], 2 ),
                2*(q[0]*q[1]-q[3]*q[2]),
                2*(q[3]*q[1]+q[0]*q[2]),

                2*(q[0]*q[1]+q[3]*q[2]),
                pow( q[3], 2 )-pow( q[0], 2 )+pow( q[1], 2 )-pow( q[2], 2 ),
                2*(q[1]*q[2]-q[3]*q[0]),

                2*(q[0]*q[2]-q[3]*q[1]),
                2*(q[3]*q[0]+q[1]*q[2]),
                pow( q[3], 2 )-pow( q[0], 2 )-pow( q[1], 2 )+pow( q[2], 2 )
            );


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

