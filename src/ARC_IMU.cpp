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
ARC_IMU::poEndpoints ( Point2f obj2d, Matx33d rot_mat, Point2f* out_line )
{
    Point2f start2f, end2f;
    Point3f start3f, end3f;

    Matx33d rot_mat_invert;
    Matx33d A_invert;
    // P_o to Cr of chosen point and similar point at bottom of frame.
    Point2f obj_end( obj2d.x, 480 );
    Point3f Cr = poToCr( obj2d, rot_mat );
    Point3f Cr_end_estimate = poToCr( obj_end, rot_mat );
    // Using Cr.x create endpoints at top and bottom of Cr frame.
    // TODO: divide 1 by norm
    Matx31d Cr_start( Cr.x, 0, 1 );
    Matx31d Cr_end( Cr.x, Cr_end_estimate.y, 1 );
    // Convert endpoints to P_o
    //transpose( rot_mat, rot_mat_transpose );
    invert( rot_mat, rot_mat_invert );
    invert( A, A_invert );

    Matx31d po_start = A_invert * (rot_mat_invert * Cr_start);
    Matx31d po_end = A_invert * (rot_mat_invert * Cr_end );
    
    start3f = (Point3f) Mat(po_start);
    start2f = Point2f( start3f.x, start3f.y );

    end3f = (Point3f) Mat(po_end);
    end2f = Point2f( end3f.x, end3f.y );

    out_line[0] = start2f;
    out_line[1] = end2f;
    return ;
}		// -----  end of method ARC_IMU::poEndpoints  ----- 

    Matx33d
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
    Matx33d rotation_matrix (
               cos(psi)*cos(theta), sin(psi)*cos(theta), -sin(theta),
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
    Matx33d rotation_matrix(
                pow( q[3], 2 )+pow( q[0], 2 )-pow( q[1], 2 )-pow( q[2], 2 ),
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
ARC_IMU::poToCr ( Point2f pt2d, Matx33d rot_mat )
{
    // TODO: use perspectiveTransform?
    Matx31d mat2d( pt2d.x, pt2d.y, 1 ) ;        // Convert point to 3x1 Mat
    Matx31d hCi = A*mat2d;                      // Multiply by A matrix to get h_o
    /*
    // Normalize
    Matx31d hCi_norm;
    normalize( hCi, hCi_norm );                 // Normalize h_o

    Matx31d inRc = rot_mat * hCi_norm;          // Transform to C_r
    */
    Matx31d inRc = rot_mat * hCi;
    Point3f ptRc = (Point3f) Mat(inRc);         // Convert C_r to Point

    return ptRc;
}		// -----  end of method ARC_IMU::2dToCr  ----- 


/*
float calc_slope( Point2f obj2d, Mat rot_mat )
{

}
*/

