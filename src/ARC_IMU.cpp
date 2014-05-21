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

    cv::Matx33d
ARC_IMU::quaternion_to_rotation ( double* q )
{
    cv::Matx33d m(
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
    return m;
}		// -----  end of method ARC_IMU::quaternion_to_rotation  ----- 

ARC_IMU::ARC_IMU ( )
{
    /* TODO: get q from file input */
    double q[4] = { 
/*
            -0.0017705, 
            -0.58656, 
            0.0039022, 
            0.8099};
    */
        /* June Data */
                /*
            0.4823,
            0.4668,
            0.53082,
            -0.5174
        */
        /* Fall Data */
            0.50798,
            0.50067,
            0.49636,
            -0.49489
    };
/*
    Matx33d Rb2b(
            1, 0, 0,
            0, 0,-1,
            0, 1, 0
    );
  */
    cv::Matx33d Rb2b(
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
    );
    cv::Matx33d Rb2c = quaternion_to_rotation( q );
    cv::Matx33d Rb2c_transpose;
    transpose( Rb2c, Rb2c_transpose );
    Rc2b = Rb2b * Rb2c_transpose;
    //Rc2b = Rb2b ;
    /*
    Rc2b = Matx33d(
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
            );
            */
    return ;
}		// -----  end of method ARC_IMU::ARC_IMU  ----- 

    double
ARC_IMU::get_rotation_angle ( cv::Matx33d rot_mat )
{
    cv::Point2f obj2d( 320, 240 );
    cv::Point3f end3f;
    cv::Point2f end2f;
    cv::Matx33d rot_mat_invert;
    cv::Matx33d A_invert;

    // Convert obj2d to Cr.
    cv::Point3f Cr = poToCr( obj2d, rot_mat );

    // Reflect point in Cr over xy plane.
    cv::Matx31d Cr_start( Cr.x, Cr.y, Cr.z );
    cv::Matx31d Cr_end( Cr.x, Cr.y, -Cr.z );
    //
    // Convert endpoints to P_o
    invert( rot_mat, rot_mat_invert );
    invert( A, A_invert );

    cv::Matx31d pi_start = rot_mat_invert * Cr_start;
    cv::Matx31d pi_end = rot_mat_invert * Cr_end;

    // Calc angle in radians.
    double theta = atan2( pi_start(1)-pi_end(1), pi_start(0)-pi_end(0) ) ;

    return theta;
}		// -----  end of method ARC_IMU::get_rotation_angle  ----- 

    cv::Matx33d
ARC_IMU::calc_rotation_matrix ( cv::Point3f imu_data )
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
    cv::Matx33d rotation_matrix = quaternion_to_rotation( q );


    return rotation_matrix * Rc2b;
}		// -----  end of method ARC_IMU::calc_rotation_matrix  ----- 


    cv::Point3f
ARC_IMU::poToCr ( cv::Point2f pt2d, cv::Matx33d rot_mat )
{
    // TODO: use perspectiveTransform?
    //cout << "START" << endl;
    cv::Matx31d mat2d( pt2d.x, pt2d.y, 1 ) ;        // Convert point to 3x1 Mat
    //cout << "input: " << Mat(mat2d) << endl;
    cv::Matx31d hCi = A*mat2d;                      // Multiply by A matrix to get h_o
    //cout << "h_o: " << Mat(hCi) << endl;
    // Normalize
    cv::Matx31d hCi_norm;
    normalize( hCi, hCi_norm );                 // Normalize h_o
    //cout << "h_o_norm: " << Mat(hCi_norm) << endl;
    cv::Matx31d inRc = rot_mat * hCi_norm;          // Transform to C_r

    //Matx31d inRc = rot_mat * hCi;
    cv::Point3f ptRc = (cv::Point3f) cv::Mat(inRc);         // Convert C_r to Point
    //cout << ptRc << endl;

    return ptRc;
}		// -----  end of method ARC_IMU::2dToCr  ----- 
