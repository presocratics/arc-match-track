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
#include <iostream>
ARC_IMU::ARC_IMU ( Mat am )
{
    A = am;
}

    Point3f
ARC_IMU::2dToCr ( Point2f pt2d, Point3f imu_data )
{
    Point3f pt2d( pt2d.x, pt2d.y, 1 );
    Mat obj2d( pt2d );
    Mat hCi = A*obj2d;                           // 3d wrt current camera orientation.

    Mat R;                                      // Transform to initial camera orientation.
    //TODO convert to Cr.
    

    return ;
}		// -----  end of method ARC_IMU::2dToCr  ----- 


float calc_slope( Point2f obj2d, Point3f imu_data )
{

}

int main()
{
	using namespace std;
	return 0;
}

