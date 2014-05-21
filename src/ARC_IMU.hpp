#ifndef  ARC_IMU_INC
#define  ARC_IMU_INC
#include <cv.h>
#include <iostream>
#include "config.hpp"

// =====================================================================================
//        Class:  ARC_IMU
//  Description:  Calculates slopes and transforms using IMU data.
// =====================================================================================
class ARC_IMU
{
    public:
        // ====================  LIFECYCLE     ======================================= 
        ARC_IMU ( );                             // constructor 

        // ====================  ACCESSORS     ======================================= 
        cv::Matx33d get_A()
        {
            return A;
        }

        // ====================  MUTATORS      ======================================= 
        void set_A( cv::Matx33d am )
        {
            A = am;
        }

        // ====================  OPERATORS     ======================================= 

        // ====================  METHODS       ======================================= 
        cv::Point3f poToCr( cv::Point2f obj2d, cv::Matx33d rot_mat );
        cv::Point2f CrTopo( cv::Point3f obj3d, cv::Matx33d rot_mat );
        double get_rotation_angle( cv::Matx33d rot_mat ) ;
        cv::Matx33d calc_rotation_matrix( cv::Point3f imu_data );
        double theta_to_slope( double theta )
        {
            return tan( theta );
        }

    protected:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 

    private:
        // ====================  METHODS       ======================================= 
        cv::Matx33d quaternion_to_rotation ( double* q );

        // ====================  DATA MEMBERS  ======================================= 
        cv::Matx33d A;
        cv::Matx33d Rc2b;

}; // -----  end of class ARC_IMU  ----- 

#endif   // ----- #ifndef ARC_IMU_INC  ----- 
